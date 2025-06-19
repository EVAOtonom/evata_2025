import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion
from ament_index_python.packages import get_package_share_directory
import math
import time
import json
import os

class ControlNode(Node):
    def __init__(self):
        super().__init__('new_control')
        self._init_state_variables()
        self._load_waypoints()
        self._setup_communication()
        self._setup_navigation()
        self.create_timer(0.5, self.check_waypoint_distance)

    def _init_state_variables(self):
        self.current_pose = None
        self.current_yaw = 0.0
        self.mode = 'normal'
        self.distance_threshold = 2.0
        
        # Goal management
        self.original_goal = None
        self.forward_goal = None
        self.active_goal_handle = None
        self.nearest_waypoint = None
        
        # Sign processing
        self.last_sign_processed = None
        self.sign_processing = False
        
        # Motion control
        self.motion_enabled = True
        self.last_cmd_vel = Twist()

    def _load_waypoints(self):
        package_path = get_package_share_directory('evata_sim')
        waypoints_file = os.path.join(package_path, 'waypoint', 'waypoint.txt')
        self.waypoints = self.load_waypoints(waypoints_file)

    def _setup_communication(self):
        # Subscribers
        self.create_subscription(String, '/detected_signs', self.sign_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        # Publishers
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.command_pub = self.create_publisher(String, 'nav_cmd', 10)

    def _setup_navigation(self):
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_client.wait_for_server()

    def load_waypoints(self, file_path):
        points = []
        with open(file_path, 'r') as f:
            for line in f:
                if line.strip() and not line.startswith("#"):
                    x, y, lat, lon = map(float, line.strip().split())
                    points.append({'x': x, 'y': y, 'lat': lat, 'lon': lon})
        return points

    def vel_callback(self, msg):
        self.last_cmd_vel = msg
        
        if not self.motion_enabled:
            stop_msg = Twist()
            self.vel_pub.publish(stop_msg)
        else:
            self.vel_pub.publish(msg)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        self.current_yaw = yaw

    def imu_callback(self, msg):
        orientation_q = msg.orientation
        _, _, yaw = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        self.current_yaw = yaw

    def sign_callback(self, msg):
        try:
            data = json.loads(msg.data)

            if 'durak' in data:
                data.pop('durak')
                if not data:
                    return

            if self.sign_processing or (self.last_sign_processed == data):
                return

            if not (self.current_pose and self.mode == 'normal'):
                return

            if self.active_goal_handle:
                self.original_goal = self._get_goal_from_handle(self.active_goal_handle)

            self.sign_processing = True
            self.last_sign_processed = data

            self._process_sign(data)

        except Exception as e:
            self.get_logger().error(f"❌ JSON parse hatası: {e}")

    def _process_sign(self, data):
        if 'sagyon' in data:
            self._handle_direction_sign('sagyon', self.send_nearest_right_waypoint)
        elif 'solyon' in data:
            self._handle_direction_sign('solyon', self.send_nearest_left_waypoint)
        elif any(sign in data for sign in ['girme', 'kazi', 'notraffic']):
            self._handle_no_entry_sign(data)
        elif any(sign in data for sign in ['sagdonulmez', 'soladonulmez']):
            self._handle_no_turn_sign(data)
        elif any(sign in data for sign in ['ilerivesag', 'ilerivesol']):
            self._handle_straight_sign()

    def _handle_direction_sign(self, sign_type, waypoint_function):
        self.get_logger().info(f"🛑 '{sign_type}' levhası algılandı.")
        self.command_pub.publish(String(data='red'))
        time.sleep(0.2)
        self.mode = 'waypoint'
        waypoint_function()

    def _handle_no_entry_sign(self, data):
        if hasattr(data, 'distance') and data['distance'] <= 5.0:
            self.get_logger().info("🛑 Girilmez levhası 5m içinde - İşlem yapılmıyor")
            return
        
        self.get_logger().info("🛑 Girilmez türü levha algılandı.")
        self.command_pub.publish(String(data='red'))
        time.sleep(0.2)
        self.mode = 'waypoint'
        self.send_nearest_noentry_waypoint()

    def _handle_no_turn_sign(self, data):
        if hasattr(data, 'distance') and data['distance'] <= 6.0:
            self.get_logger().info("🛑 Dönülmez levhası 6m içinde - İşlem yapılmıyor")
            return
        
        self.get_logger().info("➡️ Dönülmez levhası algılandı, 10 metre ilerleniyor.")
        self._execute_forward_movement()

    def _handle_straight_sign(self):
        self.get_logger().info("➡️ Düz git levhası algılandı, 10 metre ilerleniyor.")
        self._execute_forward_movement()

    def _execute_forward_movement(self):
        self.command_pub.publish(String(data='red'))
        time.sleep(0.2)
        self.mode = 'forward'
        
        if self.active_goal_handle:
            self.original_goal = self._get_goal_from_handle(self.active_goal_handle)
            cancel_future = self.active_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda _: self.go_forward_and_return())
        else:
            self.go_forward_and_return()

    def go_forward_and_return(self, distance=20.0):
        if not self.current_pose:
            self.get_logger().warn("❌ Geçerli pozisyon yok. Hareket iptal edildi.")
            return

        x = self.current_pose.position.x
        y = self.current_pose.position.y
        yaw = self.current_yaw

        forward_x = x + distance * math.cos(yaw)
        forward_y = y + distance * math.sin(yaw)
        self.forward_goal = (forward_x, forward_y)

        self.get_logger().info(f"➡️ {distance} metre ileri hedef: ({forward_x:.2f}, {forward_y:.2f})")

        goal_msg = self._create_navigation_goal(forward_x, forward_y)
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.original_goal = goal_msg
        send_goal_future.add_done_callback(self.forward_goal_response_callback)

    def _create_navigation_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0
        return goal_msg

    def forward_goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            self.original_goal = self._get_goal_from_handle(goal_handle)

        self.get_logger().info("🚗 İleri hedefe gidiliyor (20 metre)...")
        self.active_goal_handle = goal_handle

    def _send_nearest_side_waypoint(self, angle_range, side_name):
        if not self.current_pose:
            return

        x, y = self.current_pose.position.x, self.current_pose.position.y
        yaw = self.current_yaw
        side_waypoints = []

        for wp in self.waypoints:
            wp_x, wp_y = wp['x'], wp['y']
            dx, dy = wp_x - x, wp_y - y
            distance = math.hypot(dx, dy)
            
            if distance < 0.01:
                continue

            angle_to_wp = math.atan2(dy, dx)
            angle_diff = math.atan2(math.sin(angle_to_wp - yaw), math.cos(angle_to_wp - yaw))
            angle_deg = math.degrees(angle_diff)

            if angle_range[0] < angle_deg < angle_range[1]:
                side_waypoints.append((wp_x, wp_y))

        if not side_waypoints:
            self.get_logger().warn(f"⚠️ {side_name} bölgede waypoint yok!")
            return

        self.nearest_waypoint = min(side_waypoints, key=lambda p: math.hypot(p[0]-x, p[1]-y))
        self.get_logger().info(f"📍 En yakın {side_name} waypoint: {self.nearest_waypoint}")

        goal_msg = self._create_navigation_goal(self.nearest_waypoint[0], self.nearest_waypoint[1])
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def send_nearest_right_waypoint(self):
        self._send_nearest_side_waypoint(angle_range=(-100, -20), side_name="SAĞ-ÖN")

    def send_nearest_left_waypoint(self):
        self._send_nearest_side_waypoint(angle_range=(20, 100), side_name="SOL-ÖN")

    def send_nearest_noentry_waypoint(self):
        if not self.current_pose:
            return

        x, y = self.current_pose.position.x, self.current_pose.position.y

        right_waypoints = self._get_side_waypoints((-100, -20))
        left_waypoints = self._get_side_waypoints((20, 100))

        all_waypoints = right_waypoints + left_waypoints
        
        if not all_waypoints:
            self.get_logger().warn("❌ Girilmez bölge için waypoint yok!")
            return

        nearest = min(all_waypoints, key=lambda p: math.hypot(p[0]-x, p[1]-y))
        self.get_logger().info(f"📍 En yakın GİRİLMEZ waypoint: {nearest}")
        
        goal_msg = self._create_navigation_goal(nearest[0], nearest[1])
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def _get_side_waypoints(self, angle_range):
        if not self.current_pose:
            return []

        x, y = self.current_pose.position.x, self.current_pose.position.y
        yaw = self.current_yaw
        waypoints = []

        for wp in self.waypoints:
            wp_x, wp_y = wp['x'], wp['y']
            dx, dy = wp_x - x, wp_y - y
            distance = math.hypot(dx, dy)
            
            if distance < 0.01:
                continue

            angle_to_wp = math.atan2(dy, dx)
            angle_diff = math.atan2(math.sin(angle_to_wp - yaw), math.cos(angle_to_wp - yaw))
            angle_deg = math.degrees(angle_diff)

            if angle_range[0] < angle_deg < angle_range[1]:
                waypoints.append((wp_x, wp_y))

        return waypoints

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("❌ Waypoint hedefi reddedildi.")
            self.restore_previous_goal()
            return

        self.get_logger().info("🚗 Waypoint'e gidiliyor...")
        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result()
        self.sign_processing = False
        
        if result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("🛑 Waypoint hedefi iptal edildi.")
        elif result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("✅ Waypoint'e ulaşıldı.")
            self.last_sign_processed = None
            
            if self.mode == 'forward':
                self._handle_forward_completion()
            else:
                self._handle_waypoint_completion()

    def _handle_forward_completion(self):
        self.get_logger().info("⏳ 10 metre ileri gidildi, orijinal hedefe dönülüyor...")
        
        if self.original_goal:
            self.get_logger().info("↩️ Orijinal hedefe geri dönülüyor...")
            send_goal_future = self.nav_client.send_goal_async(self.original_goal)
            send_goal_future.add_done_callback(self.goal_response_callback)
            self.original_goal = None
        else:
            self.get_logger().info("ℹ️ live_gps'e devam ediliyor...")
            self.command_pub.publish(String(data='green'))
        
        self.mode = 'normal'

    def _handle_waypoint_completion(self):
        self.get_logger().info("⏳ Önceki hedefe geri dönülüyor...")
        self.command_pub.publish(String(data='green'))
        self.mode = 'normal'

    def check_waypoint_distance(self):
        if (self.mode not in ['waypoint', 'forward']) or not self.current_pose:
            return

        x = self.current_pose.position.x
        y = self.current_pose.position.y
        
        target = self._get_current_target()
        if not target:
            return

        distance = math.hypot(target[0] - x, target[1] - y)
        
        if distance < self.distance_threshold:
            self.get_logger().info(f"🛑 {distance:.2f} metre kala iptal ediliyor...")
            
            if self.active_goal_handle:
                future = self.active_goal_handle.cancel_goal_async()
                future.add_done_callback(self._handle_cancel_complete)

    def _get_current_target(self):
        if self.mode == 'waypoint' and hasattr(self, 'nearest_waypoint'):
            return self.nearest_waypoint
        elif self.mode == 'forward' and self.forward_goal:
            return self.forward_goal
        return None

    def _handle_cancel_complete(self, _):
        self.get_logger().info("✅ Hedef iptal edildi")
        self.active_goal_handle = None
        
        if self.original_goal:
            self.get_logger().info("↩️ Orijinal hedefe geri dönülüyor...")
            send_goal_future = self.nav_client.send_goal_async(self.original_goal)
            send_goal_future.add_done_callback(self.goal_response_callback)
            self.original_goal = None
        else:
            self.get_logger().info("ℹ️ live_gps'e devam ediliyor...")
            self.command_pub.publish(String(data='green'))
        
        self.mode = 'normal'
        self.sign_processing = False
        self.last_sign_processed = None

    def restore_previous_goal(self):
        if self.original_goal:
            self.get_logger().info("Önceki hedef geri yükleniyor...")
            self.nav_client.send_goal_async(self.original_goal)

    def _get_goal_from_handle(self, goal_handle):
        if goal_handle is None:
            return None
        if hasattr(goal_handle, 'request'):
            return goal_handle.request
        return goal_handle.goal if hasattr(goal_handle, 'goal') else None


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()