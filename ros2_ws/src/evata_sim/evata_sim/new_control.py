import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import Imu
import math
import time
from tf_transformations import euler_from_quaternion
import numpy as np
import json  # JSON formatında subscirbe için

            
def xy_to_latlon(x, y, ref_lat, ref_lon):
    delta_lat = y / 111320
    delta_lon = x / (40075000 * math.cos(math.radians(ref_lat)) / 360)
    lat = ref_lat + delta_lat
    lon = ref_lon + delta_lon
    return lat, lon

class ControlNode(Node):
    def __init__(self):
        super().__init__('new_control')

        self.waypoints = self.load_waypoints('/home/dilara/ros2_ws/src/evata_sim/evata_sim/waypoint.txt')
        self.current_pose = None
        self.current_yaw = 0.0
        self.mode = 'normal'
        self.distance_threshold = 2.0
        self.saved_goal = None
        self.motion_enabled = True
        self.last_cmd_vel = Twist()  # Son hız komutunu saklamak için
        self.original_goal = None
        self.forward_timer = None


        self.create_subscription(String, '/detected_signs', self.sign_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.command_pub = self.create_publisher(String, 'nav_cmd', 10)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_client.wait_for_server()

        self.create_timer(0.5, self.check_waypoint_distance)
        self.active_goal_handle = None
        
    def vel_callback(self, msg):
        # Gelen hız komutunu kaydet
        self.last_cmd_vel = msg
        
        # Eğer hareket izni yoksa, hız komutunu sıfırla
        if not self.motion_enabled:
            stop_msg = Twist()
            self.vel_pub.publish(stop_msg)
        else:
            # Hareket izni varsa, gelen komutu olduğu gibi ilet
            self.vel_pub.publish(msg)
            
    def go_forward_and_return(self, distance=10.0):
        if not self.current_pose:
            self.get_logger().warn("❌ Geçerli pozisyon yok. Hareket iptal edildi.")
            return

        x = self.current_pose.position.x
        y = self.current_pose.position.y
        yaw = self.current_yaw

        forward_x = x + distance * math.cos(yaw)
        forward_y = y + distance * math.sin(yaw)

        self.get_logger().info(f"➡️ {distance} metre ileri hedef: ({forward_x:.2f}, {forward_y:.2f})")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = forward_x
        goal_msg.pose.pose.position.y = forward_y
        goal_msg.pose.pose.orientation.w = 1.0

        self.forward_goal = goal_msg

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.forward_goal_response_callback)
        self.forward_timer = self.create_timer(25.0, self.forward_timer_callback)
        
    def forward_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("❌ İleri hedef reddedildi.")
            return

        self.get_logger().info("🚗 İleri hedefe gidiliyor (25 saniye)...")
        self.active_goal_handle = goal_handle


    def forward_timer_callback(self):
        self.forward_timer.cancel()

        self.get_logger().info("⏱️ 25 saniye doldu, ileri hedef iptal ediliyor...")

        if self.active_goal_handle and self.active_goal_handle.status == GoalStatus.STATUS_EXECUTING:
            future = self.active_goal_handle.cancel_goal_async()

            def on_cancel_done(_):
                self.get_logger().info("🛑 Hedef iptal edildi.")
                self.command_pub.publish(String(data='green'))
                self.get_logger().info("🟢 'green' komutu yayınlandı (live_gps aktif).")
                self.mode = 'normal'
                self.original_goal = None
                self.active_goal_handle = None

            future.add_done_callback(on_cancel_done)
        else:
            self.command_pub.publish(String(data='green'))
            self.get_logger().info("🟢 'green' komutu yayınlandı (live_gps aktif).")
            self.mode = 'normal'
            self.original_goal = None
            self.active_goal_handle = None


    def send_nearest_right_waypoint(self):
        if not self.current_pose:
            return

        x = self.current_pose.position.x
        y = self.current_pose.position.y
        yaw = self.current_yaw

        right_waypoints = []

        for wp_x, wp_y in self.waypoints:
            wp_x = wp['x']
            wp_y = wp['y']
            dx = wp_x - x
            dy = wp_y - y
            distance = math.hypot(dx, dy)
            if distance < 0.01:
                continue

            angle_to_wp = math.atan2(dy, dx)
            angle_diff = angle_to_wp - yaw
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            angle_deg = math.degrees(angle_diff)

            if -100 < angle_deg < -20:
                right_waypoints.append((wp_x, wp_y))

        self.nearest_waypoint = min(right_waypoints, key=lambda p: math.hypot(p[0] - x, p[1] - y))
        self.get_logger().info(f"📍 En yakın SAĞ-ÖN waypoint: {self.nearest_waypoint}")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.nearest_waypoint[0]
        goal_msg.pose.pose.position.y = self.nearest_waypoint[1]
        goal_msg.pose.pose.orientation.w = 1.0

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def send_nearest_left_waypoint(self):
        if not self.current_pose:
            return

        x = self.current_pose.position.x
        y = self.current_pose.position.y
        yaw = self.current_yaw

        left_waypoints = []

        for wp_x, wp_y in self.waypoints:
            wp_x = wp['x']
            wp_y = wp['y']
            dx = wp_x - x
            dy = wp_y - y
            distance = math.hypot(dx, dy)
            if distance < 0.01:
                continue

            angle_to_wp = math.atan2(dy, dx)
            angle_diff = angle_to_wp - yaw
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            angle_deg = math.degrees(angle_diff)

            if 20 < angle_deg < 100:
                left_waypoints.append((wp_x, wp_y))

        self.nearest_waypoint = min(left_waypoints, key=lambda p: math.hypot(p[0] - x, p[1] - y))
        self.get_logger().info(f"📍 En yakın SOL-ÖN waypoint: {self.nearest_waypoint}")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.nearest_waypoint[0]
        goal_msg.pose.pose.position.y = self.nearest_waypoint[1]
        goal_msg.pose.pose.orientation.w = 1.0

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)


    def send_nearest_noentry_waypoint(self):
        if not self.current_pose:
            return

        x = self.current_pose.position.x
        y = self.current_pose.position.y
        yaw = self.current_yaw

        right_waypoints = []
        left_waypoints = []

        for wp_x, wp_y in self.waypoints:
            wp_x = wp['x']
            wp_y = wp['y']
            dx = wp_x - x
            dy = wp_y - y
            distance = math.hypot(dx, dy)
            if distance < 0.01:
                continue

            angle_to_wp = math.atan2(dy, dx)
            angle_diff = angle_to_wp - yaw
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            angle_deg = math.degrees(angle_diff)

            if (-100 < angle_deg < -20) or (20 < angle_deg < 100):
                right_waypoints.append((wp_x, wp_y))
                left_waypoints.append((wp_x, wp_y))

        combined_waypoints = right_waypoints + left_waypoints
        if not combined_waypoints:
            self.get_logger().warn("❌ No valid waypoints found for 'girme'")
            return

        self.nearest_waypoint = min(combined_waypoints, key=lambda p: math.hypot(p[0] - x, p[1] - y))
        self.get_logger().info(f"📍 En yakın waypoint: {self.nearest_waypoint}")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.nearest_waypoint[0]
        goal_msg.pose.pose.position.y = self.nearest_waypoint[1]
        goal_msg.pose.pose.orientation.w = 1.0

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)


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
        if result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("🛑 Waypoint hedefi iptal edildi.")
        elif result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("✅ Waypoint'e ulaşıldı.")
            
            #Sag-sol girilmez
            if self.mode == 'forward':
                self.get_logger().info("⏳ 10 metre ileri gidildi, orijinal hedefe dönülüyor...")

                # Restore the original goal if it exists
                if self.original_goal:
                    self.get_logger().info("↩️ Orijinal hedefe geri dönülüyor...")
                    send_goal_future = self.nav_client.send_goal_async(self.original_goal)
                    send_goal_future.add_done_callback(self.goal_response_callback)
                    self.original_goal = None
                else:
                    self.get_logger().info("ℹ️ live_gps'e devam ediliyor...")
                    self.command_pub.publish(String(data='green'))
                
                self.mode = 'normal'
            else:
                self.get_logger().info("⏳ Önceki hedefe geri dönülüyor...")
                self.command_pub.publish(String(data='green'))
                self.mode = 'normal'
                
    def check_waypoint_distance(self):
        if (self.mode not in ['waypoint']) or not self.current_pose or not hasattr(self, 'nearest_waypoint'):
            return

        x = self.current_pose.position.x
        y = self.current_pose.position.y
        
        # Waypoint modunda nearest_waypoint'i kullan
        if self.mode == 'waypoint' and hasattr(self, 'nearest_waypoint'):
            target_x = self.nearest_waypoint[0]
            target_y = self.nearest_waypoint[1]
        else:
            return

        distance = math.hypot(target_x - x, target_y - y)

        if distance < self.distance_threshold:
            self.get_logger().info(f"🛑 {distance:.2f} metre kala iptal ediliyor...")

            if self.active_goal_handle and self.active_goal_handle.status == GoalStatus.STATUS_EXECUTING:
                future = self.active_goal_handle.cancel_goal_async()
                
                def on_cancel_done(_):
                    self.get_logger().info("✅ Hedef iptal edildi, live_gps devam ediyor.")
                    time.sleep(0.3)
                    self.command_pub.publish(String(data='green'))
                    self.mode = 'normal'

                future.add_done_callback(on_cancel_done)
    def load_waypoints(self, file_path):
        points = []
        with open(file_path, 'r') as f:
            for line in f:
                if line.strip() and not line.startswith("#"):x, y, lat, lon = map(float, line.strip().split())
                points.append({'x': x, 'y': y, 'lat': lat, 'lon': lon})
        return points

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ])
        self.current_yaw = yaw

    def imu_callback(self, msg):
        orientation_q = msg.orientation
        _, _, yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        self.current_yaw = yaw

    def sign_callback(self, msg):
        try:
            data = json.loads(msg.data)

            if self.current_pose and self.mode == 'normal':    
                if 'sagyon' in data:
                    self.get_logger().info("🛑 'sagyon' levhası algılandı.")
                    self.command_pub.publish(String(data='red'))
                    time.sleep(0.2)
                    self.mode = 'waypoint'
                    self.send_nearest_right_waypoint()
                elif 'solyon' in data:
                    self.get_logger().info("🛑 'solyon' levhası algılandı.")
                    self.command_pub.publish(String(data='red'))
                    time.sleep(0.2)
                    self.mode = 'waypoint'
                    self.send_nearest_left_waypoint()
                elif 'girme' in data or 'kazi' in data or 'notraffic' in data:
                    self.get_logger().info("🛑 Girilmez türü levha algılandı.")
                    self.command_pub.publish(String(data='red'))
                    time.sleep(0.2)
                    self.mode = 'waypoint'
                    self.send_nearest_noentry_waypoint()
                elif 'sagdonulmez' in data or 'soladonulmez' in data or 'ilerivesag' in data or 'ilerivesol' in data:
                    self.get_logger().info("➡️ Düz git levhası algılandı, 10 metre ilerleniyor.")
                    self.command_pub.publish(String(data='red'))
                    time.sleep(0.2)
                    self.mode = 'forward'
                    if self.active_goal_handle:
                        self.original_goal = self.active_goal_handle.goal
                        cancel_future = self.active_goal_handle.cancel_goal_async()
                        cancel_future.add_done_callback(lambda _: self.go_forward_and_return())
                    else:
                        self.go_forward_and_return()
  
        except Exception as e:
            self.get_logger().error(f"❌ JSON parse hatası: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
