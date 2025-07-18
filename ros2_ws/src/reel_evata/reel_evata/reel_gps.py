import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
import json
import math
import os
from std_msgs.msg import Float64
from ament_index_python.packages import get_package_share_directory


class ReelGPS(Node):
    def __init__(self):
        super().__init__('reel_gps')

        dir_path = os.path.dirname(os.path.realpath(__file__))
        src_dir = dir_path.split('/install')[0]
        self.target_file = os.path.join(src_dir, 'src', 'reel_evata', 'reel_evata', 'gps_reel.json')
        self.gps_map_file = os.path.join(src_dir, 'src', 'reel_evata', 'reel_evata', 'rgps.txt')

        self.current_lat = None
        self.current_lon = None

        self.gps_targets = self.load_gps_targets(self.target_file)
        self.current_index = 0
        self.gps_map = self.load_gps_map(self.gps_map_file)

        self.create_subscription(Float64, '/stm/gps_latitudee', self.lat_callback, 10)
        self.create_subscription(Float64, '/stm/gps_longitudee', self.lon_callback, 10)

        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._client.wait_for_server()

        self.timer_counter = 0
        self.create_timer(1.0, self.timer_callback)
        self.init_pose_published = False

    def load_gps_map(self, path):
        points = []
        with open(path, 'r') as f:
            for line in f:
                if line.strip().startswith("#") or not line.strip():
                    continue
                parts = line.strip().replace(',', ' ').split()
                if len(parts) < 4:
                    continue
                x, y, lat, lon = map(float, parts[:4])
                points.append((x, y, lat, lon))
        return points

    def gps_to_xy(self, lat, lon):
        nearest = sorted(self.gps_map, key=lambda p: (p[2] - lat) ** 2 + (p[3] - lon) ** 2)[:3]
        x_sum = y_sum = total_weight = 0.0
        for x, y, plat, plon in nearest:
            dist = math.hypot(plat - lat, plon - lon) + 1e-6
            weight = 1.0 / dist
            x_sum += x * weight
            y_sum += y * weight
            total_weight += weight
        return x_sum / total_weight, y_sum / total_weight

    def load_gps_targets(self, path):
        with open(path, 'r') as f:
            data = json.load(f)
            return [(p['lat'], p['lon']) for p in data]

    def lat_callback(self, msg):
        self.current_lat = msg.data

    def lon_callback(self, msg):
        self.current_lon = msg.data

    def timer_callback(self):
        if self.current_lat is None or self.current_lon is None:
            return

        self.timer_counter += 1

        # İlk 5 saniye boyunca bekle (AMCL başlasın)
        if not self.init_pose_published and self.timer_counter >= 5:
            x, y = self.gps_to_xy(self.current_lat, self.current_lon)

            pose = PoseWithCovarianceStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.pose.position.x = x
            pose.pose.pose.position.y = y
            pose.pose.pose.position.z = 0.0

            pose.pose.pose.orientation.x = 0.0
            pose.pose.pose.orientation.y = 0.0
            pose.pose.pose.orientation.z = 0.0
            pose.pose.pose.orientation.w = 1.0

            pose.pose.covariance = [
                0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0685
            ]

            self.init_pose_pub.publish(pose)
            self.init_pose_published = True
            self.get_logger().info(f"[✓] Initial pose yayınlandı → x: {x:.2f}, y: {y:.2f}")

            return

        if self.init_pose_published and self.current_index < len(self.gps_targets):
            target_lat, target_lon = self.gps_targets[self.current_index]
            x, y = self.gps_to_xy(target_lat, target_lon)
            self.send_goal(x, y)
            self.current_index += 1

    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"[→] Hedef gönderiliyor: x={x:.2f}, y={y:.2f}")
        send_future = self._client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("⚠️ Hedef reddedildi!")
            return
        self.get_logger().info("✅ Hedef kabul edildi.")
        goal_handle.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("🏁 Hedefe ulaşıldı.")
        else:
            self.get_logger().warn(f"🚫 Hedefe ulaşılamadı! Status: {result.status}")

def main(args=None):
    rclpy.init(args=args)
    node = ReelGPS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
