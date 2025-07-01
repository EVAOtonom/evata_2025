import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus

import math
import os
import time
import json

class GPSNavigator(Node):
    def __init__(self):
        super().__init__('gps_navigator')
        self.distance_threshold = 2.0  # 2 metre esneme payı
        self.create_timer(0.5, self.check_goal_distance)
        self.goal_cancelling = False

        self.saved_goal = None

        package_path = get_package_share_directory('evata_sim')
        self.gps_map_file = os.path.join(package_path, 'gps_data', 'gps_map.txt')
        self.gps_map = self.load_gps_map(self.gps_map_file)

        self.gps_target_file = os.path.join(package_path, 'json', 'gps_targets.json')
        self.gps_targets = self.load_gps_targets(self.gps_target_file)
        self.current_index = 0

        self.paused = False
        self.motion_enabled = True

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps_corrected', 10)
        self.heading_pub = self.create_publisher(Float32, '/gps/heading', 10)
        self.create_timer(0.5, self.publish_direction)
        self.create_subscription(String, 'nav_cmd', self.control_callback, 10)

        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._client.wait_for_server()

        self.prev_x = None
        self.prev_y = None
        self.prev_pose = None
        self.current_pose = None
        self.goal_handle = None

        self.send_next_goal()

    def check_goal_distance(self):
        if (self.paused or not self.motion_enabled or not self.current_pose or 
            self.goal_cancelling or self.current_index >= len(self.gps_targets)):
            return
        
        target_lat, target_lon = self.gps_targets[self.current_index]
        target_x, target_y = self.gps_to_xy(target_lat, target_lon)
        
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        distance = math.hypot(target_x - current_x, target_y - current_y)
        
        # Sadece 2 metre veya daha yakınsa iptal et
        if distance <= 2.0:  # 2 metre veya daha yakın
            self.goal_cancelling = True  # Kilitle
            self.get_logger().info(f'✅ Hedefe {distance:.2f}m kaldı - Tamamlandı sayılıyor.')
            
            if self.goal_handle:
                cancel_future = self.goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self._handle_distance_cancel)
            else:
                self._proceed_to_next()

    def load_gps_map(self, path):
        gps_points = []
        with open(path, 'r') as f:
            for line in f:
                if line.strip().startswith("#") or len(line.strip()) == 0:
                    continue
                x, y, lat, lon = map(float, line.strip().split())
                gps_points.append((x, y, lat, lon))
        return gps_points
        
    def _handle_distance_cancel(self, future):
        self.goal_cancelling = False
        self.goal_handle = None
        self._proceed_to_next()

    def _proceed_to_next(self):
        self.current_index += 1
        self.saved_goal = None
        if self.current_index < len(self.gps_targets):
            self.send_next_goal()

    def gps_to_xy(self, lat, lon):
        nearest_points = sorted(
            self.gps_map,
            key=lambda p: (p[2] - lat) ** 2 + (p[3] - lon) ** 2
        )[:3]

        total_weight = 0.0
        x_sum = 0.0
        y_sum = 0.0

        for x, y, plat, plon in nearest_points:
            dist = math.sqrt((plat - lat) ** 2 + (plon - lon) ** 2) + 1e-6
            weight = 1.0 / dist
            total_weight += weight
            x_sum += x * weight
            y_sum += y * weight

        return x_sum / total_weight, y_sum / total_weight

    def xy_to_gps(self, x, y):
        nearest = sorted(self.gps_map, key=lambda p: (p[0] - x) ** 2 + (p[1] - y) ** 2)[:3]
        total_weight = 0.0
        lat_sum = 0.0
        lon_sum = 0.0
        for px, py, plat, plon in nearest:
            dist = math.hypot(px - x, py - y) + 0.001
            weight = 1.0 / dist
            total_weight += weight
            lat_sum += plat * weight
            lon_sum += plon * weight
        return lat_sum / total_weight, lon_sum / total_weight

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        corrected_lat, corrected_lon = self.xy_to_gps(x, y)

        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "gps_corrected"
        gps_msg.latitude = corrected_lat
        gps_msg.longitude = corrected_lon
        gps_msg.altitude = 0.0
        self.gps_pub.publish(gps_msg)

        #self.get_logger().info(f"Corrected GPS: lat={corrected_lat:.7f}, lon={corrected_lon:.7f}")

        if self.prev_x is not None and self.prev_y is not None:
            dx = x - self.prev_x
            dy = y - self.prev_y
            if abs(dx) > 0.001 or abs(dy) > 0.001:
                heading_rad = math.atan2(dy, dx)
                self.last_heading = (math.degrees(heading_rad) + 360.0) % 360.0

        if hasattr(self, 'last_heading'):
            self.heading_pub.publish(Float32(data=self.last_heading))

        self.prev_x = x
        self.prev_y = y

        self.prev_pose = self.current_pose
        self.current_pose = msg.pose.pose

    def publish_direction(self):
        if self.prev_pose is None or self.current_pose is None:
            return

        dx = self.current_pose.position.x - self.prev_pose.position.x
        dy = self.current_pose.position.y - self.prev_pose.position.y

        if abs(dx) < 1e-4 and abs(dy) < 1e-4:
            return

        angle = math.atan2(dy, dx)
        angle_deg = (math.degrees(angle) + 360) % 360

        if 337.5 <= angle_deg or angle_deg < 22.5:
            direction = "➡️ Doğu"
        elif 22.5 <= angle_deg < 67.5:
            direction = "↗️ Kuzeydoğu"
        elif 67.5 <= angle_deg < 112.5:
            direction = "⬆️ Kuzey"
        elif 112.5 <= angle_deg < 157.5:
            direction = "↖️ Kuzeybatı"
        elif 157.5 <= angle_deg < 202.5:
            direction = "⬅️ Batı"
        elif 202.5 <= angle_deg < 247.5:
            direction = "↙️ Güneybatı"
        elif 247.5 <= angle_deg < 292.5:
            direction = "⬇️ Güney"
        elif 292.5 <= angle_deg < 337.5:
            direction = "↘️ Güneydoğu"
        else:
            direction = "❓"

        #self.get_logger().info(f"🧭 Anlık yön: {direction} ({angle_deg:.1f}°)")

    def send_next_goal(self):
        if self.paused:
            self.get_logger().info('⏸️ Navigasyon redaklatıldı.')
            return

        if not self.motion_enabled:
            self.get_logger().info("⏸️ Hareket devre dışı, hedef gönderilmeyecek.")
            return

        if self.current_index >= len(self.gps_targets):
            self.get_logger().info('🎉 Tüm GPS hedeflerine ulaşıldı.')
            return

        lat, lon = self.gps_targets[self.current_index]
        x, y = self.gps_to_xy(lat, lon)

        self.get_logger().info(f'🚀 Hedef {self.current_index + 1}/{len(self.gps_targets)}: GPS ({lat}, {lon}) → XY ({x:.2f}, {y:.2f})')
        self.send_goal(x, y)

    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"🎯 Hedef gönderiliyor: XY ({x:.2f}, {y:.2f})")
        send_goal_future = self._client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def control_callback(self, msg):
        if msg.data == 'red':
            self.motion_enabled = False
            self.get_logger().info("🛑 Hareket durduruldu, hedefe 2m kala iptal edilecek.")
            
            if self.current_index < len(self.gps_targets):
                self.saved_goal = {
                    'target': self.gps_targets[self.current_index],
                    'index': self.current_index
                }
                self.get_logger().info(f"💾 Hedef kaydedildi: {self.saved_goal['target']}")

        elif msg.data == 'green':
            self.motion_enabled = True
            self.get_logger().info("✅ Hareket yeniden başlatılıyor.")
            if self.saved_goal:
                self.get_logger().info(f"↩️ Kaydedilen hedefe dönülüyor: {self.saved_goal['target']}")
                lat, lon = self.saved_goal['target']
                self.current_index = self.saved_goal['index']
                x, y = self.gps_to_xy(lat, lon)
                self.send_goal(x, y)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('❌ Hedef reddedildi!')
            time.sleep(1.0)
            self.current_index += 1
            self.send_next_goal()
            return

        self.get_logger().info('✅ Hedef kabul edildi. Bekleniyor...')
        self.goal_handle = goal_handle

        if not self.saved_goal and self.current_index < len(self.gps_targets):
            self.saved_goal = self.gps_targets[self.current_index]

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("⏸️ Hedef iptal edildi, hafızadaki hedefe green ediliyor...")
            if self.saved_goal:
                lat, lon = self.saved_goal['target']
                x, y = self.gps_to_xy(lat, lon)
                self.send_goal(x, y)
        elif result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✅ Hedefe ulaşıldı.')
            self.saved_goal = None
            time.sleep(1.0)
            self.current_index += 1
            self.send_next_goal()
        else:
            self.get_logger().warn(f"⚠️ Hedef işlemi beklenmeyen bir redumla tamamlandı. Status: {result.status}")
            
    def load_gps_targets(self, path):
        with open(path, 'r') as f:
            data = json.load(f)
            return [(point['lat'], point['lon']) for point in data]


def main(args=None):
    rclpy.init(args=args)
    node = GPSNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

