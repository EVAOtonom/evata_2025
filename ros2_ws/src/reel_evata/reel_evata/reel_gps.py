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
from std_msgs.msg import Float32
from ament_index_python.packages import get_package_share_directory


class ReelGPS(Node):
    def __init__(self):
        super().__init__('reel_gps')

        dir_path = os.path.dirname(os.path.realpath(__file__))
        src_dir = dir_path.split('/install')[0]
        self.target_file = os.path.join(src_dir, 'src', 'reel_evata', 'reel_evata', 'gps_reel.json')
        self.gps_map_file = os.path.join(src_dir, 'src', 'reel_evata', 'reel_evata', 'rgps.txt')

        self.prev_lat = None
        self.prev_lon = None
        self.heading_ready = False
        self.current_lat = None
        self.current_lon = None
        self.current_pose = None
        self.distance_threshold = 2.0
        self.heading_update_interval = 10
        self.motion_enabled = True
        self.paused = False

        self.gps_targets = self.load_gps_targets(self.target_file)
        self.current_index = 0
        self.gps_map = self.load_gps_map(self.gps_map_file)

        self.create_subscription(Float32, '/stm/gps_latitude', self.lat_callback, 10)
        self.create_subscription(Float32, '/stm/gps_longitude', self.lon_callback, 10)

        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.timer_counter = 0
        self.create_timer(1.0, self.timer_callback)
        self.init_pose_published = False

    def load_gps_targets(self, path):
        with open(path, 'r') as f:
            data = json.load(f)
            return [(p['lat'], p['lon']) for p in data]

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

    def lat_callback(self, msg):
        if self.current_lat is not None:
            self.prev_lat = self.current_lat
        self.current_lat = msg.data

    def lon_callback(self, msg):
        if self.current_lon is not None:
            self.prev_lon = self.current_lon
        self.current_lon = msg.data

    def timer_callback(self):
        if self.current_lat is None or self.current_lon is None:
            return

        self.timer_counter += 1

        # İlk 5 saniye boyunca bekle (AMCL başlasın)
        if self.timer_counter >= 5 and self.timer_counter % self.heading_update_interval == 0:
            x, y = self.gps_to_xy(self.current_lat, self.current_lon)

            # Yön hesaplama
            yaw = 0.0
            if self.prev_lat is not None and self.prev_lon is not None:
                prev_x, prev_y = self.gps_to_xy(self.prev_lat, self.prev_lon)
                dx = x - prev_x
                dy = y - prev_y
                if math.hypot(dx, dy) > 0.05:
                    yaw = math.atan2(dy, dx)
                    self.heading_ready = True

            # Yaw → Quaternion
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)

            pose = PoseWithCovarianceStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.pose.position.x = x
            pose.pose.pose.position.y = y
            pose.pose.pose.position.z = 0.0

            pose.pose.pose.orientation.x = 0.0
            pose.pose.pose.orientation.y = 0.0
            pose.pose.pose.orientation.z = qz
            pose.pose.pose.orientation.w = qw

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

            return

        if not self.init_pose_published or self.current_index >= len(self.gps_targets):
            return
        if not self.motion_enabled or self.paused:
            return

        x, y = self.gps_to_xy(self.current_lat, self.current_lon)
        self.current_pose = PoseStamped().pose
        self.current_pose.position.x = x
        self.current_pose.position.y = y

def main(args=None):
    rclpy.init(args=args)
    node = ReelGPS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
