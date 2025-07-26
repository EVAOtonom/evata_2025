import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
import json  # <-- JSON verisi için eklendi


class SignHandler(Node):
    def __init__(self):
        super().__init__('sign_handler')

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._imu_yaw = None
        self._original_goal = None

        self.sol_waypoint = {
            'x': -12.48508358001709,
            'y': 7.337078094482422,
            'z': 0.0,
            'ox': 0.0,
            'oy': 0.0,
            'oz': 0.998342939900051,
            'ow': 0.05754454232786285
        }

        self.sag_waypoint = {
            'x': -12.427327156066895,
            'y': -1.9354193210601807,
            'z': 0.0,
            'ox': 0.0,
            'oy': 0.0,
            'oz': -0.9982578502109944,
            'ow': 0.05900224141610121
        }

        self.create_subscription(String, '/detected_signs', self.sign_callback, 10)
        self.create_subscription(Imu, '/zed2i/zed_node/imu/data', self.imu_callback, 10)

    def imu_callback(self, msg):
        orientation_q = msg.orientation
        (_, _, yaw) = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        self._imu_yaw = yaw

    def sign_callback(self, msg):
        try:
            levha_dict = json.loads(msg.data)

            if "sol" in levha_dict:
                confidence = levha_dict["sol"]
                self.get_logger().info(f"🪧 SOL levhası tespit edildi (Güven: {confidence})")
                self.handle_diversion(self.sol_waypoint)
            elif "sag" in levha_dict:
                confidence = levha_dict["sag"]
                self.get_logger().info(f"🪧 SAĞ levhası tespit edildi (Güven: {confidence})")
                self.handle_diversion(self.sag_waypoint)
            else:
                self.get_logger().info(f"⚠️ Bilinmeyen levha anahtarı: {levha_dict}")
        except json.JSONDecodeError:
            self.get_logger().error(f"❌ Geçersiz JSON formatı: {msg.data}")

    def handle_diversion(self, waypoint):
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("❌ Action server hazır değil.")
            return

        if self._original_goal:
            self.get_logger().info("🛑 Mevcut hedef iptal ediliyor...")
            self._original_goal.cancel_goal_async()

        self.get_logger().info("➡️ Waypoint'e yöneliniyor...")
        goal_msg = self.create_pose_msg(waypoint)
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.after_waypoint_callback)

    def after_waypoint_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Waypoint hedefi reddedildi.")
            return

        self.get_logger().info("✅ Waypoint hedefi kabul edildi.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.return_to_original_goal)

    def return_to_original_goal(self, future):
        self.get_logger().info("🔁 Waypoint tamamlandı. Ana hedefe geri dönülüyor...")
        if self._original_goal:
            goal_msg = self._original_goal.request.goal
            self._action_client.send_goal_async(goal_msg)

    def create_pose_msg(self, pose_dict):
        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = 'map'
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = pose_dict['x']
        msg.pose.pose.position.y = pose_dict['y']
        msg.pose.pose.position.z = pose_dict['z']
        msg.pose.pose.orientation.x = pose_dict['ox']
        msg.pose.pose.orientation.y = pose_dict['oy']
        msg.pose.pose.orientation.z = pose_dict['oz']
        msg.pose.pose.orientation.w = pose_dict['ow']
        return msg

    def set_original_goal(self, goal_handle):
        self._original_goal = goal_handle


def main(args=None):
    rclpy.init(args=args)
    node = SignHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

