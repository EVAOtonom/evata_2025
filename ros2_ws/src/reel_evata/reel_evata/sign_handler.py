import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
import math
import json


class FullMissionNode(Node):
    def __init__(self):
        super().__init__('full_mission_node')

        # Action Client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._active_goal_handle = None
        self._current_main_goal_msg = None

        # Ana hedefler
        self.main_goals = [
            {
                'x': 16.225223541259766,
                'y': 2.757939338684082,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': 0.7398665063768184,
                'ow': 0.6727537088279495
            },
            {
                'x': 1.49641990661621,
                'y': 14.913789749145508,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': 0.9471947967777883,
                'ow': 0.9471947967777883
            },
            {
                'x': 2.0612287521362305,
                'y': 5.090720176696777,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': -0.6545659666592883,
                'ow': 0.7560048910499134
            }
        ]
        self.current_main_index = 0

        # Sapma noktaları
        self.sag_waypoint = {
            "x": 14.504013061523438,
            "y": 3.9969482421875,
            "z": 0.0,
            "ox": 0.0,
            "oy": 0.0,
            "oz": 0.016691209637834846,
            "ow": 0.9998606920570614
        }

        self.sol_waypoint = {
            "x": 14.649698257446289,
            "y": -3.3240017890930176,
            "z": 0.0,
            "ox": 0.0,
            "oy": 0.0,
            "oz": -0.01931759630839744,
            "ow": 0.9998133978262472
        }

        self._in_diversion = False

        self.create_subscription(String, '/detected_signs', self.sign_callback, 10)
        self.get_logger().info("🚀 Görev başlatılıyor...")
        self.send_next_main_goal()

    def send_next_main_goal(self):
        if self.current_main_index >= len(self.main_goals):
            self.get_logger().info("🎯 Tüm ana hedeflere ulaşıldı.")
            return

        self.get_logger().info(f"📍 Ana hedef {self.current_main_index + 1} gönderiliyor...")
        goal_dict = self.main_goals[self.current_main_index]
        goal_msg = self.create_pose_msg(goal_dict)
        self._current_main_goal_msg = goal_msg

        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("❌ Action server hazır değil.")
            return

        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.main_goal_response_callback)

    def main_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("🚫 Ana hedef reddedildi.")
            return

        self.get_logger().info("✅ Ana hedef kabul edildi.")
        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.main_goal_result_callback)

    def main_goal_result_callback(self, future):
        if self._in_diversion:
            self.get_logger().warn("⚠️ Sapma sırasında sonuç geldi, dikkate alınmıyor.")
            return
        self.get_logger().info(f"✅ Ana hedef {self.current_main_index + 1} tamamlandı.")
        self.current_main_index += 1
        self.send_next_main_goal()

    def sign_callback(self, msg):
        if self._in_diversion:
            return  # aynı anda birden fazla sapmaya izin verme

        try:
            levha = json.loads(msg.data)
            if "sol" in levha:
                self.get_logger().info(f"🪧 SOL levhası tespit edildi.")
                self.divert_to(self.sol_waypoint)
            elif "sag" in levha:
                self.get_logger().info(f"🪧 SAĞ levhası tespit edildi.")
                self.divert_to(self.sag_waypoint)
        except json.JSONDecodeError:
            self.get_logger().error("❌ JSON parse hatası.")

    def divert_to(self, waypoint_dict):
        if self._active_goal_handle:
            self.get_logger().info("🛑 Aktif ana hedef iptal ediliyor...")
            self._active_goal_handle.cancel_goal_async()

        self._in_diversion = True
        waypoint_msg = self.create_pose_msg(waypoint_dict)
        self.get_logger().info("↪️ Sapma hedefi gönderiliyor...")
        future = self._action_client.send_goal_async(waypoint_msg)
        future.add_done_callback(self.diversion_goal_callback)

    def diversion_goal_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Sapma hedefi reddedildi.")
            self._in_diversion = False
            return

        self.get_logger().info("✅ Sapma hedefi kabul edildi.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.return_to_main_goal)

    def return_to_main_goal(self, future):
        self.get_logger().info("🔁 Sapma tamamlandı. Ana hedefe dönülüyor...")
        self._in_diversion = False
        # Ana hedef aynı indexte → tekrar gönder
        if self._current_main_goal_msg:
            future = self._action_client.send_goal_async(self._current_main_goal_msg)
            future.add_done_callback(self.main_goal_response_callback)

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


def main(args=None):
    rclpy.init(args=args)
    node = FullMissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
