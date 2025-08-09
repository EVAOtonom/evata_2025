import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import time
import math
import json
import os


class FullMissionNode(Node):
    def __init__(self):
        super().__init__('full_mission_node')

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.motor_power_pub = self.create_publisher(Int8, '/stm/motor_power', 10)


        # Ana hedefler
        self.main_goals = [
            {
                'x': 41.46486282348633,
                'y': -0.29233407974243164,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': 0.6439949568774449,
                'ow': 0.7650297350537546
            },
            {
                'x': 59.13648986816406,
                'y': 47.55416488647461,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': -0.09406762750698933,
                'ow': 0.9955658097058206
            }
        ]
        self.current_main_index = 0


        self._in_diversion = False
        self.current_pose = None
        self.current_yaw = None
        self.processed_signs = set()

        self._active_goal_handle = None
        self._current_main_goal_msg = None
        self.motion_enabled = True 
        self.mode = 'normal'
        self.original_goal = None

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.get_logger().info(" Görev başlatılıyor...")
        self.send_next_main_goal()

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.current_pose = pose.position
        q = pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw
        

    def send_next_main_goal(self):
        if self.current_main_index >= len(self.main_goals):
            self.get_logger().info(" Tüm ana hedeflere ulaşıldı.")
            return

        self.get_logger().info(f" Ana hedef {self.current_main_index + 1} gönderiliyor...")
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
            self.get_logger().error(" Ana hedef reddedildi.")
            return

        self.get_logger().info("✅ Ana hedef kabul edildi.")
        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.main_goal_result_callback)

    def main_goal_result_callback(self, future):
        if self._in_diversion:
            self.get_logger().warn(" Sapma sırasında sonuç geldi, dikkate alınmıyor.")
            return
        self.get_logger().info(f" Ana hedef {self.current_main_index + 1} tamamlandı.")
        self.current_main_index += 1
        self.send_next_main_goal()


    def return_to_main_goal(self, future):
        self.get_logger().info("🔁 Sapma tamamlandı. Ana hedefe dönülüyor...")
        self._in_diversion = False
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
