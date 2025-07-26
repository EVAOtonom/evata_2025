import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
import time

class RealGoalSender(Node):
    def __init__(self):
        super().__init__('real_goal_sender')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.main_goals = [
            {
                'x': -19.592893600463867,
                'y': 1.903649091720581,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': -0.7841253799926245,
                'ow': 0.620602439933507
            },
            {
                'x': -12.48508358001709,
                'y': 7.337078094482422,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': 0.998342939900051,
                'ow': 0.05754454232786285
            },
            {
                'x': -12.427327156066895,
                'y': -1.9354193210601807,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': -0.9982578502109944,
                'ow': 0.05900224141610121
            }
        ]

        self.current_goal_index = 0
        self.send_next_goal()

    def send_next_goal(self):
        if self.current_goal_index >= len(self.main_goals):
            self.get_logger().info("Tüm hedeflere ulaşıldı.")
            return

        goal = self.main_goals[self.current_goal_index]
        self.get_logger().info(f"Ana hedef {self.current_goal_index + 1} gönderiliyor...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal['x']
        goal_msg.pose.pose.position.y = goal['y']
        goal_msg.pose.pose.position.z = goal['z']
        goal_msg.pose.pose.orientation.x = goal['ox']
        goal_msg.pose.pose.orientation.y = goal['oy']
        goal_msg.pose.pose.orientation.z = goal['oz']
        goal_msg.pose.pose.orientation.w = goal['ow']

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        pass  # isteğe bağlı

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Hedef reddedildi!')
            return

        self.get_logger().info('Hedef kabul edildi.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info(f"Ana hedef {self.current_goal_index + 1} tamamlandı. 10 saniye bekleniyor...")
        time.sleep(10)
        self.current_goal_index += 1
        self.send_next_goal()

def main(args=None):
    rclpy.init(args=args)
    node = RealGoalSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
