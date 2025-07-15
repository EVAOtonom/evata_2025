import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time


class Nav2GoalSender(Node):
    def __init__(self):
        super().__init__('movement_test')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.main_goals = [
            {
                'x': -40.591511182169484,
                'y': 4.4671915741234836,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': -0.7376934110430263,
                'ow': 0.6751358613669581
            },
            {
                'x': -20.852899111789764,
                'y': -24.427935305191205,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': 0.028543736465892144,
                'ow': 0.9995925445443087
            },
            {
                'x': 48.48131892249314,
                'y': 24.890732530497115,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': -0.00028891549098560774,
                'ow': 0.9999999582639186
            }
        ]

        self.waypoints = [
            [  # 1. ana hedefin waypointleri
                {
                    'x': -35.23582335844387,
                    'y': 46.08858700497336,
                    'z': 0.0,
                    'ox': 0.0,
                    'oy': 0.0,
                    'oz': -0.9276086689865977,
                    'ow': 0.3735534195010303
                }
            ],
            [  # 2. ana hedefin waypointleri
                {
                    'x': -37.28898553886179,
                    'y': -17.698402551081752,
                    'z': 0.0,
                    'ox': 0.0,
                    'oy': 0.0,
                    'oz': -0.43465068165341025,
                    'ow': 0.9005991255482241
                }
            ],
            [  # 3. ana hedefin waypointleri
                {
                    'x': 2.1029610015246165,
                    'y': -12.632455938737536,
                    'z': 0.0,
                    'ox': 0.0,
                    'oy': 0.0,
                    'oz': 0.7080780674015152,
                    'ow': 0.7061341589704716
                },
                {
                    'x': 2.4456254892550673,
                    'y': 18.37363565642434,
                    'z': 0.0,
                    'ox': 0.0,
                    'oy': 0.0,
                    'oz': 0.6951102951696329,
                    'ow': 0.7189031072051266
                }
            ]
        ]

        self.current_goal_index = 0
        self.current_waypoint_index = 0
        self.in_waypoint_phase = True

        self.send_next_waypoint_or_goal()

    def send_next_waypoint_or_goal(self):
        if self.current_goal_index >= len(self.main_goals):
            self.get_logger().info("Tüm ana hedeflere ulaşıldı.")
            return

        if self.in_waypoint_phase:
            waypoints_for_goal = self.waypoints[self.current_goal_index]
            if self.current_waypoint_index < len(waypoints_for_goal):
                waypoint = waypoints_for_goal[self.current_waypoint_index]
                self.get_logger().info(f"Waypoint {self.current_waypoint_index+1} gönderiliyor...")
                self.send_goal(waypoint, is_waypoint=True)
                return
            else:
                self.in_waypoint_phase = False

        main_goal = self.main_goals[self.current_goal_index]
        self.get_logger().info(f"Ana Hedef {self.current_goal_index+1} gönderiliyor...")
        self.send_goal(main_goal, is_waypoint=False)

    def send_goal(self, goal, is_waypoint):
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
        self._send_goal_future.add_done_callback(
            lambda future, is_wp=is_waypoint: self.goal_response_callback(future, is_wp)
        )

    def feedback_callback(self, feedback_msg):
        pass

    def goal_response_callback(self, future, is_waypoint):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Hedef reddedildi :(')
            return

        self.get_logger().info('Hedef kabul edildi.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(
            lambda future, is_wp=is_waypoint: self.get_result_callback(future, is_wp)
        )

    def get_result_callback(self, future, is_waypoint):
        if is_waypoint:
            self.get_logger().info(f'Waypoint {self.current_waypoint_index+1} tamamlandı.')
            self.current_waypoint_index += 1
        else:
            self.get_logger().info(f'Ana Hedef {self.current_goal_index+1} tamamlandı. 20 saniye bekleniyor...')
            time.sleep(20)
            self.current_goal_index += 1
            self.current_waypoint_index = 0
            self.in_waypoint_phase = True

        self.send_next_waypoint_or_goal()


def main(args=None):
    rclpy.init(args=args)
    nav2_goal_sender = Nav2GoalSender()
    rclpy.spin(nav2_goal_sender)
    nav2_goal_sender.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
