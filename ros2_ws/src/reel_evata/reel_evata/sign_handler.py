import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import math
import json


class FullMissionNode(Node):
    def __init__(self):
        super().__init__('full_mission_node')

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._active_goal_handle = None
        self._current_main_goal_msg = None
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.motion_enabled = True 


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

        # Sağ ve sol sapma waypoint'leri
        self.diversion_waypoints = [{
            "x": 14.504013061523438,
            "y": 3.9969482421875,
            "z": 0.0,
            "ox": 0.0,
            "oy": 0.0,
            "oz": 0.016691209637834846,
            "ow": 0.9998606920570614
        },
        {   "x": 10.576292037963867,
            "y": -7.927124977111816,
            "z": 0.0,
            "ox": 0.0,
            "oy": 0.0,
            "oz": -0.7088734908016752,
            "ow": 0.7053356463689096
        },
        {   "x": 15.27441978454589,
            "y": -7.890341758728027,
            "z": 0.0,
            "ox": 0.0,
            "oy": 0.0,
            "oz": -0.6963168225090538,
            "ow": 0.717734548904325
        },    #SOL
        {
            "x": 14.649698257446289,
            "y": -3.3240017890930176,
            "z": 0.0,
            "ox": 0.0,
            "oy": 0.0,
            "oz": -0.01931759630839744,
            "ow": 0.9998133978262472
        },
        {   "x": 11.91103744506836,
            "y": 4.908627033233643,
            "z": 0.0,
            "ox": 0.0,
            "oy": 0.0,
            "oz": 0.712387997428572,
            "ow": 0.7017858228261019
        },
        {   "x": 16.0960636138916,
            "y": 4.619955539703369,
            "z": 0.0,
            "ox": 0.0,
            "oy": 0.0,
            "oz": 0.7176323193830824,
            "ow": 0.6964221809914283
        },]

        self._in_diversion = False
        self.current_pose = None
        self.current_yaw = None
        self.processed_signs = set()

        self.create_subscription(String, '/detected_signs', self.sign_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.get_logger().info(" Görev başlatılıyor...")
        self.send_next_main_goal()

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.current_pose = pose.position
        q = pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw

    def send_nearest_right_waypoint(self):
        self.send_nearest_side_waypoint(target="right")

    def send_nearest_left_waypoint(self):
        self.send_nearest_side_waypoint(target="left")
   
    def decide_no_entry_diversion(self):
        self.send_nearest_side_waypoint("noentry")


    def send_nearest_side_waypoint(self, target):
        if not self.current_pose or self.current_yaw is None:
            self.get_logger().warn("Pozisyon veya yön bilgisi eksik.")
            return

        x, y = self.current_pose.x, self.current_pose.y
        current_yaw = self.current_yaw
        candidates = []

        for wp in self.diversion_waypoints:
            wp_x, wp_y = wp['x'], wp['y']
            dx, dy = wp_x - x, wp_y - y
            distance = math.hypot(dx, dy)
            if distance < 0.5:
                continue

            wz, ww = wp['oz'], wp['ow']
            wp_yaw = math.atan2(2.0 * (ww * wz), 1.0 - 2.0 * (wz * wz))
            yaw_diff = math.degrees(wp_yaw - current_yaw)
            yaw_diff = (yaw_diff + 180) % 360 - 180  # -180..180 aralığına getir

            if target == "right" and -110 < yaw_diff < -70:
                candidates.append((distance, wp))
            elif target == "left" and 70 < yaw_diff < 110:
                candidates.append((distance, wp))
            elif target == "noentry" and (70 < yaw_diff < 110 or -110 < yaw_diff < -70):
                candidates.append((distance, wp))
        

        if not candidates:
            self.get_logger().warn(f" {target.upper()} yönünde uygun waypoint yok.")
            return

        nearest_wp = min(candidates, key=lambda t: t[0])[1]
        self.get_logger().info(f"➡️ {target.upper()} yönüne sapılıyor: {nearest_wp['x']:.2f}, {nearest_wp['y']:.2f}")
        self.divert_to(nearest_wp)


    def sign_callback(self, msg):
        if self._in_diversion:
            return
        try:
            detected = json.loads(msg.data)

            if "sag" in detected and "sag" not in self.processed_signs:
                self.get_logger().info(" Sağ levhası algılandı.")
                self.processed_signs.add("sag")
                self.send_nearest_right_waypoint()

            elif "sol" in detected and "sol" not in self.processed_signs:
                self.get_logger().info(" Sol levhası algılandı.")
                self.processed_signs.add("sol")
                self.send_nearest_left_waypoint()

            if "kirmizi" in detected and "kirmizi" not in self.processed_signs:
                self.get_logger().info(" Kırmızı ışık algılandı.")
                self.processed_signs.add("kirmizi")
                self.motion_enabled = False
                stop_msg = Twist()
                self.cmd_vel_pub.publish(stop_msg)
                return
            
            elif "girisiyok" in detected and "girisiyok" not in self.processed_signs:
                self.get_logger().info(" Girilmez levhası algılandı.")
                self.processed_signs.add("girisiyok")
                self.decide_no_entry_diversion()


        except Exception as e:
            self.get_logger().error(f"Levha verisi işlenemedi: {e}")


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

    def divert_to(self, waypoint_dict):
        if self._active_goal_handle:
            self.get_logger().info(" Aktif ana hedef iptal ediliyor...")
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
