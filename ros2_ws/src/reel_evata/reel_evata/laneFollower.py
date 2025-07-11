import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from rclpy.qos import qos_profile_sensor_data
import math

class FollowPathNode(Node):
    def __init__(self):
        super().__init__('follow_path_node')

        self.declare_parameter('path_topic', '/lane_midpoints_path')
        self.declare_parameter('odom_topic', '/odom')

        self.path_topic = self.get_parameter('path_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Path, self.path_topic, self.path_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos_profile_sensor_data)

        self.path = None
        self.current_index = 0

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.timer = self.create_timer(0.1, self.follow_path)

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # orientation to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def path_callback(self, msg):
        if len(msg.poses) == 0:
            self.get_logger().warn("Boş path alındı.")
            return
        self.path = msg
        self.current_index = 0
        self.get_logger().info(f"Yeni path alındı, {len(msg.poses)} pozisyon içeriyor.")

    def follow_path(self):
        if self.path is None or self.current_index >= len(self.path.poses):
            self.stop_robot()
            return

        target_pose = self.path.poses[self.current_index].pose.position

        dx = target_pose.x - self.robot_x
        dy = target_pose.y - self.robot_y
        distance = math.sqrt(dx**2 + dy**2)

        if distance < 0.3:
            self.current_index += 1
            self.get_logger().info(f"Waypoint {self.current_index} tamamlandı.")
            return

        angle_to_target = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_target - self.robot_yaw)

        cmd = Twist()
        cmd.linear.x = min(0.5, distance)  # max hız 0.5 m/s
        cmd.angular.z = angle_diff  # hedefe yönel

        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = FollowPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

