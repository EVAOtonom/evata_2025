import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometer_listener')

        self.declare_parameter('wheel_base_cm', 155.0)
        self.wheel_base = self.get_parameter('wheel_base_cm').value  # cm

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Float32, '/stm/read_odometer', self.odom_callback, 10)
        self.create_subscription(Int32, '/stm/read_wheel_angle', self.angle_callback, 10)

        self.last_odom = None
        self.last_time = self.get_clock().now()
        self.current_angle_deg = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def angle_callback(self, msg: Int32):
        self.current_angle_deg = msg.data
        self.get_logger().debug(f"Steering angle: {msg.data}")

    def odom_callback(self, msg: Float32):
        current_odom = msg.data  # toplam mesafe cm
        now = self.get_clock().now()
        delta_time = (now - self.last_time).nanoseconds / 1e9  # saniye

        if self.last_odom is None:
            self.last_odom = current_odom
            self.last_time = now
            return

        delta_s = current_odom - self.last_odom
        self.last_odom = current_odom

        if abs(delta_s) < 1e-3 or delta_time <= 0.0:
            return

        steering_angle_rad = -1*math.radians(self.current_angle_deg)

        if abs(steering_angle_rad) < 1e-3:
            dx = delta_s * math.cos(self.yaw)
            dy = delta_s * math.sin(self.yaw)
            delta_yaw = 0.0
        else:
            R = self.wheel_base / math.tan(steering_angle_rad)
            delta_yaw = delta_s / R
            dx = R * (math.sin(self.yaw + delta_yaw) - math.sin(self.yaw))
            dy = R * (-math.cos(self.yaw + delta_yaw) + math.cos(self.yaw))

        self.x += dx
        self.y += dy
        self.yaw += delta_yaw

        # Quaternion oluştur
        q = quaternion_from_euler(0.0, 0.0, self.yaw)

        # Odometry mesajı hazırla
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        odom_msg.pose.pose.position.x = self.x / 100.0
        odom_msg.pose.pose.position.y = self.y / 100.0
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        odom_msg.twist.twist.linear.x = (delta_s / 100.0) / delta_time
        odom_msg.twist.twist.angular.z = delta_yaw / delta_time

        # Covariance matrisini sıfırdan 36 elemanlı liste olarak koyabilirsin (örnek)
        odom_msg.pose.covariance = [0.0] * 36
        odom_msg.twist.covariance = [0.0] * 36

        # TF mesajı hazırla ve yayınla
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x / 100.0
        t.transform.translation.y = self.y / 100.0
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.tf_broadcaster.sendTransform(t)
        self.odom_pub.publish(odom_msg)

        self.last_time = now


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
