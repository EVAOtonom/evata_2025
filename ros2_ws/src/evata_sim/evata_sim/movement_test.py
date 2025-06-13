import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)  # Topic ve queue boyutunu ayarlayın
        self.timer = self.create_timer(1.0, self.publish_cmd_vel)  # 1 saniye aralıklarla mesaj gönder

    def publish_cmd_vel(self):
        msg = Twist()

        # İleri hareket için hız değerlerini ayarlayın
        msg.linear.x = 1.0  # İleri hareket
        msg.linear.y = 0.0  # Y ekseninde hareket yok
        msg.linear.z = 0.0  # Z ekseninde hareket yok

        # Dönme hareketi için hız değerlerini ayarlayın
        msg.angular.x = 0.0  # Dönme hareketi yok
        msg.angular.y = 0.0  # Dönme hareketi yok
        msg.angular.z = 0.5  # Saat yönünde dönme

        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg}")

        time.sleep(2)

        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg}")

        time.sleep(2)

        msg.linear.x = -1.0
        msg.linear.z = -0.5
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg}")

        time.sleep(2)

        msg.linear.x = 0.0
        msg.linear.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg}")

        self.get_logger().info("kod bitti 5 saniye bekle")
        time.sleep(5)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
