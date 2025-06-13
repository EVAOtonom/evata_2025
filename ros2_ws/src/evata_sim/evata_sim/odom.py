import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',  # Odom topiği adı
            self.listener_callback,
            10
        )
        self.subscription  # Abonelik değişkeni kullanılmazsa uyarı alabilirsiniz

    def listener_callback(self, msg):
        # Gelen odometry mesajını terminalde göster
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        self.get_logger().info(
            f'Position -> x: {position.x:.2f}, y: {position.y:.2f}, z: {position.z:.2f}'
        )
        self.get_logger().info(
            f'Orientation -> x: {orientation.x:.2f}, y: {orientation.y:.2f}, z: {orientation.z:.2f}, w: {orientation.w:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = OdomSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

