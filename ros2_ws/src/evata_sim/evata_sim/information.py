import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import json

class StatusMonitor(Node):
    def __init__(self):
        super().__init__('status_monitor')

        # Değerleri başlat
        self.latitude = None
        self.longitude = None
        self.heading = None
        self.velocity = None
        self.sign_name = None
        self.sign_distance = None

        # Abonelikler
        self.create_subscription(NavSatFix, '/gps_corrected', self.gps_callback, 10)
        self.create_subscription(Float32, '/gps/heading', self.heading_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(String, '/detected_signs', self.sign_callback, 10)

        self.create_timer(0.5, self.display_status)

    def gps_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude

    def heading_callback(self, msg):
        self.heading = msg.data

    def odom_callback(self, msg):
        self.velocity = msg.twist.twist.linear.x

    def sign_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if data:
                self.sign_name, self.sign_distance = next(iter(data.items()))
            else:
                self.sign_name = None
                self.sign_distance = None
        except json.JSONDecodeError:
            self.sign_name = None
            self.sign_distance = None

    def display_status(self):
        if self.latitude is None or self.longitude is None or self.heading is None or self.velocity is None:
            return

        output = f"long: {self.longitude:.6f}  lat: {self.latitude:.6f}  heading: {self.heading:.2f}°  velocity: {self.velocity:.2f} m/s"

        if self.sign_name:
            output += f"  sign: {self.sign_name}  sign distance: {self.sign_distance:.2f} m"

        print(output)

def main(args=None):
    rclpy.init(args=args)
    node = StatusMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
