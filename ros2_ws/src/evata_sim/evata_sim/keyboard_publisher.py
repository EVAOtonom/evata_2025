import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(String, 'keyboard_cmd', 10)
        self.get_logger().info('red, green, sag, ileri sag, sol, ileri sol, girme, notraffic, kazi, stop, soldonusyok, sagdonusyok')
        
        threading.Thread(target=self.keyboard_loop, daemon=True).start()

    def keyboard_loop(self):
        while rclpy.ok():
            key = input().strip().lower()
            if key in ['red', 'green', 'sag', 'sol','ilerisol', 'ilerisag', 'girme', 'notraffic', 'kazi', 'dur' , 'soldonusyok', 'sagdonusyok']:
                msg = String()
                msg.data = key
                self.publisher_.publish(msg)
                self.get_logger().info(f"Published command: {key}")
            else:
                self.get_logger().warn("❗Geçersiz tuş. ")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

