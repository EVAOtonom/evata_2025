#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class BrakeControlNode(Node):
    def __init__(self):
        super().__init__('brake_control_node')
        self.brake_pub = self.create_publisher(Bool, '/stm/brake', 10)

        self.timer_period = 3.0  # Her 3 saniyede bir çağrılır
        self.timer = self.create_timer(self.timer_period, self.brake_cycle)

        self.brake_pressed = False  # Başlangıçta fren bırakılmış

    def brake_cycle(self):
        if self.brake_pressed:
            self.get_logger().info("Fren bırakılıyor (0)")
            self.brake_pub.publish(Bool(data=False))
        else:
            self.get_logger().info("Frene basılıyor (1)")
            self.brake_pub.publish(Bool(data=True))

        self.brake_pressed = not self.brake_pressed  # Durumu tersine çevir

def main(args=None):
    rclpy.init(args=args)
    node = BrakeControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
