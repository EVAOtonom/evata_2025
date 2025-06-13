#!/usr/bin/env python3.9

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
import math


class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')

        self.steering_angle_pub = self.create_publisher(Int8, '/stm/steering_angle', 10)
        self.motor_power_pub = self.create_publisher(Int8, '/stm/motor_power', 10)

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)

        self.get_logger().info('CmdVel Subscriber Node başlatıldı...')

    def listener_callback(self, msg):
        linear_x = msg.linear.x         # m/s
        angular_z = msg.angular.z       # rad/s

        # Aracın maksimum direksiyon açısı sınırları (derece cinsinden)
        MAX_LEFT_DEG = 40
        MAX_RIGHT_DEG = -43

        # Radyan → Derece dönüşümü
        angle_deg = math.degrees(angular_z)

        # Sınırlandır (derece cinsinden)
        steering_deg = max(MAX_RIGHT_DEG, min(MAX_LEFT_DEG, angle_deg))

        # Mesaj olarak gönder (Int8 olduğundan değer -128 ila 127 arasında olmalı)
        steering_msg = Int8()
        steering_msg.data = int(steering_deg)

        # Linear hızdan motor gücü hesapla (örnek: 1 m/s = %100 güç → 5 birim)
        # Katsayıyı artır: 20 veya 30 gibi
        motor_value = int(linear_x * 30)

        # Eğer motor_value çok küçükse, minimum hareket zorla (örneğin ±2)
        if 0 < abs(motor_value) < 2:
            motor_value = 2 * int(motor_value / abs(motor_value))  # Yönünü koru

        # Int8 sınırlarını kontrol et
        motor_value = max(-2, min(2, motor_value))

        motor_msg = Int8()
        motor_msg.data = motor_value

        # Yayınla
        self.steering_angle_pub.publish(steering_msg)
        self.motor_power_pub.publish(motor_msg)

        self.get_logger().info(
            f'CMD_VEL: linear_x={linear_x:.2f} m/s, angular_z={angular_z:.2f} rad/s '
            f'| Wheel Angle={steering_deg:.1f}°, Motor Power={motor_value}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
