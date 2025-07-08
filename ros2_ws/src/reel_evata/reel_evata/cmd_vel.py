#!/usr/bin/env python3.9

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8,Bool
import math


class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')

        self.steering_angle_pub = self.create_publisher(Int8, '/stm/steering_angle', 10)
        self.motor_power_pub = self.create_publisher(Int8, '/stm/motor_power', 10)
        self.brake_pub = self.create_publisher(Bool, '/stm/brake', 10)

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)

        self.get_logger().info('CmdVel Subscriber Node başlatıldı...')

    def listener_callback(self, msg):
        linear_x = msg.linear.x         # m/s
        angular_z = msg.angular.z       # rad/s

        # Aracın özellikleri
        WHEELBASE = 1.55  # metre
        MAX_LEFT_DEG = 40
        MAX_RIGHT_DEG = -43

        # Direksiyon açısı hesapla
        if abs(linear_x) > 0.01:
            steering_rad = math.atan((WHEELBASE * angular_z) / linear_x)
        else:
            steering_rad = 0.0

        angle_deg = math.degrees(steering_rad)
        steering_deg = max(MAX_RIGHT_DEG, min(MAX_LEFT_DEG, angle_deg)) * -1

        steering_msg = Int8()
        steering_msg.data = int(steering_deg)

        # Motor gücü ve fren
        if linear_x <= 0:
            motor_value = 0
            brake = True
        elif linear_x < 0.03:
            motor_value = 5
            brake = False
        elif linear_x < 0.06:
            motor_value = 8
            brake = False
        elif linear_x < 0.08:
            motor_value = 10
            brake = False

        motor_msg = Int8()
        motor_msg.data = motor_value

        brake_msg = Bool()
        brake_msg.data = brake

        # Yayınla
        self.steering_angle_pub.publish(steering_msg)
        self.motor_power_pub.publish(motor_msg)
        self.brake_pub.publish(brake_msg)

        self.get_logger().info(
            f'CMD_VEL: linear_x={linear_x:.2f} m/s, angular_z={angular_z:.2f} rad/s '
            f'| Wheel Angle={steering_deg:.1f}°, Motor Power={motor_value}, Brake={brake}'
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
