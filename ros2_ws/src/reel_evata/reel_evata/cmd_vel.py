#!/usr/bin/env python3.9

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Bool, Float32
import math
from time import time


class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')

        self.steering_angle_pub = self.create_publisher(Int8, '/stm/steering_angle', 10)
        self.motor_power_pub = self.create_publisher(Int8, '/stm/motor_power', 10)
        self.brake_pub = self.create_publisher(Bool, '/stm/brake', 10)

        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_sub = self.create_subscription(Float32, '/stm/read_odometer', self.odom_callback, 10)

        self.current_velocity = 0.0  # m/s
        self.target_velocity = 0.0   # m/s
        self.last_odom = None        # cm
        self.last_odom_time = None   # s

        # PID parametreleri
        self.kp = 2.0 #kp yüksek olursa hızlı ama dengesiz tepkiler verir.
        self.ki = 0.7 #ki artarsa hatalar zamanla toparlanır ama aşım olabilir.
        self.kd = 0.8 #kd artarsa tepki yumuşar ama geç kalabilir.
        self.integral = 0.0
        self.last_error = 0.0

        self.get_logger().info('CmdVel Node başlatıldı.')

    def odom_callback(self, msg: Float32):
        current_odom = msg.data  # cm cinsinden
        current_time = time()

        if self.last_odom is None or self.last_odom_time is None:
            self.last_odom = current_odom
            self.last_odom_time = current_time
            return

        delta_s_cm = current_odom - self.last_odom
        delta_t = current_time - self.last_odom_time

        if delta_t <= 0:
            return

        distance_m = delta_s_cm / 100.0
        velocity_mps = distance_m / delta_t

        self.current_velocity = velocity_mps
        self.last_odom = current_odom
        self.last_odom_time = current_time

    def cmd_vel_callback(self, msg: Twist):
        # Hedef hızı 10 ile çarp
        self.target_velocity = msg.linear.x * 10
        angular_z = msg.angular.z

        # Direksiyon açısı hesapla
        WHEELBASE = 1.55
        MAX_LEFT_DEG = 40
        MAX_RIGHT_DEG = -43

        if abs(self.target_velocity) > 0.01:
            steering_rad = math.atan((WHEELBASE * angular_z) / self.target_velocity)
        else:
            steering_rad = 0.0

        angle_deg = math.degrees(steering_rad)
        steering_deg = max(MAX_RIGHT_DEG, min(MAX_LEFT_DEG, angle_deg)) * -1
        self.steering_angle_pub.publish(Int8(data=int(steering_deg)))

        # PID kontrol
        error = self.target_velocity - self.current_velocity
        self.integral += error
        derivative = error - self.last_error
        self.last_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Motor gücünü sınırlıyoruz
        motor_power = int(min(5, max(0, round(output))))

        # Fren durumu
        brake = False
        if self.target_velocity == 0.0:
            motor_power = 0
            brake = True

        # Yayınla
        self.motor_power_pub.publish(Int8(data=motor_power))
        self.brake_pub.publish(Bool(data=brake))

        self.get_logger().info(
            f'[CMD_VEL] Hedef: {self.target_velocity:.2f} m/s | '
            f'Anlık: {self.current_velocity:.2f} m/s | '
            f'Motor Power: {motor_power}, Fren: {brake}'
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
