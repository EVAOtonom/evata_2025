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
        self.reverse_pub = self.create_publisher(Bool, '/stm/reverse_command', 10)


        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_sub = self.create_subscription(Float32, '/stm/read_odometer', self.odom_callback, 10)
        self.obstacle_sub = self.create_subscription(Int8, '/obstacle_detected', self.obstacle_callback, 10)

        self.current_velocity = 0.0  # m/s
        self.target_velocity = 0.0   # m/s
        self.last_odom = None
        self.last_odom_time = None

        self.obstacle_detected = False

        # Direksiyon hassasiyet çarpanı
        self.steering_gain = 1 # İstersen runtime parametre olarak ayarla

        # Motor power scale
        self.max_motor_power = 17  # Motor güç limiti
        self.max_velocity = 1.5    # Navigasyondaki max hız (vx_max)
        self.get_logger().info('CmdVel Node başlatıldı.')

    def obstacle_callback(self, msg: Int8):
        self.obstacle_detected = (msg.data == 1)
        if self.obstacle_detected:
            self.get_logger().warn('[ENGEL] Engel algılandı! Araç durdurulacak.')

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
        if self.obstacle_detected:
            # Engel varsa durdur
            self.target_velocity = 0.0
            motor_power = 0  # ✅ motor_power tanımlanmalı
            self.motor_power_pub.publish(Int8(data=motor_power))
            self.brake_pub.publish(Bool(data=True))
            self.get_logger().warn('[ENGEL] Engel algılandı! Araç durduruluyor.')
            return

        self.target_velocity = msg.linear.x * 2.5
        angular_z = msg.angular.z * 2
        is_reverse = self.target_velocity < 0
        #self.reverse_pub.publish(Bool(data=is_reverse))
        
        # === Steering Angle Hesaplama ===
        WHEELBASE = 1.75
        MAX_LEFT_DEG = 40
        MAX_RIGHT_DEG = -43

        if abs(self.target_velocity) > 0.01:
            steering_rad = math.atan((WHEELBASE * angular_z) / self.target_velocity)
        else:
            steering_rad = 0.0

        angle_deg = math.degrees(steering_rad) * self.steering_gain
        steering_deg = max(MAX_RIGHT_DEG, min(MAX_LEFT_DEG, angle_deg)) * -1
        steering_deg = int(steering_deg)

        self.steering_angle_pub.publish(Int8(data=steering_deg))

        # === Motor Power Hesaplama ===
        speed_error = abs(self.target_velocity - self.current_velocity)
        brake = False
        if is_reverse:
            motor_power = 3
            brake = False
        elif self.current_velocity >= self.target_velocity + 1.0:
            motor_power = 0
            brake = True
        else:
            # Kademeli motor gücü ayarı
            if speed_error > 1.0:
                motor_power = self.max_motor_power  # Yüksek güç
            elif speed_error > 0.7:
                motor_power = int(self.max_motor_power * 0.8)  # Orta güç
            elif speed_error > 0.5:
                motor_power = int(self.max_motor_power * 0.6)  # Orta güç
            elif speed_error > 0.3:
                motor_power = int(self.max_motor_power * 0.4)  # Az güç
            elif speed_error > 0.2:
                motor_power = int(self.max_motor_power * 0.2)  # Daha az güç
            else:
                motor_power = 0  # Hemen hemen eşit, güç verme

            # Hedef hız 0 ise motoru kapat ve frenle
            if self.target_velocity == 0.0:
                motor_power = 0
                brake = True

        # motor_power sınırla
        motor_power = max(0, min(self.max_motor_power, int(motor_power)))
        self.motor_power_pub.publish(Int8(data=int(motor_power)))
        self.brake_pub.publish(Bool(data=brake))

        self.get_logger().info(
            f'[CMD_VEL] Hedef Hız: {self.target_velocity:.2f} m/s | '
            f'Anlık Hız: {self.current_velocity:.2f} m/s | '
            f'Motor Gücü: {motor_power} | Fren: {brake} | Direksiyon Açısı: {steering_deg}'
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
