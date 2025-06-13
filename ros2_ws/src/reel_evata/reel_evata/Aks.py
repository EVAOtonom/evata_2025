#!/usr/bin/env python3.9

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int8, Int32
import minimalmodbus
from enum import Enum
import time

class Register(Enum):
    STEERING_ANGLE = 0
    BRAKE = 1
    MOTOR_POWER = 2
    READ_WHEEL_ANGLE = 3
    READ_BRAKE_PRESSED = 4
    READ_BRAKE_RELEASED = 5
    READ_ODOMETER = 6
    REVERSE_COMMAND = 7
    LEFT_TURN_SIGNAL = 8
    RIGHT_TURN_SIGNAL = 9
    EMERGENCY_STOP = 10
    HEADLIGHTS_ON = 11
    MANUAL_DRIVE_MODE = 12
    RESET_ENCODER = 13
    GPS_LATITUDE = 14
    GPS_LATITUDE_2 = 15
    GPS_LONGITUDE = 16
    GPS_LONGITUDE_2 = 17
    GPS_SPEED = 18
    GPS_ALTITUDE = 19
    GPS_IS_LAVID = 20
    DRIVING_OTONOM = 21

class STMCommunication(Node):
    def __init__(self, port, slave_address=1, baudrate=38400):
        super().__init__('stm32_node')

        self.stm = minimalmodbus.Instrument(port, slave_address)
        self.stm.serial.baudrate = baudrate
        self.stm.serial.timeout = 1
        self.stm.clear_buffers_before_each_transaction = True
        self.stm.serial.bytesize = 8
        self.stm.serial.parity = 'N'
        self.stm.serial.stopbits = 1

        self.mutex = 0
        self.gps_latitude = None
        self.gps_latitude_1 = None
        self.gps_latitude_2 = None
        self.gps_longitude = None
        self.gps_longitude_1 = None
        self.gps_longitude_2 = None
        self.read_odometer = 0
        self.check_otonom = None
        self.check_otonom_stop = 0
        self.encoder_reset_done = False

        # Publishers
        self.gps_latitude_pub = self.create_publisher(Float32, '/stm/gps_latitude', 10)
        self.gps_longitude_pub = self.create_publisher(Float32, '/stm/gps_longitude', 10)
        self.read_odometer_pub = self.create_publisher(Float32, '/stm/read_odometer', 10)
        self.check_otonom_pub = self.create_publisher(Bool, '/stm/check_otonom', 10)
        self.brake_status_pub = self.create_publisher(Bool, '/stm/brake_status', 10)
        self.read_wheel_angle_pub = self.create_publisher(Int32, '/stm/read_wheel_angle', 10)


        # Subscribers
        self.create_subscription(Int8, '/stm/steering_angle', self.steering_angle_callback, 10)
        self.create_subscription(Int8, '/stm/motor_power', self.motor_power_callback, 10)
        self.create_subscription(Bool, '/stm/reset_odometer', self.reset_odometer_callback, 10)
        self.create_subscription(Bool, '/stm/brake', self.brake_callback, 10)
        self.create_subscription(Int8, '/stm/left_signal', self.l_signal_callback, 10)
        self.create_subscription(Int8, '/stm/right_signal', self.r_signal_callback, 10)

        self.create_timer(1.0 / 3.0, self.publish_data)  # 3 Hz

    def send_command(self, register, data):
        try:
            datatemp = data
            if -32769 < data < 32768:
                if data < 0:
                    data = 65536 + data
                if self.mutex == 0:
                    self.mutex = 1
                    self.stm.write_register(register.value, int(data))
                    self.mutex = 0
                    self.get_logger().info(f'{register.name} degeri {datatemp} olarak gonderildi.')
            else:
                self.get_logger().warn(f"32767 ila -32768 araliginda deger giriniz. {datatemp}")
        except Exception as e:
            self.get_logger().error(f"GONDERME HATASI {e}")
            self.mutex = 0

    def read_data(self, register):
        try:
            return self.stm.read_register(register.value)
        except Exception as e:
            self.get_logger().warn("OKUMA HATASI")
            return None

    def steering_angle_callback(self, msg):
        self.send_command(Register.STEERING_ANGLE, msg.data)

    def brake_callback(self, msg):
        self.send_command(Register.BRAKE, msg.data)

    def motor_power_callback(self, msg):
        self.send_command(Register.MOTOR_POWER, msg.data)

    def reset_odometer_callback(self, msg):
        self.send_command(Register.RESET_ENCODER, msg.data)

    def right_signal(self, x=5):
        for _ in range(x):
            self.send_command(Register.RIGHT_TURN_SIGNAL, 1)
            time.sleep(0.6)
            self.send_command(Register.RIGHT_TURN_SIGNAL, 0)
            time.sleep(0.6)
        self.get_logger().info("SAG SINYAL BITTI")

    def left_signal(self, x=5):
        self.get_logger().info("SOL SINYAL BITTI")

    def r_signal_callback(self, msg):
        self.right_signal(msg.data)

    def l_signal_callback(self, msg):
        self.left_signal(msg.data)

    def publish_data(self):
        if not self.encoder_reset_done:
            self.send_command(Register.RESET_ENCODER, 1)
            self.encoder_reset_done = True        
        if self.mutex == 0:
            self.mutex = 1
            self.gps_latitude_1 = self.read_data(Register.GPS_LATITUDE)
            self.gps_latitude_2 = self.read_data(Register.GPS_LATITUDE_2)
            self.gps_longitude_1 = self.read_data(Register.GPS_LONGITUDE)
            self.gps_longitude_2 = self.read_data(Register.GPS_LONGITUDE_2)
            self.read_odometer = self.read_data(Register.READ_ODOMETER)
            self.read_wheel_angle = self.read_data(Register.READ_WHEEL_ANGLE)
            
        if self.read_wheel_angle is not None:
            try:
                self.read_wheel_angle = int(self.read_wheel_angle)
                # Signed 16-bit dönüşüm:
                if self.read_wheel_angle > 32767:
                    self.read_wheel_angle -= 65536
                if -128 <= self.read_wheel_angle <= 127:
                    self.read_wheel_angle_pub.publish(Int32(data=self.read_wheel_angle))
                else:
                    self.get_logger().warn(f"Wheel angle out of Int32 range. {self.read_wheel_angle}")
            except ValueError:
                self.get_logger().warn("Geçersiz wheel angle değeri.")
            
            if self.gps_latitude_1 is not None and self.gps_latitude_2 is not None:
                self.gps_latitude = ((self.gps_latitude_1 * 10000) + self.gps_latitude_2) / 1000000
                self.gps_latitude_pub.publish(Float32(data=self.gps_latitude))
            
            if self.gps_longitude_1 is not None and self.gps_longitude_2 is not None:
                self.gps_longitude = ((self.gps_longitude_1 * 100000) + (self.gps_longitude_2 * 10)) / 100000000
                self.gps_longitude_pub.publish(Float32(data=self.gps_longitude))
            
            if self.read_odometer is not None:
                # read_odometer'ı float'a dönüştür ve sınırları kontrol et
                try:
                    self.read_odometer = float(self.read_odometer)
                    if self.read_odometer < 64000:
                        self.read_odometer_pub.publish(Float32(data=self.read_odometer))
                except ValueError:
                    self.get_logger().warn("Geçersiz odometre değeri.")
            
            if self.check_otonom is not None and self.check_otonom != 0:
                self.check_otonom_pub.publish(Bool(data=self.check_otonom))

            self.mutex = 0



def main(args=None):
    rclpy.init(args=args)
    port = '/dev/ttyUSB0'
    stm_node = STMCommunication(port)
    rclpy.spin(stm_node)

    stm_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
