#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int8, Bool
import time

class StabilVelocityNode(Node):
    def __init__(self):
        super().__init__('stabil_velocity_node')
        
        # Global variables
        self.current_velocity = 0.0
        self.brake_status = False
        self.previous_odom = None
        self.previous_time = None
        self.last_brake_change_time = None
        
        self.get_logger().info("Waiting for 'lane_track_node' service...")
        self.get_logger().info("'lane_track_node' service is now available.")

        # Publishers
        self.motor_power_pub = self.create_publisher(Int8, '/stm/motor_power', 10)
        self.velocity_pub = self.create_publisher(Float32, '/vehicle/velocity_kmh', 10)
        self.brake_pub = self.create_publisher(Bool, '/stm/brake', 10)

        # Subscribers
        self.brake_sub = self.create_subscription(
            Bool,
            '/stm/brake',
            self.brake_callback,
            10)
        
        self.odometer_sub = self.create_subscription(
            Float32,
            '/stm/read_odometer',
            self.odometer_callback,
            10)
        
        # Timer for checking brake status
        self.timer = self.create_timer(1.0, self.check_brake_status)

    def brake_callback(self, msg):
        self.brake_status = msg.data
        self.last_brake_change_time = time.time()  # Update the time when brake status changes

    def odometer_callback(self, msg):
        current_odom = msg.data  # Incoming data in centimeters
        current_time = time.time()

        # If first call, update previous values and return
        if self.previous_odom is None or self.previous_time is None:
            self.previous_odom = current_odom
            self.previous_time = current_time
            return

        # Calculate velocity
        distance_traveled_cm = current_odom - self.previous_odom # cm
        time_elapsed = current_time - self.previous_time

        if time_elapsed > 0:
            # cm to meters
            distance_traveled_m = distance_traveled_cm / 100.0

            # m/s to km/h
            velocity_mps = distance_traveled_m / time_elapsed
            velocity_kmh = velocity_mps * 3.6

            # Publish velocity and update current_velocity
            self.velocity_pub.publish(Float32(data=velocity_kmh))
            self.current_velocity = velocity_kmh

        # Update previous values
        self.previous_odom = current_odom
        self.previous_time = current_time

        self.control_motor_power()

    def control_motor_power(self):
        if self.brake_status:
            self.motor_power_pub.publish(Int8(data=0))
        else:
            if self.current_velocity > 2.5:
                self.motor_power_pub.publish(Int8(data=0))
            elif self.current_velocity < 2.2:
                self.motor_power_pub.publish(Int8(data=4))

    def check_brake_status(self):
        if self.last_brake_change_time is not None and (time.time() - self.last_brake_change_time) > 35:
            self.brake_pub.publish(Bool(data=False))  
            self.last_brake_change_time = time.time()  

def main(args=None):
    rclpy.init(args=args)
    node = StabilVelocityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()