#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int8, Bool
import time
import matplotlib.pyplot as plt

class VelocityPlotterNode(Node):
    def __init__(self):
        super().__init__('velocity_plotter_node')

        self.motor_pub = self.create_publisher(Int8, '/stm/motor_power', 10)
        self.brake_pub = self.create_publisher(Bool, '/stm/brake', 10)
        self.odom_sub = self.create_subscription(Float32, '/stm/read_odometer', self.odom_callback, 10)

        self.prev_odom = None
        self.prev_time = None
        self.velocity_data = []
        self.time_data = []

        self.state = 'waiting_to_start'
        self.start_time = None

        self.timer = self.create_timer(0.5, self.state_machine)

    def state_machine(self):
        if self.state == 'waiting_to_start':
            self.get_logger().info("Sending motor power: 13")
            self.motor_pub.publish(Int8(data=13))
            self.start_time = time.time()
            self.state = 'accelerating'

        elif self.state == 'braking':
            self.get_logger().info("Applying brakes")
            self.brake_pub.publish(Bool(data=True))

        elif self.state == 'done':
            self.get_logger().info("Stopping motor and shutting down")
            self.motor_pub.publish(Int8(data=0))
            self.plot_data()
            rclpy.shutdown()

    def odom_callback(self, msg):
        current_time = time.time()
        current_odom = msg.data

        if self.prev_odom is None:
            self.prev_odom = current_odom
            self.prev_time = current_time
            return

        delta_odom = current_odom - self.prev_odom
        delta_time = current_time - self.prev_time

        if delta_time <= 0:
            return

        distance_m = delta_odom / 100.0
        velocity_mps = distance_m / delta_time
        velocity_kmh = velocity_mps * 3.6

        self.time_data.append(current_time - self.start_time)
        self.velocity_data.append(velocity_kmh)

        self.prev_odom = current_odom
        self.prev_time = current_time

        if self.state == 'accelerating' and velocity_kmh >= 10.0:
            self.get_logger().info(f"Reached 10 km/h at {current_time - self.start_time:.2f} s")
            self.state = 'braking'

        elif self.state == 'braking' and velocity_kmh <= 0.5:
            self.get_logger().info(f"Vehicle stopped at {current_time - self.start_time:.2f} s")
            self.state = 'done'

    def plot_data(self):
        accel_time = []
        accel_velocity = []
        brake_time = []
        brake_velocity = []

        reached_10 = False
        for t, v in zip(self.time_data, self.velocity_data):
            if not reached_10:
                accel_time.append(t)
                accel_velocity.append(v)
                if v >= 10.0:
                    reached_10 = True
            else:
                brake_time.append(t)
                brake_velocity.append(v)

        # Acceleration Plot
        plt.figure()
        plt.plot(accel_time, accel_velocity, label='Acceleration')
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (km/h)')
        plt.title('Acceleration Curve')
        plt.legend()
        plt.grid()
        plt.savefig('/Home/Desktop/acceleration_graph.png')
        plt.show()

        # Braking Plot
        plt.figure()
        plt.plot(brake_time, brake_velocity, label='Braking', color='red')
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (km/h)')
        plt.title('Braking Curve')
        plt.legend()
        plt.grid()
        plt.savefig('/Home/Desktop/braking_graph.png')
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPlotterNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()