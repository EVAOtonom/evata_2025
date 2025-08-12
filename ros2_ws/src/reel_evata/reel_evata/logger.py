import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from datetime import datetime
import os

class GPSAndSignLogger(Node):
    def __init__(self):
        super().__init__('gps_and_sign_logger')

        # Dosya yolu ve isimlendirme
        log_dir = os.path.join(os.path.expanduser('~'), 'ros2_logs')
        os.makedirs(log_dir, exist_ok=True)
        self.log_file_path = os.path.join(log_dir, 'gps_sign_log.txt')

        # GPS verilerini tutmak için geçici değişkenler
        self.latest_lat = None
        self.latest_lon = None
        self.latest_sign = None

        # Topic abonelikleri
        self.create_subscription(Float32, '/stm/gps_latitude', self.lat_callback, 10)
        self.create_subscription(Float32, '/stm/gps_longitude', self.lon_callback, 10)
        self.create_subscription(String, '/detected_signs', self.sign_callback, 10)

        self.get_logger().info(f"Log dosyası: {self.log_file_path}")

    def lat_callback(self, msg):
        self.latest_lat = msg.data
        self.write_log()

    def lon_callback(self, msg):
        self.latest_lon = msg.data
        self.write_log()

    def sign_callback(self, msg):
        self.latest_sign = msg.data
        self.write_log()

    def write_log(self):
        if self.latest_lat is not None and self.latest_lon is not None and self.latest_sign is not None:
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            log_line = f"{timestamp}, LAT: {self.latest_lat:.8f}, LON: {self.latest_lon:.8f}, SIGN: {self.latest_sign}\n"

            with open(self.log_file_path, 'a') as f:
                f.write(log_line)

            # İstersen terminale de yazsın
            self.get_logger().info(log_line.strip())

def main(args=None):
    rclpy.init(args=args)
    node = GPSAndSignLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
