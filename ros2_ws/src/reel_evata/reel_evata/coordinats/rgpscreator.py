import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64
import time

class XYLatLonMapper(Node):
    def __init__(self):
        super().__init__('xy_lat_lon_mapper')

        # Değişkenler
        self.current_x = None
        self.current_y = None
        self.current_lat = None
        self.current_lon = None

        # Abonelikler
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        self.create_subscription(Float64, '/stm/gps_latitude', self.lat_callback, 10)
        self.create_subscription(Float64, '/stm/gps_longitude', self.lon_callback, 10)

        # Kayıt dosyası
        self.output_file = f"/home/otonom/xy_latlon_map_{int(time.time())}.txt"
        self.get_logger().info(f"📄 Kayıt dosyası: {self.output_file}")

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        # Nav2 harita koordinatları
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.save_if_ready()

    def lat_callback(self, msg: Float64):
        self.current_lat = msg.data
        self.save_if_ready()

    def lon_callback(self, msg: Float64):
        self.current_lon = msg.data
        self.save_if_ready()

    def save_if_ready(self):
        # Hem map x,y hem gps lat,lon varsa kaydet
        if None not in (self.current_x, self.current_y, self.current_lat, self.current_lon):
            line = f"{self.current_x:.6f} {self.current_y:.6f} {self.current_lat:.8f} {self.current_lon:.8f}\n"
            with open(self.output_file, 'a') as f:
                f.write(line)
            self.get_logger().info(f"💾 Kayıt: {line.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = XYLatLonMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
