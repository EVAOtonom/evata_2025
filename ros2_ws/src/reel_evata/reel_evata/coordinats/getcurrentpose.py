import os
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformListener

class XYLatLonLogger(Node):
    def __init__(self):
        super().__init__('xy_latlon_logger')

        # TF2 buffer ve listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # GPS verileri
        self.current_lat = None
        self.current_lon = None

        # TXT dosyası
        self.output_file = f"/home/otonom/xy_latlon_map_{int(time.time())}.txt"
        self.get_logger().info(f"📄 Kayıt dosyası: {self.output_file}")

        # GPS abonelikleri
        self.create_subscription(Float64, '/stm/gps_latitude', self.lat_callback, 10)
        self.create_subscription(Float64, '/stm/gps_longitude', self.lon_callback, 10)

        # Timer ile pozisyon al
        self.timer = self.create_timer(0.5, self.get_pose_and_log)

    def lat_callback(self, msg: Float64):
        self.current_lat = msg.data

    def lon_callback(self, msg: Float64):
        self.current_lon = msg.data

    def get_pose_and_log(self):
        try:
            # base_link → map dönüşümünü al
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )

            x = trans.transform.translation.x
            y = trans.transform.translation.y

            # GPS verileri hazırsa kaydet
            if None not in (self.current_lat, self.current_lon):
                line = f"{x:.6f} {y:.6f} {self.current_lat:.8f} {self.current_lon:.8f}\n"
                with open(self.output_file, 'a') as f:
                    f.write(line)
                self.get_logger().info(f"💾 Kayıt: {line.strip()}")

        except Exception as e:
            self.get_logger().warn(f"TF dönüşümü alınamadı: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = XYLatLonLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

