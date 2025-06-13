import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        # ZED kameranın RGB ve derinlik görüntülerini almak için abone oluyoruz
        self.rgb_subscription = self.create_subscription(
            Image,
            '/depth_camera/zed/image',  # ZED kameranın RGB görüntüsü
            self.rgb_listener_callback,
            10)
        
        self.depth_subscription = self.create_subscription(
            Image,
            '/depth_camera/zed/depth_image',  # ZED kameranın derinlik görüntüsü
            self.depth_listener_callback,
            10)
        
        self.bridge = CvBridge()

    def rgb_listener_callback(self, msg):
        try:
            # RGB görüntüsünü OpenCV formatına dönüştür
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Görüntüyü ekranda göster
            cv2.imshow("RGB Camera Image", cv_image)
            cv2.waitKey(1)  # OpenCV'nin GUI'sini güncel tutar

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridgeError: {e}')
    
    def depth_listener_callback(self, msg):
        try:
            # Derinlik görüntüsünü OpenCV formatına dönüştür
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            # Görüntüyü ekranda göster
            cv2.imshow("Depth Camera Image", cv_image)
            cv2.waitKey(1)  # OpenCV'nin GUI'sini güncel tutar

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridgeError: {e}')

def main(args=None):
    rclpy.init(args=args)  # ROS 2'yi başlat
    node = ImageSubscriber()  # Node'u oluştur

    try:
        rclpy.spin(node)  # Node'u çalıştır (bu, sürekli olarak gelen verileri dinleyecek)
    except KeyboardInterrupt:
        pass

    node.destroy_node()  # Node'u yok et
    rclpy.shutdown()  # ROS 2'yi kapat
    cv2.destroyAllWindows()  # OpenCV penceresini kapat

if __name__ == '__main__':
    main()

