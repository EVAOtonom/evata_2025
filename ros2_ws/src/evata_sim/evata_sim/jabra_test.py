
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/rgb',  # Kameranızın görüntü topiğini buraya yazın
            self.listener_callback,
            10)
        self.subscription  # Abonelik değişkeni kullanılmazsa uyarı alabilirsiniz
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # ROS Image mesajını OpenCV formatına dönüştür
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Görüntüyü ekranda göster
        cv2.imshow("Camera Image", cv_image)
        cv2.waitKey(1)  # OpenCV'nin GUI'sini güncel tutar

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
