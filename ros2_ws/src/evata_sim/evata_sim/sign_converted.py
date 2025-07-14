#!/usr/bin/env python3.10

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import logging
from sensor_msgs_py import point_cloud2
from ament_index_python.packages import get_package_share_directory
import math
import json
import time

logging.getLogger('ultralytics').setLevel(logging.ERROR)

class SignDetector(Node):
    def __init__(self):
        super().__init__('sign_detector_node')
        package_path = get_package_share_directory('evata_sim')
        model_path = os.path.join(package_path, 'model2', 'best.pt')

        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.fx = 277.0
        self.tracked_signs = {}
        self.latest_pointcloud = None
        self.last_detection_time = time.time()
        self.detection_interval = 0.15

        self.create_subscription(Image, "/depth_camera/zed/image", self.color_image_callback, 10)
        self.create_subscription(CameraInfo, "/depth_camera/zed/camera_info", self.camera_info_callback, 10)
        self.create_subscription(PointCloud2, "/depth_camera/zed/points", self.point_cloud_callback, 10)

        self.sign_publisher = self.create_publisher(String, "/detected_signs", 10)

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]

    def point_cloud_callback(self, msg):
        self.latest_pointcloud = msg

    def color_image_callback(self, msg):
        now = time.time()
        if now - self.last_detection_time < self.detection_interval:
            return
        self.last_detection_time = now

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            original_image = cv_image.copy()

            resized_image = cv2.resize(cv_image, (640, 360))
            scale_x = cv_image.shape[1] / 640
            scale_y = cv_image.shape[0] / 360

            results = self.model(resized_image, verbose=False)
            self.annotated_image = original_image

            # 🟡 Sol üst köşeye threshold bilgisi yaz
            cv2.putText(self.annotated_image, "Threshold: 0.7m - 12.0m", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            detected_signs = {}
            sign_data = {}

            for r in results:
                for box in r.boxes:
                    if box.conf > 0.6:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        x1 = int(x1 * scale_x)
                        y1 = int(y1 * scale_y)
                        x2 = int(x2 * scale_x)
                        y2 = int(y2 * scale_y)
                        class_name = self.model.names[int(box.cls)]
                        confidence = float(box.conf)
                        detected_signs[class_name] = (x1, y1, x2, y2, 0, confidence)

            updated_tracked_signs = {}
            for class_name, values in self.tracked_signs.items():
                if class_name in detected_signs:
                    updated_tracked_signs[class_name] = detected_signs[class_name]
                elif values[4] < 5:
                    updated_tracked_signs[class_name] = (*values[:4], values[4] + 1, values[5])
            for class_name, values in detected_signs.items():
                if class_name not in updated_tracked_signs:
                    updated_tracked_signs[class_name] = values
            self.tracked_signs = updated_tracked_signs

            for class_name, (x1, y1, x2, y2, _, confidence) in self.tracked_signs.items():
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                distance = self.calculate_distance_pointcloud(center_x, center_y)

                if distance == -1:
                    distance = self.calculate_distance(x1, y1, x2, y2)

                # 🟥 Mesafe threshold kontrolü
                if distance < 0.7 or distance > 12.0:
                    continue

                self._draw_box(x1, y1, x2, y2, class_name, distance, confidence)

                if distance <= 7.0:
                    sign_data[class_name] = round(distance, 2)

            if sign_data:
                msg = String()
                msg.data = json.dumps(sign_data)
                self.sign_publisher.publish(msg)
                self.get_logger().info(f"Published: {msg.data}")

            if self.annotated_image.shape[0] > 0:
                cv2.imshow("Levha Tespiti", self.annotated_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Görüntü Hatası: {e}")

    def calculate_distance_pointcloud(self, center_x, center_y):
        if self.latest_pointcloud is None:
            return -1
        try:
            width = self.latest_pointcloud.width
            height = self.latest_pointcloud.height
            center_x = min(center_x, width - 1)
            center_y = min(center_y, height - 1)
            index = center_y * width + center_x

            gen = point_cloud2.read_points(self.latest_pointcloud, field_names=("x", "y", "z"), skip_nans=False)
            for i, pt in enumerate(gen):
                if i == index:
                    x, y, z = pt
                    if math.isnan(z) or math.isinf(z):
                        return -1
                    return math.sqrt(x**2 + y**2 + z**2)
            return -1
        except Exception as e:
            self.get_logger().error(f"PointCloud Hatası: {e}")
            return -1

    def calculate_distance(self, x1, y1, x2, y2):
        real_width = 0.5
        bbox_width = max(x2 - x1, 1)
        return (real_width * self.fx) / bbox_width * 1.7

    def _draw_box(self, x1, y1, x2, y2, class_name, distance, confidence):
        cv2.rectangle(self.annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"{class_name}: {distance:.2f}m ({confidence:.2f})"
        cv2.putText(self.annotated_image, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

def main(args=None):
    rclpy.init(args=args)
    node = SignDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Klavye ile çıkıldı.")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
