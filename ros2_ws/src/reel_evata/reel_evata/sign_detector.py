#!/usr/bin/env python3.10

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String, Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from ultralytics import YOLO
import logging
from sensor_msgs_py import point_cloud2
import torch
from collections import defaultdict, deque
from statistics import median
import os
logging.getLogger('ultralytics').setLevel(logging.ERROR)

class SignDetector(Node):
    def __init__(self):
        super().__init__('sign_detector_node')

        self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"Using device: {self.device}")
        weights_path = "/home/akif/evata_2025/ros2_ws/src/evata_sim/evata_sim/bestcihan.pt"
        self.model = YOLO(weights_path).to(self.device)
        self.model.fuse()
        
        self.bridge = CvBridge()
        self.process_width = 1280
        self.process_height = 720

        self.tracked_signs = {}
        self.latest_pointcloud = None

        self.distance_history = defaultdict(lambda: deque(maxlen=7))
        self.smoothed_distance = {}
        self.smoothing_alpha = 0.3

        self.MIN_VISUAL_DIST = 0.7
        self.MAX_VISUAL_DIST = 30.0
        self.MAX_PUBLISH_DIST = 8.0

        self.setup_ros()

    def setup_ros(self):
        self.create_subscription(PointCloud2, "/zed/zed_node/point_cloud/cloud_registered", self.point_cloud_callback, 10)
        self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.color_image_callback, 10)

        self.sign_info_pub = self.create_publisher(String, '/sign_detector/sign_info', 10)
        self.position_pub = self.create_publisher(Float32MultiArray, '/sign_detector/position', 10)

        cv2.namedWindow("Sign Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Sign Detection", self.process_width, self.process_height)

    def point_cloud_callback(self, msg):
        self.latest_pointcloud = msg

    def color_image_callback(self, msg):
        if not msg.data or len(msg.data) == 0:
            self.get_logger().warn("Received empty image data!")
            return

        try:
            encoding = msg.encoding
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            if encoding == 'bgra8':
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)
            elif encoding == 'bgr8':
                pass
            elif encoding == 'rgb8':
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            else:
                self.get_logger().warn(f"Unexpected encoding '{encoding}', assuming BGR8")

            resized_img = cv2.resize(cv_image, (self.process_width, self.process_height))

            results = self.model(resized_img, imgsz=(self.process_width, self.process_height),
                                 device=self.device, conf=0.5, iou=0.45)

            self.process_detections(results, resized_img)

            cv2.putText(resized_img, f"Threshold: {self.MIN_VISUAL_DIST:.1f}m - {self.MAX_VISUAL_DIST:.1f}m",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

            cv2.imshow("Sign Detection", resized_img)
            cv2.waitKey(1)

        except CvBridgeError as cve:
            self.get_logger().error(f"CvBridge error: {str(cve)}")
        except Exception as e:
            self.get_logger().error(f"Image processing error: {str(e)}")

    def process_detections(self, results, image):
        current_detections = defaultdict(list)

        for r in results:
            for box in r.boxes:
                if box.conf > 0.5:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    class_id = int(box.cls)
                    class_name = self.model.names[class_id]
                    conf = box.conf.item() if hasattr(box.conf, 'item') else float(box.conf)
                    current_detections[class_id].append((x1, y1, x2, y2, class_name, conf))

        self.tracked_signs.clear()

        for class_id, detections in current_detections.items():
            for idx, det in enumerate(detections):
                x1, y1, x2, y2, class_name, conf = det
                unique_id = f"{class_id}_{idx}"

                dist_raw = self.calculate_pointcloud_distance(x1, y1, x2, y2)
                if dist_raw > 0:
                    self.distance_history[unique_id].append(dist_raw)
                else:
                    if len(self.distance_history[unique_id]) == 0:
                        self.distance_history[unique_id].append(-1)

                valid_distances = [d for d in self.distance_history[unique_id] if d > 0]

                if len(valid_distances) >= 3:
                    q1 = np.percentile(valid_distances, 25)
                    q3 = np.percentile(valid_distances, 75)
                    iqr = q3 - q1
                    filtered = [d for d in valid_distances if q1 - 1.5 * iqr <= d <= q3 + 1.5 * iqr]
                    median_dist = median(filtered) if filtered else median(valid_distances)
                else:
                    median_dist = median(valid_distances) if valid_distances else -1

                prev_smooth = self.smoothed_distance.get(unique_id, median_dist if median_dist > 0 else -1)
                if median_dist > 0 and prev_smooth > 0:
                    smooth_dist = self.smoothing_alpha * median_dist + (1 - self.smoothing_alpha) * prev_smooth
                else:
                    smooth_dist = median_dist

                self.smoothed_distance[unique_id] = smooth_dist

                self.tracked_signs[unique_id] = (x1, y1, x2, y2, class_name, conf, smooth_dist)

        for unique_id, (x1, y1, x2, y2, class_name, conf, distance) in self.tracked_signs.items():
            if self.MIN_VISUAL_DIST <= distance <= self.MAX_VISUAL_DIST:
                self.visualize_detection(image, x1, y1, x2, y2, class_name, distance, conf)

                if distance <= self.MAX_PUBLISH_DIST:
                    self.publish_sign_info(class_name, distance, x1, y1, x2, y2)
            else:
                self.visualize_detection(image, x1, y1, x2, y2, class_name, -1, conf)

    def calculate_pointcloud_distance(self, x1, y1, x2, y2):
        if self.latest_pointcloud is None:
            self.get_logger().warn("No pointcloud data received yet", throttle_duration_sec=5)
            return -1

        try:
            pc_np = point_cloud2.read_points_numpy(self.latest_pointcloud, field_names=("x", "y", "z"))
            pc_np = np.reshape(pc_np, (self.latest_pointcloud.height, self.latest_pointcloud.width, 3))

            scale_x = self.latest_pointcloud.width / self.process_width
            scale_y = self.latest_pointcloud.height / self.process_height

            center_x = int((x1 + x2) / 2 * scale_x)
            center_y = int((y1 + y2) / 2 * scale_y)

            distances = []
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    px = np.clip(center_x + dx, 0, self.latest_pointcloud.width - 1)
                    py = np.clip(center_y + dy, 0, self.latest_pointcloud.height - 1)
                    point = pc_np[py, px, :]
                    if np.all(np.isfinite(point)):
                        dist = np.linalg.norm(point)
                        if 0.3 < dist < 20.0:
                            distances.append(dist)

            if len(distances) < 5:
                self.get_logger().warn("Not enough valid points", throttle_duration_sec=5)
                return -1

            # Outlier filtrele
            q1 = np.percentile(distances, 25)
            q3 = np.percentile(distances, 75)
            iqr = q3 - q1
            filtered = [d for d in distances if q1 - 1.5 * iqr <= d <= q3 + 1.5 * iqr]

            return float(np.median(filtered)) if filtered else float(np.median(distances))

        except Exception as e:
            self.get_logger().error(f"Distance calc error: {str(e)}", throttle_duration_sec=5)
            return -1

    def visualize_detection(self, image, x1, y1, x2, y2, class_name, distance, conf):
        if distance > 0:
            if distance < 5.0:
                color = (0, 0, 255)
            elif distance < 10.0:
                color = (0, 165, 255)
            else:
                color = (0, 255, 0)
        else:
            color = (255, 0, 0)

        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)

        text = f"{class_name}"
        if distance > 0:
            text += f" {distance:.1f}m"
        text += f" ({conf:.2f})"

        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(image, (x1, y1 - th - 10), (x1 + tw, y1), color, -1)
        cv2.putText(image, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    def publish_sign_info(self, class_name, distance, x1, y1, x2, y2):
        sign_info_msg = String()
        sign_info_msg.data = f"{class_name}:{distance:.2f}" if distance > 0 else f"{class_name}:nan"
        self.sign_info_pub.publish(sign_info_msg)

        if class_name in ['park', 'engellipark', 'engellipark_ters']:
            position_msg = Float32MultiArray()
            position_msg.data = [float(x1), float(y1), float(x2), float(y2), float(self.process_width)]
            self.position_pub.publish(position_msg)

        self.get_logger().info(f"Published: {class_name} at {distance:.2f}m")

def main(args=None):
    rclpy.init(args=args)
    node = SignDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped cleanly")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

