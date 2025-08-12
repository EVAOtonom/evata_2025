#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class AmclPoseFilter(Node):
    def __init__(self):
        super().__init__("amcl_pose_filter")

        # Parametreler
        self.max_distance = 1.0  # metre
        self.max_yaw_diff = math.radians(5)  # 5 derece
        self.base_frame = "base_footprint"
        self.odom_frame = "odom"
        self.map_frame = "map"

        self.last_pose = None  # Son kabul edilen poz

        # Subscriber: AMCL çıkışı
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.pose_callback,
            10
        )

        # Publisher: Filtrelenmiş pose
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/filtered_amcl_pose",
            10
        )

        # TF Publisher
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("✅ AMCL pose filter + TF yayını aktif.")

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        # İlk poz ise direkt kabul et
        if self.last_pose is None:
            self.last_pose = msg
            self.publish_pose_and_tf(msg)
            return

        # Eski poz bilgisi
        last_x = self.last_pose.pose.pose.position.x
        last_y = self.last_pose.pose.pose.position.y
        last_yaw = self.quaternion_to_yaw(self.last_pose.pose.pose.orientation)

        # Yeni poz bilgisi
        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.y
        new_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.odom_frame

        # Mesafe farkı
        dist = math.sqrt((new_x - last_x) ** 2 + (new_y - last_y) ** 2)
        # Yaw farkı (normalize edilmiş)
        yaw_diff = abs((new_yaw - last_yaw + math.pi) % (2 * math.pi) - math.pi)

        if dist <= self.max_distance and yaw_diff <= self.max_yaw_diff:
            self.last_pose = msg
            self.publish_pose_and_tf(msg)
        else:
            self.get_logger().warn(
                f"❌ Pozisyon reddedildi: Δd={dist:.2f} m, Δyaw={math.degrees(yaw_diff):.2f}°"
            )

    def publish_pose_and_tf(self, pose_msg: PoseWithCovarianceStamped):
        # Filtrelenmiş pose'u yayınla
        self.pose_pub.publish(pose_msg)

        # TF oluştur ve yayınla (map → odom)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = pose_msg.pose.pose.position.x
        t.transform.translation.y = pose_msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = pose_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def quaternion_to_yaw(self, q):
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw


def main(args=None):
    rclpy.init(args=args)
    node = AmclPoseFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
