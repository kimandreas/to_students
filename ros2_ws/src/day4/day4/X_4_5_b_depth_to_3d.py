import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import message_filters
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
import numpy as np
import cv2
import sys

class DepthToMap(Node):
    def __init__(self, namespace):
        super().__init__('depth_to_map_node')
        self.bridge = CvBridge()
        self.namespace = namespace.strip('/')  # Remove leading/trailing slashes
        self.shutdown_requested = False  # Flag to handle shutdown

        depth_topic = f'/{self.namespace}/oakd/stereo/image_raw'
        info_topic  = f'/{self.namespace}/oakd/stereo/camera_info'

        self.get_logger().info(f"Subscribing to:\n  Depth: {depth_topic}\n  CameraInfo: {info_topic}")

        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self.info_sub  = message_filters.Subscriber(self, CameraInfo, info_topic)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.depth_sub, self.info_sub], 10, 0.1)
        self.ts.registerCallback(self.callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def callback(self, depth_msg, info_msg):
        if self.shutdown_requested:
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            height, width = depth_image.shape
            cx = width // 2
            cy = height // 2
            depth = float(depth_image[cy, cx]) / 1000.0  # mm to m

            if np.isnan(depth) or depth <= 0.1:
                self.get_logger().warn(f"Invalid depth at center pixel: {depth}")
                return

            fx = info_msg.k[0]
            fy = info_msg.k[4]
            cx_intr = info_msg.k[2]
            cy_intr = info_msg.k[5]

            x = (cx - cx_intr) * depth / fx
            y = (cy - cy_intr) * depth / fy
            z = depth

            camera_point = PointStamped()
            camera_point.header = depth_msg.header
            camera_point.point.x = x
            camera_point.point.y = y
            camera_point.point.z = z

            map_frame = 'map'
            source_frame = depth_msg.header.frame_id
            self.get_logger().info(f"Source frame: {source_frame}")

            if not self.tf_buffer.can_transform(map_frame, source_frame, depth_msg.header.stamp):
                self.get_logger().warn("Transform not available yet, skipping...")
                return

            transform = self.tf_buffer.lookup_transform(
                target_frame=map_frame,
                source_frame=source_frame,
                time=depth_msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            map_point = tf2_geometry_msgs.do_transform_point(camera_point, transform)
            self.get_logger().info(f"Map coordinates: ({map_point.point.x:.2f}, {map_point.point.y:.2f}, {map_point.point.z:.2f})")

            # Visualize depth image
            display_image = (depth_image / np.nanmax(depth_image) * 255.0).astype(np.uint8)
            display_image = cv2.applyColorMap(display_image, cv2.COLORMAP_JET)

            cv2.imshow("Depth Image", display_image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info("Shutdown requested by pressing 'q'.")
                self.shutdown_requested = True
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().warn(f"Transform or projection failed: {e}")

def main():
    ns = input("Enter the robot namespace (e.g., 'robot0'): ").strip()
    rclpy.init()
    node = DepthToMap(ns)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt detected. Shutting down node.")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
