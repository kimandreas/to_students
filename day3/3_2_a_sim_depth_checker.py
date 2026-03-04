# depth_checker.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

class DepthChecker(Node):
    def __init__(self):
        super().__init__('depth_checker')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/oakd/rgb/preview/depth',
            self.depth_callback,
            10)

    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        height, width = depth_image.shape

        u, v = width // 2, height // 2  # dynamically get center pixel
        distance = depth_image[v, u]
        self.get_logger().info(f"Image size: {width}x{height}, Distance at ({u},{v}) = {distance:.2f} meters")


def main():
    rclpy.init()
    node = DepthChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
