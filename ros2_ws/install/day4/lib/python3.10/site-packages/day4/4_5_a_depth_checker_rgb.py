import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from threading import Thread, Event

class RGBDViewer(Node):
    def __init__(self):
        super().__init__('rgbd_viewer')
        self.bridge = CvBridge()

        self.rgb_image = None
        self.depth_image = None

        self.rgb_sub = self.create_subscription(
            CompressedImage,
            '/robot0/oakd/rgb/image_raw/compressed',
            self.rgb_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            CompressedImage,
            '/robot0/oakd/rgb/image_raw/compressedDepth',
            self.depth_callback,
            10
        )

        self.running = Event()
        self.running.set()

        self.display_thread = Thread(target=self.display_loop)
        self.display_thread.start()

    def rgb_callback(self, msg):
        try:
            self.rgb_image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"RGB decode failed: {e}")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth decode failed: {e}")

    def display_loop(self):
        cv2.namedWindow('RGB')
        cv2.namedWindow('Depth')
        while self.running.is_set():
            if self.rgb_image is not None:
                cv2.imshow('RGB', self.rgb_image)
            if self.depth_image is not None:
                depth_vis = cv2.normalize(self.depth_image, None, 0, 255, cv2.NORM_MINMAX)
                depth_vis = np.uint8(depth_vis)
                depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                cv2.imshow('Depth', depth_color)
            key = cv2.waitKey(1)
            if key == ord('q'):
                self.get_logger().info("Exit requested.")
                self.running.clear()
                rclpy.shutdown()
                break
        cv2.destroyAllWindows()

def main():
    rclpy.init()
    node = RGBDViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.running.clear()
    node.display_thread.join()
    node.destroy_node()

if __name__ == '__main__':
    main()