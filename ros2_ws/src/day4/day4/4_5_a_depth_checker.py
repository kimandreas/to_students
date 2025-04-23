import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import signal
import sys

class RGBDepthViewer(Node):
    def __init__(self, namespace):
        super().__init__('rgb_depth_viewer')
        self.bridge = CvBridge()
        self.rgb_image_raw = None
        self.rgb_image_compressed = None
        self.depth_image = None
        self.running = True

        # raw_rgb_topic = f"{namespace}/oakd/rgb/image_raw"
        compressed_rgb_topic = f"{namespace}/oakd/rgb/image_raw/compressed"
        depth_topic = f"{namespace}/oakd/stereo/image_raw"

        # self.rgb_raw_sub = self.create_subscription(
        #     Image,
        #     raw_rgb_topic,
        #     self.rgb_raw_callback,
        #     10)

        self.rgb_compressed_sub = self.create_subscription(
            CompressedImage,
            compressed_rgb_topic,
            self.rgb_compressed_callback,
            10)

        self.depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            10)

        self.display_thread = threading.Thread(target=self.display_loop)
        self.display_thread.start()

    def rgb_raw_callback(self, msg):
        try:
            raw_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.rgb_image_raw = cv2.resize(raw_img, (640, 480))
        except Exception as e:
            self.get_logger().error(f"RGB raw conversion failed: {e}")

    def rgb_compressed_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            decoded = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.rgb_image_compressed = cv2.resize(decoded, (640, 480))
        except Exception as e:
            self.get_logger().error(f"RGB compressed conversion failed: {e}")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

    def display_loop(self):
        # cv2.namedWindow("RGB Raw", cv2.WINDOW_NORMAL)
        # cv2.moveWindow("RGB Raw", 0, 0)
        cv2.namedWindow("RGB Compressed", cv2.WINDOW_NORMAL)
        cv2.moveWindow("RGB Compressed", 640, 0)
        cv2.namedWindow("Depth", cv2.WINDOW_NORMAL)
        cv2.moveWindow("Depth", 1280, 0)

        while self.running:
            # if self.rgb_image_raw is not None:
            #     cv2.imshow("RGB Raw", self.rgb_image_raw)

            if self.depth_image is not None:
                center_x = self.depth_image.shape[1] // 2
                center_y = self.depth_image.shape[0] // 4*3
                distance_mm = float(self.depth_image[center_y, center_x])
                distance_m = distance_mm / 1000.0

                depth_vis = cv2.normalize(self.depth_image, None, 0, 255, cv2.NORM_MINMAX)
                depth_vis = np.uint8(depth_vis)
                depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

                cv2.circle(depth_color, (center_x, center_y), 5, (0, 255, 255), -1)
                cv2.putText(depth_color, f"Depth: {distance_m:.2f} m", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.imshow("Depth", depth_color)

                if self.rgb_image_compressed is not None:
                    cv2.circle(self.rgb_image_compressed, (center_x, center_y), 5, (0, 255, 255), -1)
                    cv2.imshow("RGB Compressed", self.rgb_image_compressed)

            key = cv2.waitKey(1)
            if key == ord('q'):
                self.get_logger().info("Exit requested by user.")
                self.running = False
                rclpy.shutdown()
                break

        cv2.destroyAllWindows()

    def shutdown(self):
        if self.running:
            self.get_logger().info("Shutting down display loop.")
            self.running = False
            if threading.current_thread() != self.display_thread:
                self.display_thread.join()


def main(args=None):
    namespace = input("Enter the robot namespace (e.g., /robot0): ").strip()
    if not namespace.startswith('/'):
        namespace = '/' + namespace

    rclpy.init(args=args)
    node = RGBDepthViewer(namespace)

    def sigint_handler(signum, frame):
        print("Keyboard interrupt, shutting down.")
        node.shutdown()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, sigint_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
