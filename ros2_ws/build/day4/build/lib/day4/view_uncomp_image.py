import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import threading
import signal
import sys

class RGBDepthViewer(Node):
    def __init__(self, namespace):
        super().__init__('rgb_depth_viewer')
        self.rgb_image = None
        self.depth_image = None
        self.running = True

        # Aligned RGB and depth topics
        rgb_topic = f"{namespace}/oakd/rgb/image_raw"
        depth_topic = f"{namespace}/oakd/rgb/preview/depth"

        self.rgb_sub = self.create_subscription(
            Image,
            rgb_topic,
            self.rgb_callback,
            10)

        self.depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            10)

        self.display_thread = threading.Thread(target=self.display_loop)
        self.display_thread.start()

    def rgb_callback(self, msg):
        try:
            rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            self.rgb_image = rgb
        except Exception as e:
            self.get_logger().error(f"RGB conversion failed: {e}")

    def depth_callback(self, msg):
        try:
            dtype = np.uint16 if msg.encoding == '16UC1' else np.float32
            depth = np.frombuffer(msg.data, dtype=dtype).reshape((msg.height, msg.width))
            self.depth_image = depth
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

    def display_loop(self):
        cv2.namedWindow("RGB", cv2.WINDOW_NORMAL)
        cv2.moveWindow("RGB", 0, 0)
        cv2.namedWindow("Depth", cv2.WINDOW_NORMAL)
        cv2.moveWindow("Depth", 720, 0)

        while self.running:
            if self.rgb_image is not None:
                cv2.imshow("RGB", self.rgb_image)

            if self.depth_image is not None:
                center_x = self.depth_image.shape[1] // 2
                center_y = self.depth_image.shape[0] // 2
                distance_mm = float(self.depth_image[center_y, center_x])
                distance_m = distance_mm / 1000.0 if self.depth_image.dtype == np.uint16 else distance_mm

                depth_vis = cv2.normalize(self.depth_image, None, 0, 255, cv2.NORM_MINMAX)
                depth_vis = np.uint8(depth_vis)
                depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

                cv2.circle(depth_color, (center_x, center_y), 5, (0, 255, 255), -1)
                cv2.putText(depth_color, f"Depth: {distance_m:.2f} m", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.imshow("Depth", depth_color)

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
