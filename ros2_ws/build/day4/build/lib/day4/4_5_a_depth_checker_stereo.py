import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading


class RGBDepthViewer(Node):
    def __init__(self):
        super().__init__('rgb_depth_viewer')
        self.bridge = CvBridge()

        # Topics
        self.rgb_topic = '/robot0/oakd/rgb/preview/image_raw'
        self.depth_topic = '/robot0/oakd/stereo/image_raw'

        # Image buffers
        self.rgb_image = None
        self.depth_image = None

        self.rgb_lock = threading.Lock()
        self.depth_lock = threading.Lock()

        # Subscriptions
        self.create_subscription(Image, self.rgb_topic, self.rgb_callback, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)

        # Display thread
        self.display_thread = threading.Thread(target=self.display_loop)
        self.display_thread.daemon = True
        self.display_thread.start()

    def rgb_callback(self, msg):
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.rgb_lock:
                self.rgb_image = rgb
        except Exception as e:
            self.get_logger().error(f"RGB conversion failed: {e}")

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            with self.depth_lock:
                self.depth_image = depth
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

    def display_loop(self):
        cv2.namedWindow("RGB Image", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Depth Image", cv2.WINDOW_NORMAL)
        cv2.moveWindow("RGB Image", 0, 0)
        cv2.moveWindow("Depth Image", 700, 0)

        while rclpy.ok():
            rgb = None
            depth = None
            with self.rgb_lock:
                if self.rgb_image is not None:
                    rgb = self.rgb_image.copy()
            with self.depth_lock:
                if self.depth_image is not None:
                    depth = self.depth_image.copy()

            if rgb is not None:
                cv2.imshow("RGB Image", rgb)

            if depth is not None:
                norm = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
                colorized = cv2.applyColorMap(norm.astype(np.uint8), cv2.COLORMAP_JET)
                h, w = depth.shape
                dist_mm = float(depth[h // 2, w // 2])
                dist_m = dist_mm / 1000.0
                cv2.circle(colorized, (w // 2, h // 2), 5, (0, 255, 255), -1)
                cv2.putText(colorized, f"{dist_m:.2f} m", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.imshow("Depth Image", colorized)

            key = cv2.waitKey(1)
            if key == ord('q'):
                rclpy.shutdown()
                break

        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    viewer = RGBDepthViewer()
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    viewer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
