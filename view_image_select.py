import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import signal
import time

# Global shared variables
latest_rgb_raw = None
latest_rgb_compressed = None
latest_rgb_image_raw = None
latest_depth = None
shutdown_requested = False
display_mode = 1  # default: preview + depth

class RGBDepthViewer(Node):
    def __init__(self):
        super().__init__('rgb_depth_viewer')
        self.bridge = CvBridge()

        namespace = self.get_namespace()
        raw_rgb_topic = f"{namespace}/oakd/rgb/preview/image_raw"
        compressed_rgb_topic = f"{namespace}/oakd/rgb/image_raw/compressed"
        rgb_image_raw_topic = f"{namespace}/oakd/rgb/image_raw"
        depth_topic = f"{namespace}/oakd/stereo/image_raw"

        self.rgb_raw_sub = self.create_subscription(
            Image, raw_rgb_topic, self.rgb_raw_callback, 10)
        self.rgb_compressed_sub = self.create_subscription(
            CompressedImage, compressed_rgb_topic, self.rgb_compressed_callback, 10)
        self.rgb_image_raw_sub = self.create_subscription(
            Image, rgb_image_raw_topic, self.rgb_image_raw_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self.depth_callback, 10)

    def rgb_raw_callback(self, msg):
        global latest_rgb_raw
        try:
            raw_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            latest_rgb_raw = cv2.resize(raw_img, (640, 480))
        except Exception as e:
            self.get_logger().error(f"RGB raw conversion failed: {e}")

    def rgb_compressed_callback(self, msg):
        global latest_rgb_compressed
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            decoded = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            latest_rgb_compressed = cv2.resize(decoded, (640, 480))
        except Exception as e:
            self.get_logger().error(f"RGB compressed conversion failed: {e}")

    def rgb_image_raw_callback(self, msg):
        global latest_rgb_image_raw
        try:
            raw_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            latest_rgb_image_raw = cv2.resize(raw_img, (640, 480))
        except Exception as e:
            self.get_logger().error(f"RGB image_raw conversion failed: {e}")

    def depth_callback(self, msg):
        global latest_depth
        try:
            latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

def display_loop():
    global shutdown_requested, display_mode

    font = cv2.FONT_HERSHEY_SIMPLEX
    label_height = 40
    border = 5

    current_window = None

    def prepare_panel(img, label):
        if img is None:
            return None
        h, w = img.shape[:2]
        label_bar = np.full((label_height, w, 3), 255, dtype=np.uint8)
        cv2.putText(label_bar, label, (10, 28), font, 0.8, (0, 0, 0), 2)
        return np.vstack((label_bar, img))

    while not shutdown_requested:
        # Select RGB image based on mode
        if display_mode == 1:
            rgb = latest_rgb_raw
            label = "Preview RGB"
            mode_name = "Preview RGB + Depth"
        elif display_mode == 2:
            rgb = latest_rgb_image_raw
            label = "RGB Image Raw"
            mode_name = "Image Raw RGB + Depth"
        elif display_mode == 3:
            rgb = latest_rgb_compressed
            label = "Compressed RGB"
            mode_name = "Compressed RGB + Depth"
        else:
            rgb = None
            label = "Invalid Mode"
            mode_name = "Invalid Mode"

        rgb_panel = prepare_panel(rgb, label)

        # Handle depth
        depth_panel = None
        if latest_depth is not None:
            depth = latest_depth
            h, w = depth.shape
            center_x, center_y = w // 2, h // 2
            distance_mm = float(depth[center_y, center_x])
            distance_m = distance_mm / 1000.0

            depth_vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
            depth_vis = np.uint8(depth_vis)
            depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            cv2.circle(depth_color, (center_x, center_y), 5, (0, 255, 255), -1)
            cv2.putText(depth_color, f"{distance_m:.2f} m", (10, 30), font, 0.7, (255, 255, 255), 2)

            depth_panel = prepare_panel(depth_color, "Depth Map")

        # Resize border based on actual height
        panel_h = rgb_panel.shape[0] if rgb_panel is not None else (
            depth_panel.shape[0] if depth_panel is not None else 520)
        border_img = 255 * np.ones((panel_h, border, 3), dtype=np.uint8)

        # Combine available panels
        panels = [p for p in [rgb_panel, border_img, depth_panel] if p is not None]
        combined = np.hstack(panels) if panels else np.zeros((480, 640, 3), dtype=np.uint8)

        # Update window
        window_name = f"Camera Feeds - {mode_name}"
        if current_window != window_name:
            if current_window:
                cv2.destroyWindow(current_window)
            current_window = window_name
            cv2.namedWindow(current_window, cv2.WINDOW_NORMAL)

        cv2.imshow(current_window, combined)

        key = cv2.waitKey(1)
        if key == ord('1'):
            display_mode = 1
        elif key == ord('2'):
            display_mode = 2
        elif key == ord('3'):
            display_mode = 3
        elif key == ord('q'):
            shutdown_requested = True
            break

        time.sleep(0.01)

    if current_window:
        cv2.destroyWindow(current_window)


def main():
    global shutdown_requested

    rclpy.init()
    node = RGBDepthViewer()
    executor_thread = threading.Thread(target=rclpy.spin, args=(node,))
    executor_thread.start()

    def handle_sigint(sig, frame):
        print("KeyboardInterrupt received. Shutting down...")
        shutdown_requested = True

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        display_loop()
    finally:
        shutdown_requested = True
        rclpy.shutdown()
        executor_thread.join()
        node.destroy_node()


if __name__ == '__main__':
    main()
