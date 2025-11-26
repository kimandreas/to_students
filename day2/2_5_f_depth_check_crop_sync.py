#!/usr/bin/env python3
"""
Depth‑click demo – **timestamp‑synchronised RGB & cropped Stereo depth**
-----------------------------------------------------------------------
* Uses *ApproximateTimeSynchronizer* to guarantee each RGB frame matches the
  depth frame you crop/resize, so the pixel you click refers to the *same* instant.
* Keeps the *original* crop factors already in your code (26 % width, 18 % height).
* Outputs a side‑by‑side preview window (RGB‑left | depth‑right). Clicking on the
  RGB pane prints the distance at that pixel.

Topics consumed
===============
* `/ns/oakd/rgb/image_raw/compressed`   – colour 640×480 (BGR8 JPEG)
* `/ns/oakd/stereo/image_raw` - depth 640×480 (mono16, mm)
* `/ns/oakd/rgb/camera_info` - for intrinsics only (back‑projection not used here)

RGB FOV, draw text, and dock/undock via *TurtleBot4Navigator*.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge
from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Navigator, TurtleBot4Directions,
)

import numpy as np
import cv2
import threading
from message_filters import Subscriber, ApproximateTimeSynchronizer


class DepthToMap(Node):
    def __init__(self):
        super().__init__("depth_to_map_node")

        self.bridge = CvBridge()
        self.K: np.ndarray | None = None
        self.lock = threading.Lock()

        # ───────────────── topic names ─────────────────
        ns = self.get_namespace()
        self.depth_topic = f"{ns}/oakd/stereo/image_raw"           # uint16, mm
        self.rgb_topic   = f"{ns}/oakd/rgb/image_raw/compressed"  # BGR8 JPEG
        self.info_topic  = f"{ns}/oakd/rgb/camera_info"

        # shared state
        self.depth_image: np.ndarray | None = None   # after crop/resize
        self.rgb_image  : np.ndarray | None = None
        self.clicked_point: tuple[int, int] | None = None
        self.display_image: np.ndarray | None = None

        # ───────────────── navigator stuff (unchanged) ─────────────────
        self.navigator = TurtleBot4Navigator()
        if not self.navigator.getDockedStatus():
            self.get_logger().info("Docking before initializing pose")
            self.navigator.dock()
        self.navigator.undock()

        # ───────────────── publishers / subscribers ─────────────────
        self.create_subscription(CameraInfo, self.info_topic, self.camera_info_callback, 1)

        rgb_sub   = Subscriber(self, CompressedImage, self.rgb_topic)
        depth_sub = Subscriber(self, Image,          self.depth_topic)
        self.ts   = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=20, slop=0.1)
        self.ts.registerCallback(self.synced_callback)

        # ───────────────── UI thread ─────────────────
        self.gui_thread_stop = threading.Event()
        self.gui_thread = threading.Thread(target=self.gui_loop, daemon=True)
        self.gui_thread.start()

        # run display loop at 5 Hz (adjust as needed)
        self.timer = self.create_timer(0.2, self.compose_preview)

    # ───────────────── Callbacks ─────────────────────────────────────
    def camera_info_callback(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
            self.get_logger().info(f"Camera intrinsics fx={fx:.2f} fy={fy:.2f} cx={cx:.1f} cy={cy:.1f}")

    def synced_callback(self, rgb_msg: CompressedImage, depth_msg: Image):
        """Called when RGB+depth share (nearly) the same timestamp."""
        try:
            # decode
            rgb = cv2.imdecode(np.frombuffer(rgb_msg.data, np.uint8), cv2.IMREAD_COLOR)
            depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")  # mono16 mm

            if rgb is None or depth_raw is None:
                return

            # ─ crop stereo depth to approximate RGB FOV ─
            h, w = depth_raw.shape              # 480 × 640 expected
            crop_x = int(0.26 * w / 2) * 2      # 26 % total width ⇒ 13 % per side
            crop_y = int(0.18 * h / 2) * 2      # 18 % total height ⇒  9 % top/bot
            depth_crop = depth_raw[crop_y : h - crop_y, crop_x : w - crop_x]
            depth_aligned = cv2.resize(depth_crop, (w, h), cv2.INTER_NEAREST)

            with self.lock:
                self.rgb_image   = rgb
                self.depth_image = depth_aligned
        except Exception as e:
            self.get_logger().error(f"Sync callback failed: {e}")

    # ───────────────── Preview generation ────────────────────────────
    def compose_preview(self):
        with self.lock:
            rgb   = None if self.rgb_image   is None else self.rgb_image.copy()
            depth = None if self.depth_image is None else self.depth_image.copy()
            click = self.clicked_point

        if rgb is None or depth is None:
            return

        try:
            depth_vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
            depth_vis = cv2.applyColorMap(depth_vis.astype(np.uint8), cv2.COLORMAP_JET)

            if click:
                x, y = click
                if 0 <= x < 640 and 0 <= y < 480:
                    z_m = depth[y, x] / 1000.0
                    text = f"{z_m:.2f} m" if 0.2 < z_m < 5.0 else "Invalid"
                    for img in (rgb, depth_vis):
                        cv2.drawMarker(img, (x, y), (0, 255, 0), cv2.MARKER_CROSS, 12, 2)
                    cv2.putText(depth_vis, text, (x, y-6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

            combined = np.hstack((rgb, depth_vis))
            with self.lock:
                self.display_image = combined
        except Exception as e:
            self.get_logger().warn(f"compose_preview error: {e}")

    # ───────────────── GUI thread ────────────────────────────────────
    def gui_loop(self):
        cv2.namedWindow("RGB (left) | Depth (right)", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("RGB (left) | Depth (right)", 1280, 480)
        cv2.setMouseCallback("RGB (left) | Depth (right)", self.mouse_callback)

        while not self.gui_thread_stop.is_set():
            frame = None
            with self.lock:
                if self.display_image is not None:
                    frame = self.display_image.copy()
            if frame is not None:
                cv2.imshow("RGB (left) | Depth (right)", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.get_logger().info("GUI shutdown requested.")
                    self.gui_thread_stop.set()
                    rclpy.shutdown()
            else:
                cv2.waitKey(10)
        cv2.destroyAllWindows()

    # ───────────────── util ─────────────────────────────────────────
    def mouse_callback(self, event, x, y, *_):
        if event == cv2.EVENT_LBUTTONDOWN:
            with self.lock:
                self.clicked_point = (x, y)
            self.get_logger().info(f"Clicked pixel: ({x}, {y})")

# ──────────────────── main ──────────────────────────────────────────

def main():
    rclpy.init()
    node = DepthToMap()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.gui_thread_stop.set()
        node.gui_thread.join()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
