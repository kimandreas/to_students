#!/usr/bin/env python3
"""
Depth‑click → continuous Nav2 goals (sync, crop, map‑frame)
===========================================================
Fixes & changes
---------------
1. **Multiple goals:** every new click cancels the current Nav2 task and sends
   a fresh `goToPose()` goal.
2. **Proper TF2 conversion:** keeps `tf2_geometry_msgs` import for PointStamped.
3. **Clean exit on ‘q’:** cancels any active Nav2 task, undocks, and shuts the
   executor without throwing.
4. Crop percentages remain *26 % width* and *18 % height*.
"""

import rclpy
from rclpy.node         import Node
from rclpy.executors    import MultiThreadedExecutor
from rclpy.duration     import Duration

from sensor_msgs.msg    import Image, CameraInfo, CompressedImage
from geometry_msgs.msg  import PointStamped, PoseStamped, Point
from cv_bridge          import CvBridge
from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Navigator, TurtleBot4Directions,
)
from tf2_ros            import Buffer, TransformListener
import tf2_geometry_msgs.tf2_geometry_msgs as _  # noqa: F401  (register types)
from message_filters    import Subscriber, ApproximateTimeSynchronizer

import numpy as np
import cv2, threading, time

from visualization_msgs.msg import Marker

CROP_W = 0.26   # 26 % total width → 13 % each side
CROP_H = 0.18   # 18 % total height →  9 % each edge

# ---------------------------------------------------------------------------
class DepthClickNav(Node):
    def __init__(self):
        super().__init__("depth_click_nav")

        ns = self.get_namespace()
        self.rgb_topic   = f"{ns}/oakd/rgb/image_raw/compressed"
        self.depth_topic = f"{ns}/oakd/stereo/image_raw"
        self.info_topic  = f"{ns}/oakd/rgb/camera_info"

        self.marker_pub = self.create_publisher(Marker, '/goal_marker', 10)

        self.bridge = CvBridge()
        self.K: np.ndarray | None = None
        self.camera_frame: str | None = None

        # TF2
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        time.sleep(2.0)  # give TF tree a moment at startup

        # Nav2 helper
        self.navigator = TurtleBot4Navigator()
        if not self.navigator.getDockedStatus():
            self.get_logger().info('Docking before initializing pose')
            self.navigator.dock()
        self.navigator.undock()

        # Synced subs
        rgb_sub   = Subscriber(self, CompressedImage, self.rgb_topic)
        depth_sub = Subscriber(self, Image,          self.depth_topic)

        self.ts = ApproximateTimeSynchronizer([rgb_sub, depth_sub], 30, 0.15)
        self.ts.registerCallback(self.cb_sync)

        self.create_subscription(CameraInfo, self.info_topic, self.cam_info_cb, 1)

        self.create_subscription(Point, 'detected_object_center', self.point_callback, 10)

        # UI state
        self.lock = threading.Lock()
        self.rgb_img:   np.ndarray | None = None
        self.depth_u16: np.ndarray | None = None  # cropped + resized
        self.display:   np.ndarray | None = None
        self.clicked:   tuple[int, int] | None = None

    # ---------------- Camera info -----------------------------------
    def cam_info_cb(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info("RGB intrinsics stored.")

    # ---------------- Sync callback ---------------------------------
    def cb_sync(self, rgb_msg: CompressedImage, depth_msg: Image):
        rgb   = cv2.imdecode(np.frombuffer(rgb_msg.data, np.uint8), cv2.IMREAD_COLOR)
        depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")  # uint16
        if rgb is None or depth is None:
            return

        h, w = depth.shape
        cx = int(CROP_W * w / 2) * 2
        cy = int(CROP_H * h / 2) * 2
        depth_crop = depth[cy: h-cy, cx: w-cx]
        depth_aligned = cv2.resize(depth_crop, (w, h), cv2.INTER_NEAREST)

        with self.lock:
            self.rgb_img   = rgb
            self.depth_u16 = depth_aligned
            self.camera_frame = depth_msg.header.frame_id

    # ---------------- Preview & goal logic --------------------------
    def point_callback(self, msg: Point):
        with self.lock:
            rgb   = None if self.rgb_img   is None else self.rgb_img.copy()
            depth = None if self.depth_u16 is None else self.depth_u16.copy()

        if rgb is None or depth is None:
            return

        if self.K is not None:
            u, v = int(msg.x), int(msg.y)
            z   = depth[v, u] / 1000.0  # m
            
            if 0.2 < z < 5.0:
                fx, fy, cx, cy = self.K[0,0], self.K[1,1], self.K[0,2], self.K[1,2]
                X = (u - cx) * z / fx
                Y = (v - cy) * z / fy
                pt_cam = PointStamped()
                pt_cam.header.stamp = self.get_clock().now().to_msg()
                pt_cam.header.frame_id = self.camera_frame
                pt_cam.point.x, pt_cam.point.y, pt_cam.point.z = X, Y, z
                try:
                    pt_map = self.tf_buffer.transform(pt_cam, "map", Duration(seconds=0.5))

                    try:
                        tf_now = self.get_clock().now().to_msg()
                        tf = self.tf_buffer.lookup_transform(
                            'map',                   # target frame
                            'base_link',             # source frame (robot body)
                            rclpy.time.Time(),       # latest available
                            Duration(seconds=0.5))   # timeout
                        q = tf.transform.rotation    # current quaternion in map frame
                    except Exception as e:
                        self.get_logger().warn(f"TF lookup failed: {e}")
                        return

                    # current = self.navigator.getCurrentPose()           # PoseStamped
                    goal = PoseStamped()
                    goal.header.frame_id = "map"
                    goal.header.stamp    = tf_now
                    goal.pose.position.x = pt_map.point.x
                    goal.pose.position.y = pt_map.point.y
                    goal.pose.orientation = q    # keep heading

                    # cancel any current task & send the new one
                    try:
                        self.navigator.cancelTask()
                    except Exception:
                        pass
                    self.navigator.goToPose(goal)
                    
                    self.get_logger().info(
                        f"New goal: ({pt_map.point.x:.2f}, {pt_map.point.y:.2f}) m")
                    
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = "goal_marker"
                    marker.id = 0
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    marker.pose.position = goal.pose.position
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = 0.1
                    marker.scale.y = 0.1
                    marker.scale.z = 0.1
                    marker.color.a = 1.0
                    marker.color.r = 1.0
                    marker.color.g = 0.2
                    marker.color.b = 0.2
                    self.marker_pub.publish(marker)

                except Exception as e:
                    self.get_logger().warn(f"TF transform failed: {e}")

# ---------------------------------------------------------------------------
def main():
    rclpy.init()
    node = DepthClickNav()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
