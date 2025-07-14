import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration

from sensor_msgs.msg import Image as ROSImage, CameraInfo, CompressedImage
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener

from cv_bridge import CvBridge
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions

import numpy as np
import cv2
import threading

from rclpy.time import Time

from message_filters import Subscriber, ApproximateTimeSynchronizer


class DepthToMap(Node):
    def __init__(self):
        super().__init__('depth_to_map_node')

        self.bridge = CvBridge()
        self.K = None  # Camera intrinsics
        self.lock = threading.Lock()

        # Build topic names using namespace
        ns = self.get_namespace()
        self.depth_topic = f'{ns}/oakd/stereo/image_raw'
        self.rgb_topic = f'{ns}/oakd/rgb/image_raw/compressed'
        self.info_topic = f'{ns}/oakd/rgb/camera_info'

        self.depth_image = None
        self.rgb_image = None
        self.clicked_point = None

        # TF2 buffer for transforms (camera_frame -> base_link -> map)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Dock, set initial pose, undock
        self.navigator = TurtleBot4Navigator()
        if not self.navigator.getDockedStatus():
            self.get_logger().info('Docking before initializing pose')
            self.navigator.dock()

        initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        self.navigator.undock()

        self.logged_intrinsics = False
        self.logged_rgb_shape = False
        self.logged_depth_shape = False

        # Publishers for the processed images (for rqt_image_view)
        self.rgb_pub = self.create_publisher(ROSImage, f'{ns}/rgb_processed', 1)
        self.depth_pub = self.create_publisher(ROSImage, f'{ns}/depth_colored', 1)

        # Subscribe to camera intrinsics (needed once)
        self.create_subscription(CameraInfo, self.info_topic, self.camera_info_callback, 1)

        # Time-synchronized subscribers for RGB + Depth images
        self.rgb_sub = Subscriber(self, CompressedImage, self.rgb_topic)
        self.depth_sub = Subscriber(self, ROSImage, self.depth_topic)

        # ApproximateTimeSynchronizer: allows slight timestamp mismatch
        self.ts = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.synced_callback)

        # Thread for mouse click input
        self.gui_thread_stop = threading.Event()
        self.gui_thread = threading.Thread(target=self.gui_loop, daemon=True)
        self.gui_thread.start()

        # Delay to allow TF tree to stabilize
        self.get_logger().info("TF Tree stabilization starting. Will begin transforms in 5 sec.")
        self.start_timer = self.create_timer(5.0, self.start_transform)

    def start_transform(self):
        self.get_logger().info("TF Tree stabilized. Starting image processing & publishing.")
        # Loop timer to process images and publish overlays
        self.timer = self.create_timer(0.2, self.process_and_publish)
        self.start_timer.cancel()

    def camera_info_callback(self, msg):
        """ Store camera intrinsics (fx, fy, cx, cy) from CameraInfo """
        with self.lock:
            self.K = np.array(msg.k).reshape(3, 3)
            if not self.logged_intrinsics:
                self.get_logger().info(
                    f"Camera intrinsics: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, "
                    f"cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}"
                )
                self.logged_intrinsics = True

    def synced_callback(self, rgb_msg, depth_msg):
        """ Called when RGB and Depth frames arrive together (synchronized) """
        try:
            with self.lock:
                # Decode RGB image from compressed format
                np_arr = np.frombuffer(rgb_msg.data, np.uint8)
                rgb = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if rgb is not None and rgb.size > 0:
                    if not self.logged_rgb_shape:
                        self.get_logger().info(f"RGB image shape: {rgb.shape}")
                        self.logged_rgb_shape = True
                    self.rgb_image = rgb

                # Convert Depth image from ROS format to OpenCV
                depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
                if depth is not None and depth.size > 0:
                    if not self.logged_depth_shape:
                        self.get_logger().info(f"Depth image shape: {depth.shape}")
                        self.logged_depth_shape = True
                    self.depth_image = depth
                    self.camera_frame = depth_msg.header.frame_id
        except Exception as e:
            self.get_logger().error(f"Synced callback failed: {e}")

    def mouse_callback(self, event, x, y, flags, param):
        """ Store pixel coordinates when user clicks on RGB image """
        if event == cv2.EVENT_LBUTTONDOWN:
            with self.lock:
                self.clicked_point = (x, y)
            self.get_logger().info(f"Clicked pixel: ({x}, {y})")

    def gui_loop(self):
        """ Basic OpenCV window to accept mouse clicks for pixel selection """
        cv2.namedWindow('Click to select pixel', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Click to select pixel', 640, 480)
        cv2.setMouseCallback('Click to select pixel', self.mouse_callback)

        while not self.gui_thread_stop.is_set():
            img = None
            with self.lock:
                if self.rgb_image is not None:
                    img = self.rgb_image.copy()
            if img is not None:
                cv2.imshow('Click to select pixel', img)
                if cv2.waitKey(10) & 0xFF == ord('q'):
                    self.gui_thread_stop.set()
            else:
                cv2.waitKey(10)

    def process_and_publish(self):
        """ Process RGB & Depth, overlay info, transform clicked point, and publish """
        with self.lock:
            rgb = self.rgb_image.copy() if self.rgb_image is not None else None
            depth = self.depth_image.copy() if self.depth_image is not None else None
            click = self.clicked_point
            frame_id = getattr(self, 'camera_frame', None)

        if rgb is not None and depth is not None and frame_id:
            try:
                rgb_display = rgb.copy()
                depth_display = depth.copy()

                # Normalize and color-map the raw depth image for better visualization
                depth_normalized = cv2.normalize(depth_display, None, 0, 255, cv2.NORM_MINMAX)
                depth_colored = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_JET)

                if click:
                    # Calculate depth and transform to map frame for clicked pixel
                    x, y = click
                    if x < rgb_display.shape[1] and y < rgb_display.shape[0] \
                            and y < depth_display.shape[0] and x < depth_display.shape[1]:
                        z = float(depth_display[y, x]) / 1000.0  # mm -> meters
                        if 0.2 < z < 5.0:  # Valid range filter
                            fx, fy = self.K[0, 0], self.K[1, 1]
                            cx, cy = self.K[0, 2], self.K[1, 2]

                            # Back-project to camera coordinate system
                            X = (x - cx) * z / fx
                            Y = (y - cy) * z / fy
                            Z = z

                            # Create PointStamped for TF transform
                            pt_camera = PointStamped()
                            pt_camera.header.stamp = Time().to_msg()
                            pt_camera.header.frame_id = frame_id
                            pt_camera.point.x = X
                            pt_camera.point.y = Y
                            pt_camera.point.z = Z

                            # Transform to map frame
                            try:
                                pt_map = self.tf_buffer.transform(
                                    pt_camera,
                                    'map',
                                    timeout=Duration(seconds=1.0)
                                )
                                self.get_logger().info(
                                    f"Map coordinate: ({pt_map.point.x:.2f}, {pt_map.point.y:.2f}, {pt_map.point.z:.2f})"
                                )
                            except Exception as e:
                                self.get_logger().warn(f"TF transform failed: {e}")

                        # Draw overlay markers
                        text = f"{z:.2f} m" if 0.2 < z < 5.0 else "Invalid"
                        cv2.putText(rgb_display, '+', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                        cv2.circle(rgb_display, (x, y), 4, (0, 255, 0), -1)
                        cv2.putText(depth_colored, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                        cv2.circle(depth_colored, (x, y), 4, (255, 255, 255), -1)

                # Publish processed RGB image
                rgb_msg = self.bridge.cv2_to_imgmsg(rgb_display, encoding="bgr8")
                rgb_msg.header.stamp = self.get_clock().now().to_msg()
                self.rgb_pub.publish(rgb_msg)

                # Publish processed Depth image (color-mapped)
                depth_msg = self.bridge.cv2_to_imgmsg(depth_colored, encoding="bgr8")
                depth_msg.header.stamp = self.get_clock().now().to_msg()
                self.depth_pub.publish(depth_msg)

            except Exception as e:
                self.get_logger().warn(f"Image process/publish error: {e}")

    def destroy_node(self):
        """ Ensure GUI thread stops cleanly """
        self.gui_thread_stop.set()
        super().destroy_node()


def main():
    rclpy.init()
    node = DepthToMap()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.gui_thread_stop.set()
    node.gui_thread.join()
    node.destroy_node()


if __name__ == '__main__':
    main()

