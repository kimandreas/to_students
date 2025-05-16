import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator,TurtleBot4Directions
import numpy as np
import cv2
import tf2_ros
import tf2_geometry_msgs

class DepthToMap(Node):
    def __init__(self):
        super().__init__('depth_to_map_node')

        self.bridge = CvBridge()
        self.K = None

        ns = self.get_namespace()
        self.depth_topic = f'{ns}/oakd/stereo/image_raw'
        self.rgb_topic = f'{ns}/oakd/rgb/image_raw/compressed'
        self.info_topic = f'{ns}/oakd/rgb/camera_info'

        self.depth_image = None
        self.rgb_image = None
        self.clicked_point = None
        self.depth_header = None
        self.shutdown_requested = False

        self.navigator = TurtleBot4Navigator()
        
        # Dock → Set Pose → Undock
        if not self.navigator.getDockedStatus():
            self.get_logger().info('Docking before initializing pose')
            self.navigator.dock()

        initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        self.navigator.undock()

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.logged_intrinsics = False

        self.create_subscription(CameraInfo, self.info_topic, self.camera_info_callback, 1)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 1)
        self.create_subscription(CompressedImage, self.rgb_topic, self.rgb_callback, 1)

        # ★ 5초 후에 변환 시작 타이머 설정
        self.get_logger().info("TF Tree 안정화 시작. 5초 후 변환 시작합니다.")
        self.start_timer = self.create_timer(5.0, self.start_transform)

    def start_transform(self):
        # 첫 변환 시도
        self.get_logger().info("TF Tree 안정화 완료. 변환 시작합니다.")

        self.timer = self.create_timer(1.0, self.display_images)
        cv2.namedWindow('RGB (left) | Depth (right)')
        cv2.setMouseCallback('RGB (left) | Depth (right)', self.mouse_callback)

        self.start_timer.cancel()

    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        if not self.logged_intrinsics:
            self.get_logger().info(f"Camera intrinsics received: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")
            self.logged_intrinsics = True

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_header = msg.header
        except Exception as e:
            self.get_logger().error(f"Depth CV bridge conversion failed: {e}")

    def rgb_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.rgb_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"Compressed RGB decode failed: {e}")

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.clicked_point = (x, y)
            self.get_logger().info(f"Clicked RGB pixel: ({x}, {y})")

    def display_images(self):
        if self.rgb_image is not None and self.depth_image is not None:
            try:
                rgb_display = self.rgb_image.copy()
                depth_display = self.depth_image.copy()

                depth_normalized = cv2.normalize(depth_display, None, 0, 255, cv2.NORM_MINMAX)
                depth_colored = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_JET)

                if self.clicked_point and self.K is not None and self.depth_header:
                    x, y = self.clicked_point
                    if x < rgb_display.shape[1] and y < rgb_display.shape[0]:
                        z = float(depth_display[y, x]) / 1000.0
                        if z > 0:
                            fx, fy = self.K[0, 0], self.K[1, 1]
                            cx, cy = self.K[0, 2], self.K[1, 2]

                            X = (x - cx) * z / fx
                            Y = (y - cy) * z / fy

                            pt = PointStamped()
                            pt.header.frame_id = self.depth_header.frame_id
                            pt.header.stamp = self.depth_header.stamp
                            pt.point.x = X
                            pt.point.y = Y
                            pt.point.z = z

                            try:
                                pt_map = self.tf_buffer.transform(pt, 'map', timeout=rclpy.duration.Duration(seconds=0.5))
                                self.get_logger().info(f"map: ({pt_map.point.x:.2f}, {pt_map.point.y:.2f}, {pt_map.point.z:.2f})")
                            except Exception as e:
                                self.get_logger().warn(f"TF transform to map failed: {e}")

                        text = f"{z:.2f} m" if z > 0 else "Invalid"
                        cv2.putText(rgb_display, '+', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                        cv2.circle(rgb_display, (x, y), 4, (0, 255, 0), -1)
                        cv2.putText(depth_colored, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                        cv2.circle(depth_colored, (x, y), 4, (255, 255, 255), -1)

                combined = np.hstack((rgb_display, depth_colored))
                cv2.imshow('RGB (left) | Depth (right)', combined)
                key = cv2.waitKey(1)
                if key == ord('q'):
                    self.get_logger().info("Shutdown requested by user.")
                    self.navigator.dock()
                    self.shutdown_requested = True
            except Exception as e:
                self.get_logger().warn(f"Image display error: {e}")


def main():
    rclpy.init()
    node = DepthToMap()
    try:
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
