import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import numpy as np
import math
import cv2

class DepthClickToMap(Node):
    def __init__(self, namespace):
        super().__init__('depth_click_to_map')
        self.bridge = CvBridge()
        self.namespace = namespace

        self.depth_topic = f'/{namespace}/oakd/stereo/image_raw'
        self.info_topic = f'/{namespace}/oakd/stereo/camera_info'
        self.pose_topic = f'/{namespace}/amcl_pose'

        self.camera_info = None
        self.robot_pose = None
        self.cv_image = None

        self.subscription_img = self.create_subscription(Image,
            self.depth_topic, self.image_callback, 10)
        self.subscription_info = self.create_subscription(CameraInfo,
            self.info_topic, self.info_callback, 10)
        self.subscription_pose = self.create_subscription(PoseWithCovarianceStamped,
            self.pose_topic, self.pose_callback, 10)

        # Start OpenCV window
        cv2.namedWindow("Click on Depth")
        cv2.setMouseCallback("Click on Depth", self.mouse_callback)

        self.get_logger().info("Click on the image to select target points. Press 'q' to quit.")

    def info_callback(self, msg):
        self.camera_info = msg

    def pose_callback(self, msg):
        pose = msg.pose.pose
        self.robot_pose = (
            pose.position.x,
            pose.position.y,
            self.get_yaw_from_quaternion(pose.orientation)
        )

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed: {e}")

    def mouse_callback(self, event, u, v, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self.cv_image is None or self.camera_info is None or self.robot_pose is None:
            self.get_logger().info("Waiting for data...")
            return

        if v >= self.cv_image.shape[0] or u >= self.cv_image.shape[1]:
            self.get_logger().error("Pixel coordinate out of bounds.")
            return

        Z = float(self.cv_image[v, u]) / 1000.0
        if Z == 0:
            self.get_logger().warn("Depth at selected pixel is zero.")
            return

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        x_cam = (u - cx) * Z / fx
        y_cam = (v - cy) * Z / fy
        z_cam = Z

        cam_offset = [0.1, 0.0, 0.2]
        x_base = cam_offset[0] + x_cam
        y_base = cam_offset[1] + y_cam
        z_base = cam_offset[2] + z_cam

        x_map, y_map, theta = self.robot_pose
        x_obj = x_map + math.cos(theta) * x_base - math.sin(theta) * y_base
        y_obj = y_map + math.sin(theta) * x_base + math.cos(theta) * y_base

        self.get_logger().info(f"\nClicked pixel: (u={u}, v={v})")
        self.get_logger().info(f"Base_link: x={x_base:.2f}, y={y_base:.2f}, z={z_base:.2f}")
        self.get_logger().info(f"Map: x={x_obj:.2f}, y={y_obj:.2f}")

    def spin_image(self):
        rate = self.create_rate(10)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.cv_image is not None:
                img_vis = cv2.normalize(self.cv_image, None, 0, 255, cv2.NORM_MINMAX)
                img_vis = img_vis.astype(np.uint8)
                img_vis = cv2.cvtColor(img_vis, cv2.COLOR_GRAY2BGR)
                cv2.imshow("Click on Depth", img_vis)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

        cv2.destroyAllWindows()

def main():
    rclpy.init()
    namespace = input("Enter robot namespace (e.g. robot0): ").strip()
    node = DepthClickToMap(namespace)
    node.spin_image()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
