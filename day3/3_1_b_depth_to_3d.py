import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import numpy as np
import tf2_ros
import tf2_geometry_msgs


class DepthToMap(Node):
    def __init__(self):
        super().__init__('depth_to_map_node')

        self.bridge = CvBridge()
        self.K = None
        self.depth_topic = '/oakd/rgb/preview/depth'
        self.info_topic = '/oakd/rgb/preview/camera_info'

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, self.info_topic, self.camera_info_callback, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)

        self.logged_intrinsics = False

    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        if not self.logged_intrinsics:
            self.get_logger().info(f"Camera intrinsics received: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")
            self.logged_intrinsics = True

    def depth_callback(self, msg):
        if self.K is None:
            self.get_logger().warn('Waiting for camera intrinsics...')
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"CV bridge conversion failed: {e}")
            return

        # height, width = depth_image.shape
        cx = self.K[0, 2]
        cy = self.K[1, 2]
        fx = self.K[0, 0]
        fy = self.K[1, 1]

        u = int(cx)
        v = int(cy)

        z = float(depth_image[v, u])
        if z == 0.0:
            self.get_logger().warn('Invalid depth at center pixel')
            return

        # Compute 3D point in camera frame
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        # Auto-detect the camera frame from the depth image header
        camera_frame = msg.header.frame_id
        self.get_logger().info(f"camera_frame_id ({camera_frame})")
        self.get_logger().info(f"camera_frame: ({x:.2f}, {y:.2f}, {z:.2f})")

        # Prepare PointStamped in camera frame
        pt = PointStamped()
        pt.header.frame_id = camera_frame
        pt.header.stamp = msg.header.stamp  # for base_link we can use this timestamp
        pt.point.x = x
        pt.point.y = y
        pt.point.z = z

        # Transform to base_link (use exact timestamp)
        try:
            pt_base = self.tf_buffer.transform(pt, 'base_link', timeout=rclpy.duration.Duration(seconds=0.5))
            self.get_logger().info(f"base_link:    ({pt_base.point.x:.2f}, {pt_base.point.y:.2f}, {pt_base.point.z:.2f})")
        except Exception as e:
            self.get_logger().warn(f"TF to base_link failed: {e}")

        # Transform to map (avoid extrapolation by using latest available time)
        try:
            # Update header with latest time to avoid extrapolation error
            pt_latest = PointStamped()
            pt_latest.header.frame_id = camera_frame
            pt_latest.header.stamp = rclpy.time.Time().to_msg()
            pt_latest.point = pt.point

            pt_map = self.tf_buffer.transform(pt_latest, 'map', timeout=rclpy.duration.Duration(seconds=0.5))
            self.get_logger().info(f"map:          ({pt_map.point.x:.2f}, {pt_map.point.y:.2f}, {pt_map.point.z:.2f})")
        except Exception as e:
            self.get_logger().warn(f"TF to map failed: {e}")


def main():
    rclpy.init()
    node = DepthToMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
# This script converts depth images to 3D point clouds and transforms them to the map frame.
# It uses the camera intrinsics and depth information to compute the 3D coordinates.
# The script also handles TF transformations to convert the points from the camera frame to the base_link and map frames.