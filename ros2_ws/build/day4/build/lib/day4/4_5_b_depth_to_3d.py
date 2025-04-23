import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import numpy as np
import tf2_ros
import tf2_geometry_msgs


class DepthToMap(Node):
    def __init__(self, topic_namespace):
        super().__init__('depth_to_map_node')
        self.topic_namespace = topic_namespace.strip('/')

        self.bridge = CvBridge()
        self.K = None

        self.depth_topic = f'/{self.topic_namespace}/oakd/stereo/image_raw'
        self.info_topic = f'/{self.topic_namespace}/oakd/stereo/camera_info'

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

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        camera_frame = msg.header.frame_id
        self.get_logger().info(f"camera_frame_id ({camera_frame})")
        self.get_logger().info(f"camera_frame: ({x:.2f}, {y:.2f}, {z:.2f})")

        pt = PointStamped()
        pt.header.frame_id = camera_frame
        pt.header.stamp = msg.header.stamp
        pt.point.x = x
        pt.point.y = y
        pt.point.z = z

        try:
            pt_base = self.tf_buffer.transform(pt, 'base_link', timeout=rclpy.duration.Duration(seconds=0.5))
            self.get_logger().info(f"base_link:    ({pt_base.point.x:.2f}, {pt_base.point.y:.2f}, {pt_base.point.z:.2f})")
        except Exception as e:
            self.get_logger().warn(f"TF to base_link failed: {e}")

        try:
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

    topic_namespace = input("Enter the robot namespace (e.g., /robot0): ").strip()
    if not topic_namespace.startswith('/'):
        topic_namespace = '/' + topic_namespace

    node = DepthToMap(topic_namespace)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
