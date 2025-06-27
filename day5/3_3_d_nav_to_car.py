import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Quaternion
from tf2_geometry_msgs import do_transform_point
from visualization_msgs.msg import Marker

from tf2_ros import Buffer, TransformListener

from cv_bridge import CvBridge
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions

import numpy as np
import math
import threading


class DepthToGoal(Node):
    def __init__(self):
        super().__init__('depth_to_goal_node')

        self.bridge = CvBridge()
        self.K = None
        self.lock = threading.Lock()
        self.depth_image = None
        self.camera_frame = None
        self.tf_ready = False

        ns = self.get_namespace()
        self.depth_topic = f'{ns}/oakd/stereo/image_raw'
        self.info_topic = f'{ns}/oakd/rgb/camera_info'

        self.create_subscription(CameraInfo, self.info_topic, self.camera_info_callback, 1)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 1)
        self.create_subscription(Point, 'detected_object_center', self.point_callback, 10)

        self.marker_pub = self.create_publisher(Marker, '/goal_marker', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.navigator = TurtleBot4Navigator()
        if not self.navigator.getDockedStatus():
            self.get_logger().info('Docking before initializing pose')
            self.navigator.dock()

        initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        self.navigator.undock()

        self.get_logger().info("Waiting 5 seconds for TF tree to stabilize...")
        self.create_timer(5.0, self.enable_tf)

    def enable_tf(self):
        self.tf_ready = True
        self.get_logger().info("TF tree ready. Processing center points.")

    def camera_info_callback(self, msg):
        with self.lock:
            self.K = np.array(msg.k).reshape(3, 3)
            self.camera_frame = msg.header.frame_id

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            with self.lock:
                self.depth_image = depth
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

    def point_callback(self, msg: Point):
        if not self.tf_ready:
            self.get_logger().info("TF not ready yet. Skipping point.")
            return

        with self.lock:
            x, y = int(msg.x), int(msg.y)
            depth = self.depth_image.copy() if self.depth_image is not None else None
            K = self.K.copy() if self.K is not None else None
            frame_id = self.camera_frame

        if depth is None or K is None:
            self.get_logger().warn("Missing depth or intrinsics.")
            return

        if not (0 <= x < depth.shape[1] and 0 <= y < depth.shape[0]):
            self.get_logger().warn(f"Invalid point: ({x}, {y}) outside image.")
            return

        z = float(depth[y, x]) / 1000.0
        if not (0.2 < z < 5.0):
            self.get_logger().info(f"Depth {z:.2f}m out of range at point ({x},{y}).")
            return

        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        X = (x - cx) * z / fx
        Y = (y - cy) * z / fy
        Z = z

        pt_camera = PointStamped()
        pt_camera.header.stamp = self.get_clock().now().to_msg()
        pt_camera.header.frame_id = frame_id
        pt_camera.point.x = X
        pt_camera.point.y = Y
        pt_camera.point.z = Z

        if not self.tf_buffer.can_transform('map', frame_id, Time(), timeout=Duration(seconds=1.0)):
            self.get_logger().warn(f"TF not available from {frame_id} to map.")
            return

        try:
            # pt_map = self.tf_buffer.transform(pt_camera, 'map', timeout=Duration(seconds=1.0))

            transform = self.tf_buffer.lookup_transform(
                'map',
                frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            pt_map = do_transform_point(pt_camera, transform)

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = pt_map.point.x
            goal_pose.pose.position.y = pt_map.point.y
            goal_pose.pose.position.z = 0.0
            yaw = 0.0
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            goal_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)

            self.navigator.goToPose(goal_pose)
            self.get_logger().info(f"Sent navigation goal: ({pt_map.point.x:.2f}, {pt_map.point.y:.2f})")

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "goal_marker"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = goal_pose.pose.position
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.2
            marker.color.b = 0.2
            self.marker_pub.publish(marker)
        except Exception as e:
            self.get_logger().warn(f"TF transform error: {e}")


def main():
    rclpy.init()
    node = DepthToGoal()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == '__main__':
    main()
