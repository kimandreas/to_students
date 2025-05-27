import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from cv_bridge import CvBridge
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import cv2
import time


class DepthToNavGoal(Node):
    def __init__(self):
        super().__init__('depth_to_nav_goal_node')

        self.bridge = CvBridge()
        self.K = None
        self.latest_map_point = None
        self.capture_enabled = True

        self.depth_topic = '/oakd/rgb/preview/depth'
        self.info_topic = '/oakd/rgb/preview/camera_info'

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, self.info_topic, self.camera_info_callback, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_timer(0.1, self.check_key)

        self.logged_intrinsics = False

        cv2.namedWindow("Control")
        cv2.imshow("Control", np.zeros((100, 300), dtype=np.uint8))

        self.last_feedback_log_time = 0


    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        if not self.logged_intrinsics:
            self.get_logger().info(f"Camera intrinsics received: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")
            self.logged_intrinsics = True

    def depth_callback(self, msg):
        if self.K is None or not self.capture_enabled:
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

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        camera_frame = msg.header.frame_id

        pt = PointStamped()
        pt.header.frame_id = camera_frame
        pt.header.stamp = msg.header.stamp
        pt.point.x = x
        pt.point.y = y
        pt.point.z = z

        try:
            # Update header with latest time to avoid extrapolation error
            pt_latest = PointStamped()
            pt_latest.header.frame_id = camera_frame
            pt_latest.header.stamp = rclpy.time.Time().to_msg()
            pt_latest.point = pt.point

            pt_map = self.tf_buffer.transform(pt_latest, 'map', timeout=rclpy.duration.Duration(seconds=0.5))
            self.latest_map_point = pt_map
            self.get_logger().info(f"map: ({pt_map.point.x:.2f}, {pt_map.point.y:.2f}, {pt_map.point.z:.2f})")
        except Exception as e:
            self.get_logger().warn(f"TF to map failed: {e}")

    def check_key(self):
        cv2.imshow("Control", np.zeros((100, 300), dtype=np.uint8))  # refresh dummy window
        key = cv2.waitKey(1) & 0xFF
        if key == ord('g'):
            self.get_logger().info("Key 'g' pressed: stopping capture and sending goal.")
            self.capture_enabled = False
            self.send_goal()

    def send_goal(self):
        if self.latest_map_point is None:
            self.get_logger().warn("No map coordinate available to send as goal.")
            return

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.latest_map_point.point.x
        pose.pose.position.y = self.latest_map_point.point.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0  # face forward

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(f"Sending goal to map position: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by Nav2.')
            return
        self.get_logger().info('Goal accepted. Waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Goal finished with result code: {future.result().status}")
        
        # Stop spinning and shutdown ROS
        self.get_logger().info("Shutting down after goal.")
        rclpy.shutdown()
        cv2.destroyAllWindows()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_time = time.time()
        if current_time - self.last_feedback_log_time > 1.0:
            self.get_logger().info(f"Distance remaining: {feedback.distance_remaining:.2f} m")
            self.last_feedback_log_time = current_time

def main():
    rclpy.init()
    node = DepthToNavGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # Don't shut down here, it's handled in callback
    node.destroy_node()


if __name__ == '__main__':
    main()
