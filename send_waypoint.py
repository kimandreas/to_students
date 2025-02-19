import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import FollowWaypoints
import math
import threading
import sys
import select
import termios
import tty
import time
from action_msgs.msg import GoalStatus


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self._goal_handle = None

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def send_goal(self):
        waypoints = []
        
        # Define three waypoints
        positions = [
            (0.35624730587005615, -0.7531262636184692),
            (-1.0062505006790161, -0.15937140583992004),
            (-1.443751335144043, -0.3468696177005768)
        ]

        for x, y in positions:
            waypoint = PoseStamped()
            waypoint.header.stamp.sec = 0
            waypoint.header.stamp.nanosec = 0
            waypoint.header.frame_id = "map"
            waypoint.pose.position.x = x
            waypoint.pose.position.y = y
            waypoint.pose.position.z = 0.0
            waypoint.pose.orientation = self.euler_to_quaternion(0, 0, 0.0)  
            waypoints.append(waypoint)

        # Send goal
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        self.action_client.wait_for_server()
        self.get_logger().info('Sending waypoints...')
        
        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')
        self._goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current Waypoint Index: {feedback.current_waypoint}')

    def cancel_goal(self):
        if self._goal_handle is not None:
            self.get_logger().info('Attempting to cancel the goal...')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.get_logger().info('No active goal to cancel.')

    # def cancel_done_callback(self, future):
    #     cancel_response = future.result()
    #     if len(cancel_response.goals_cancelled) > 0:
    #         self.get_logger().info('Goal cancellation accepted. Exiting program...')
    #         self.destroy_node()
    #         rclpy.shutdown()
    #         sys.exit(0)  # Exit the program after successful cancellation
    #     else:
    #         self.get_logger().info('Goal cancellation failed or no active goal to cancel.')
    
    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info(f'Goal cancellation in progress: {len(cancel_response.goals_canceling)} goal(s) canceling...')
            self.wait_for_cancellation()
        else:
            self.get_logger().info('Goal cancellation failed or no active goal to cancel.')

    def wait_for_cancellation(self):
        """ Polls the goal state until it reaches STATUS_CANCELED """
        if self._goal_handle is None:
            self.get_logger().info('No active goal to check for cancellation.')
            return

        while True:
            result_future = self._goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            result = result_future.result()
            if result is not None and result.status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info('Goal successfully canceled. Exiting program...')
                self.destroy_node()
                rclpy.shutdown()
                sys.exit(0)
            else:
                self.get_logger().info('Waiting for goal to be fully canceled...')
                time.sleep(0.1)

    def get_result_callback(self, future):
        result = future.result().result
        missed_waypoints = result.missed_waypoints
        if missed_waypoints:
            self.get_logger().info(f'Missed waypoints: {missed_waypoints}')
        else:
            self.get_logger().info('All waypoints completed successfully!')

def keyboard_listener(node):
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while True:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key.lower() == 'g':
                    node.get_logger().info('Key "g" pressed. Sending goal...')
                    node.send_goal()
                elif key.lower() == 's':
                    node.get_logger().info('Key "s" pressed. Cancelling goal...')
                    node.cancel_goal()
                    break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    
    thread = threading.Thread(target=keyboard_listener, args=(node,), daemon=True)
    thread.start()
    
    rclpy.spin(node)


if __name__ == '__main__':
    main()
