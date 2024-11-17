import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import threading
import sys
import select
import termios
import tty

class GoalCanceller(Node):
    def __init__(self):
        super().__init__('goal_canceller')
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def cancel_goal(self):
        self.get_logger().info('Waiting for the action server to become available...')
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available. Cannot cancel goals.')
            return

        self.get_logger().info('Attempting to cancel all active goals...')
        self._cancel_all_active_goals()

    def _cancel_all_active_goals(self):
        # Create a dummy goal request to send a cancel request for active goals
        cancel_future = self.action_client._cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if cancel_response.accepted:
            self.get_logger().info('Goal cancellation accepted. Exiting program...')
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)  # Exit the program after successful cancellation
        else:
            self.get_logger().info('Goal cancellation failed or no active goal to cancel.')

def keyboard_listener(node):
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while True:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key.lower() == 's':
                    node.get_logger().info('Key "s" pressed. Cancelling goal...')
                    node.cancel_goal()
                    break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main(args=None):
    rclpy.init(args=args)
    node = GoalCanceller()

    # Start the keyboard listener in a separate thread
    thread = threading.Thread(target=keyboard_listener, args=(node,), daemon=True)
    thread.start()

    rclpy.spin(node)

if __name__ == '__main__':
    main()
