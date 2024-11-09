import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
import threading
import sys
import select
import termios
import tty

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def publish_initial_pose(self):
        # Create the initial pose message
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'  # The frame in which the pose is defined
        initial_pose.header.stamp = self.get_clock().now().to_msg()

        # /initialpose

        # position:
        #   x: 0.1750425100326538
        #   y: 0.05808566138148308
        #   z: 0.0
        # orientation:
        #   x: 0.0
        #   y: 0.0
        #   z: -0.04688065682721989
        #   w: 0.9989004975549108
  


        # Set the position (adjust these values as needed)
        initial_pose.pose.pose.position.x = 0.1750425100326538 # X-coordinate
        initial_pose.pose.pose.position.y = 0.05808566138148308 # Y-coordinate
        initial_pose.pose.pose.position.z = 0.0  # Z should be 0 for 2D navigation

        # Set the orientation (in quaternion form)
        initial_pose.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=-0.04688065682721989,  # 90-degree rotation in yaw (example)
            w=0.9989004975549108  # Corresponding quaternion w component
        )

        # covariance:
        #   - 0.25
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.25
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.06853891909122467

        # Set the covariance values for the pose estimation
        initial_pose.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467
        ]


        # Publish the initial pose
        self.publisher.publish(initial_pose)
        self.get_logger().info('Initial pose published.')

        # Destroy the node and shutdown rclpy
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

def keyboard_listener(node):
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while True:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key.lower() == 'i':
                    node.get_logger().info('Key "i" pressed. Publishing initial pose...')
                    node.publish_initial_pose()
                    break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()

    # Start the keyboard listener in a separate thread
    thread = threading.Thread(target=keyboard_listener, args=(node,), daemon=True)
    thread.start()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
