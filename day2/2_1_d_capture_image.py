import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageCaptureNode(Node):
    def __init__(self, save_directory, file_prefix):
        super().__init__('image_capture_node')
        self.subscription = self.create_subscription(
            Image,
            '/robot0/oakd/rgb/preview/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.frame = None
        self.save_directory = save_directory
        self.file_prefix = f"{file_prefix}_"
        self.image_count = 0
        os.makedirs(self.save_directory, exist_ok=True)

    def listener_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

def main():
    save_directory = input("Enter directory name to save images: ")
    file_prefix = input("Enter a file prefix to use: ")

    rclpy.init()
    node = ImageCaptureNode(save_directory, file_prefix)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            if node.frame is not None:
                cv2.imshow("Live Feed", node.frame)
                key = cv2.waitKey(1) & 0xFF

                if key == ord('c'):
                    file_name = os.path.join(
                        node.save_directory,
                        f"{node.file_prefix}img_{node.image_count}.jpg"
                    )
                    cv2.imwrite(file_name, node.frame)
                    print(f"Image saved: {file_name}")
                    node.image_count += 1

                elif key == ord('q'):
                    break

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

# This script captures images from the Oak-D camera and saves them to a specified directory.
# The user can specify the directory and file prefix for saved images.

# The script uses OpenCV to display the live feed and allows the user to capture images by pressing 'c'.
# Pressing 'q' will exit the program.
# The images are saved in the specified directory with the specified prefix and a sequential number.
