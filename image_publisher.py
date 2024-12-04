import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_image)
        self.bridge = CvBridge()
        self.get_logger().info('Image Publisher Node has been started.')

    def publish_image(self):

        img = cv2.imread('/home/kimandreas/ros2_ws/src/image_transfer/image_transfer/output_1730357899.jpg')
        
        if img is not None:
            msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            self.publisher_.publish(msg)
            self.get_logger().info('Published image')
        else:
            self.get_logger().warn('Failed to load image')

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
