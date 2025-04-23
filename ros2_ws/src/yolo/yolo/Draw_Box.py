import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageBoxDrawer(Node):
    def __init__(self):
        super().__init__('image_box_drawer')
        self.subscription = self.create_subscription(
            Image,
            '/robot0/oakd/rgb/preview/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

        # Box properties
        self.x1, self.y1, self.x2, self.y2 = 100, 100, 200, 200
        self.color = (255, 0, 0)
        self.thickness = 2

        # Shutdown flag
        self.should_shutdown = False

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.rectangle(frame, (self.x1, self.y1), (self.x2, self.y2), self.color, self.thickness)
        cv2.imshow("image_raw", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("Shutdown requested via keypress.")
            self.should_shutdown = True

def main(args=None):
    rclpy.init(args=args)
    node = ImageBoxDrawer()

    try:
        while rclpy.ok() and not node.should_shutdown:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received.")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()
        print("Node and OpenCV windows closed cleanly.")

if __name__ == '__main__':
    main()
