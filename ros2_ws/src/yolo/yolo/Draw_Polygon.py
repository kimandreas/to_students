import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

coordinates = []

def get_coordinates(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(coordinates) < 4:
            coordinates.append((x, y))
            print(f"Coordinate {len(coordinates)}: ({x}, {y})")
        if len(coordinates) == 4:
            print("Final coordinates collected:", coordinates)
            cv2.setMouseCallback('Camera', lambda *args: None)

class PolygonDrawer(Node):
    def __init__(self):
        super().__init__('polygon_drawer')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/robot0/oakd/rgb/preview/image_raw',
            self.image_callback,
            10
        )
        self.should_shutdown = False
        cv2.namedWindow('Camera')
        cv2.setMouseCallback('Camera', get_coordinates)

    def image_callback(self, msg):
        global coordinates

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        for (x, y) in coordinates:
            cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)

        if len(coordinates) >= 2:
            pts = np.array(coordinates, np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

        cv2.imshow('Camera', frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('r'):
            print("Resetting points...")
            coordinates = []
            cv2.setMouseCallback('Camera', get_coordinates)

        elif key == ord('q'):
            print("Shutting down...")
            self.should_shutdown = True

def main(args=None):
    rclpy.init(args=args)
    node = PolygonDrawer()

    try:
        while rclpy.ok() and not node.should_shutdown:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()
        print("Node shut down and windows closed.")

if __name__ == '__main__':
    main()
