import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YOLOTrackingPublisher(Node):
    def __init__(self):
        super().__init__('yolo_tracking_publisher')
        self.image_publisher = self.create_publisher(Image, 'tracked_image', 10)
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.model = YOLO('/home/rokey/ros2_ws/src/amr_controller/amr_controller/best.pt')
        self.cap = cv2.VideoCapture(0)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Adjust timer frequency as needed

        # Get screen dimensions and calculate 80% threshold for bounding box area
        self.screen_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.screen_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.screen_area = self.screen_width * self.screen_height
        self.target_area_threshold = 0.8 * self.screen_area

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame from webcam.")
            return

        # Run tracking and get results
        results = self.model.track(source=frame, show=False, tracker='bytetrack.yaml')

        highest_confidence_detection = None
        highest_confidence = 0.0

        # Iterate over results to find the object with the highest confidence
        for result in results:
            for detection in result.boxes.data:
                if len(detection) >= 6:
                    x1, y1, x2, y2, confidence, class_id = detection[:6]
                    if confidence > highest_confidence:
                        highest_confidence = confidence
                        highest_confidence_detection = (x1, y1, x2, y2, confidence, class_id)

        # If a detection is found, draw it and control the robot
        if highest_confidence_detection:
            x1, y1, x2, y2, confidence, class_id = highest_confidence_detection
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)
            box_width = x2 - x1
            box_height = y2 - y1
            box_area = box_width * box_height

            # Draw the bounding box and center point on the frame
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
            label_text = f'Conf: {confidence:.2f} Class: {int(class_id)}'
            cv2.putText(frame, label_text, (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Publish movement commands to align and move backward if the box area is too large
            twist = Twist()

            # Align the robot with the center of the screen
            screen_center_x = int(self.screen_width / 2)
            alignment_tolerance = 20  # Pixel tolerance for alignment

            if abs(center_x - screen_center_x) > alignment_tolerance:
                if center_x < screen_center_x:
                    twist.angular.z = 0.2  # Rotate left
                else:
                    twist.angular.z = -0.2  # Rotate right
            else:
                twist.angular.z = 0.0  # Stop rotation when aligned

            # Move backward if the object's bounding box area is greater than or equal to 80% of the screen area
            if box_area >= self.target_area_threshold:
                twist.linear.x = -0.1  # Move backward slowly
            else:
                twist.linear.x = 0.0  # Stop moving if the object is not too close

            self.cmd_publisher.publish(twist)

        # Convert the frame to a ROS 2 Image message and publish
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_publisher.publish(msg)

    def destroy_node(self):
        super().destroy_node()
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = YOLOTrackingPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        if rclpy.ok():
            node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
