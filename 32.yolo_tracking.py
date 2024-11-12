import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YOLOTrackingPublisher(Node):
    def __init__(self):
        super().__init__('yolo_tracking_publisher')
        self.publisher_ = self.create_publisher(Image, 'tracked_image', 10)
        self.bridge = CvBridge()
        self.model = YOLO('/home/rokey/ros2_ws/src/amr_controller/amr_controller/best.pt')
        self.cap = cv2.VideoCapture(0)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Adjust timer frequency as needed

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame from webcam.")
            return

        # Run tracking and get results
        results = self.model.track(source=frame, show=False, tracker='bytetrack.yaml')

        # Iterate over results to extract data
        for result in results:
            for detection in result.boxes.data:
                if len(detection) >= 6:
                    x1, y1, x2, y2, confidence, class_id = detection[:6]
                    track_id = detection[6] if len(detection) > 6 else None
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)

                    # Draw bounding box and center point on the frame
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                    label_text = f'Conf: {confidence:.2f} Class: {int(class_id)}'
                    if track_id is not None:
                        label_text = f'Track_ID: {int(track_id)}, ' + label_text

                    cv2.putText(frame, label_text, (int(x1), int(y1) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Convert the frame to a ROS 2 Image message and publish
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

    def destroy_node(self):
        super().destroy_node()
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = YOLOTrackingPublisher()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
