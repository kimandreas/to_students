import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YOLOTrackingPublisher(Node):
    def __init__(self):
        super().__init__('yolo_tracking_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, 'AMR_image', 10)
        self.bridge = CvBridge()
        self.model = YOLO('/home/rokey7/ros2_ws/src/amr_controller/amr_controller/best.pt')
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
            boxes = result.boxes  # Bounding boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Convert to integers
                track_id = int(box.id[0]) if box.id is not None else None  # Tracking ID
                conf = float(box.conf[0])  # Confidence score
                cls = int(box.cls[0])  # Class ID
                
                # Get class name (optional)
                class_name = self.model.names[cls] if cls in self.model.names else "Unknown"

                # Draw bounding box
                color = (0, 255, 0)  # Green
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

                # Display tracking ID, confidence, and class name
                label = f"ID: {track_id} {class_name} ({conf:.2f})"
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Compress the frame and convert it to a ROS 2 CompressedImage message
        _, compressed_frame = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        compressed_img_msg = CompressedImage()
        compressed_img_msg.header.stamp = self.get_clock().now().to_msg()
        compressed_img_msg.format = "jpeg"
        compressed_img_msg.data = compressed_frame.tobytes()

        # Publish the compressed image
        self.publisher_.publish(compressed_img_msg)

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
