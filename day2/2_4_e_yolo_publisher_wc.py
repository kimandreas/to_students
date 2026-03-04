import json
import csv
import time
import math
import os
import shutil
import sys
from ultralytics import YOLO
from pathlib import Path
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class YOLOWebcamPublisher(Node):
    def __init__(self, model, output_dir):
        super().__init__('yolo_webcam_publisher')
        self.model = model
        self.output_dir = output_dir
        self.csv_output = []
        self.confidences = []
        self.max_object_count = 0
        self.classNames = model.names
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, 'processed_image', 10)
        self.should_shutdown = False

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open webcam.")
            raise RuntimeError("Webcam not available")

        self.timer = self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        if self.should_shutdown:
            return

        ret, img = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame from webcam.")
            return

        results = self.model(img, stream=True)
        object_count = 0
        fontScale = 1

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)

                confidence = math.ceil((box.conf[0] * 100)) / 100
                cls = int(box.cls[0])
                label = self.classNames.get(cls, f"class_{cls}")
                self.confidences.append(confidence)

                org = [x1, y1]
                cv2.putText(img, f"{label}: {confidence}", org,
                            cv2.FONT_HERSHEY_SIMPLEX, fontScale, (255, 0, 0), 2)

                self.csv_output.append([x1, y1, x2, y2, confidence, label])
                object_count += 1

        self.max_object_count = max(self.max_object_count, object_count)
        cv2.putText(img, f"Objects_count: {object_count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, fontScale, (0, 255, 0), 1)

        if object_count > 0:
            filename = f'output_{int(time.time())}.jpg'
            cv2.imwrite(os.path.join(self.output_dir, filename), img)

        display_img = cv2.resize(img, (img.shape[1] * 2, img.shape[0] * 2))
        msg = self.bridge.cv2_to_imgmsg(display_img, encoding="bgr8")
        self.publisher.publish(msg)

    def save_output(self):
        with open(os.path.join(self.output_dir, 'output.csv'), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['X1', 'Y1', 'X2', 'Y2', 'Confidence', 'Class'])
            writer.writerows(self.csv_output)

        with open(os.path.join(self.output_dir, 'output.json'), 'w') as f:
            json.dump(self.csv_output, f)

        with open(os.path.join(self.output_dir, 'statistics.csv'), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Max Object Count', 'Average Confidence'])
            avg_conf = sum(self.confidences) / len(self.confidences) if self.confidences else 0
            writer.writerow([self.max_object_count, avg_conf])

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main():
    model_path = input("Enter path to model file (.pt, .engine, .onnx): ").strip()

    if not os.path.exists(model_path):
        print(f"❌ File not found: {model_path}")
        exit(1)

    suffix = Path(model_path).suffix.lower()
    if suffix == '.pt':
        model = YOLO(model_path)
    elif suffix in ['.onnx', '.engine']:
        model = YOLO(model_path, task='detect')
    else:
        print(f"❌ Unsupported model format: {suffix}")
        exit(1)

    output_dir = './output'
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.mkdir(output_dir)

    rclpy.init()
    node = YOLOWebcamPublisher(model, output_dir)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🔴 Ctrl+C received. Exiting...")
    finally:
        node.save_output()
        node.destroy_node()
        rclpy.shutdown()
        print("✅ Shutdown complete.")
        sys.exit(0)

if __name__ == '__main__':
    main()
