import json
import csv
import time
import math
import os
import shutil
import sys
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from pathlib import Path
import cv2

class YOLOImageProcessor(Node):
    def __init__(self, model, output_dir):
        super().__init__('yolo_image_processor')
        self.model = model
        self.output_dir = output_dir
        self.bridge = CvBridge()
        self.classNames = ['Car']

        self.csv_output = []
        self.confidences = []
        self.max_object_count = 0

        self.latest_frame = None
        self.lock = threading.Lock()
        self.should_shutdown = False

        self.subscription = self.create_subscription(
            Image,
            '/robot0/oakd/rgb/preview/image_raw',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.latest_frame = img
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")

    def inference_loop(self):
        while rclpy.ok() and not self.should_shutdown:
            with self.lock:
                frame = self.latest_frame.copy() if self.latest_frame is not None else None

            if frame is None:
                rclpy.spin_once(self, timeout_sec=0.05)
                continue

            results = self.model(frame, stream=True)
            object_count = 0
            fontScale = 1

            for r in results:
                boxes = r.boxes
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

                    confidence = math.ceil((box.conf[0]*100))/100
                    cls = int(box.cls[0])
                    self.confidences.append(confidence)

                    org = [x1, y1]
                    cv2.putText(frame, f"{self.classNames[cls]}: {confidence}", org, cv2.FONT_HERSHEY_SIMPLEX, fontScale, (255, 0, 0), 2)

                    self.csv_output.append([x1, y1, x2, y2, confidence, cls])
                    object_count += 1

            self.max_object_count = max(self.max_object_count, object_count)
            cv2.putText(frame, f"Objects_count: {object_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, fontScale, (0, 255, 0), 1)

            if object_count > 0:
                cv2.imwrite(os.path.join(self.output_dir, f'output_{int(time.time())}.jpg'), frame)

            display_img = cv2.resize(frame, (frame.shape[1]*2, frame.shape[0]*2))
            cv2.imshow("Detection", display_img)

            key = cv2.waitKey(1)
            if key == ord('q'):
                self.get_logger().info("Shutting down...")
                self.should_shutdown = True
                break

    def save_output(self):
        with open(os.path.join(self.output_dir, 'output.csv'), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(self.csv_output)

        with open(os.path.join(self.output_dir, 'output.json'), 'w') as f:
            json.dump(self.csv_output, f)

        with open(os.path.join(self.output_dir, 'statistics.csv'), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Max Object Count', 'Average Confidence'])
            writer.writerow([self.max_object_count, sum(self.confidences)/len(self.confidences) if self.confidences else 0])


def main():
    model_path = input("Enter path to model file (.pt, .engine, .onnx): ").strip()

    if not os.path.exists(model_path):
        print(f"File not found: {model_path}")
        exit(1)

    suffix = Path(model_path).suffix.lower()
    if suffix == '.pt':
        model = YOLO(model_path)
    elif suffix in ['.onnx', '.engine']:
        model = YOLO(model_path, task='detect')
    else:
        print(f"Unsupported model format: {suffix}")
        exit(1)

    output_dir = './output'
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.mkdir(output_dir)

    rclpy.init()
    node = YOLOImageProcessor(model, output_dir)

    try:
        # Run inference loop in main thread, ROS2 spin in background
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()

        node.inference_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.save_output()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        print("Shutdown complete.")
        sys.exit(0)

if __name__ == '__main__':
    main()
