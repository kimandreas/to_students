import time
import math
import os
import sys
import rclpy
import threading
from queue import Queue
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from pathlib import Path
import cv2

class YOLOImageSubscriber(Node):
    def __init__(self, model):
        super().__init__('yolo_image_subscriber')
        self.model = model
        self.bridge = CvBridge()
        self.image_queue = Queue(maxsize=1)
        self.should_shutdown = False
        self.classNames = model.names if hasattr(model, 'names') else ['Object']

        self.subscription = self.create_subscription(
            Image,
            '/robot5/oakd/rgb/preview/image_raw',
            self.listener_callback,
            10)

        self.thread = threading.Thread(target=self.detection_loop, daemon=True)
        self.thread.start()

    def listener_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if not self.image_queue.full():
                self.image_queue.put(img)
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

    def detection_loop(self):
        while not self.should_shutdown:
            try:
                img = self.image_queue.get(timeout=0.5)
            except:
                continue

            results = self.model.predict(img, stream=True)

            for r in results:
                if not hasattr(r, 'boxes') or r.boxes is None:
                    continue
                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls = int(box.cls[0]) if box.cls is not None else 0
                    conf = float(box.conf[0]) if box.conf is not None else 0.0

                    label = f"{self.classNames[cls]} {conf:.2f}"
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow("YOLOv8 Detection", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("Shutdown requested via 'q'")
                self.should_shutdown = True
                break

def main():
    model_path = input("Enter path to model file (.pt): ").strip()

    if not os.path.exists(model_path):
        print(f"File not found: {model_path}")
        sys.exit(1)

    suffix = Path(model_path).suffix.lower()
    if suffix == '.pt':
        model = YOLO(model_path)
    elif suffix in ['.onnx', '.engine']:
        model = YOLO(model_path, task='detect')
    else:
        print(f"Unsupported model format: {suffix}")
        sys.exit(1)

    rclpy.init()
    node = YOLOImageSubscriber(model)

    try:
        while rclpy.ok() and not node.should_shutdown:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested via Ctrl+C.")
    finally:
        node.should_shutdown = True
        node.thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        print("Shutdown complete.")

if __name__ == '__main__':
    main()
