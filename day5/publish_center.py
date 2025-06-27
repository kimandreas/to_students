import time
import math
import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2


class YOLOTracker(Node):
    def __init__(self, model):
        super().__init__('yolo_tracker')
        self.model = model
        self.bridge = CvBridge()

        ns = self.get_namespace().rstrip('/')
        self.rgb_topic = f'{ns}/oakd/rgb/image_raw/compressed' if ns else '/oakd/rgb/image_raw/compressed'

        self.subscription = self.create_subscription(
            CompressedImage,
            self.rgb_topic,
            self.listener_callback,
            10)

        self.center_pub = self.create_publisher(Point, 'detected_object_center', 10)

        self.classNames = self.model.names if hasattr(self.model, 'names') else ['Object']
        self.should_shutdown = False

        self.track_this_id = -1  # Track only the first detected object's ID

    def listener_callback(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model.track(source=img, stream=True, persist=True, conf=0.8, verbose=False)

        # ✅ Assign track ID only once for the lifetime of the node
        if self.track_this_id == -1:
            for r in results:
                if not hasattr(r, 'boxes') or r.boxes is None:
                    continue
                for box in r.boxes:
                    if box.id is not None and int(box.id[0]) >= 0:
                        self.track_this_id = int(box.id[0])
                        self.get_logger().info(f"Tracking object with ID: {self.track_this_id}")
                        break
                if self.track_this_id != -1:
                    break

        for r in results:
            if not hasattr(r, 'boxes') or r.boxes is None:
                continue

            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls = int(box.cls[0]) if box.cls is not None else 0
                conf = float(box.conf[0]) if box.conf is not None else 0.0
                class_name = self.classNames[cls] if cls < len(self.classNames) else "Unknown"
                track_id = int(box.id[0]) if box.id is not None else -1

                if track_id < 0:
                    continue

                if track_id == self.track_this_id:
                    # Calculate center point
                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0

                    point_msg = Point()
                    point_msg.x = float(cx)
                    point_msg.y = float(cy)
                    point_msg.z = 0.0
                    self.center_pub.publish(point_msg)
                    self.get_logger().info(f"Detected {class_name} {conf:.2f} ID:{track_id} at center ({int(cx)}, {int(cy)})")

                label = f"{class_name} {conf:.2f} ID:{track_id}"
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("YOLOv8 Tracking", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Shutdown requested by 'q'.")
            self.should_shutdown = True


def main():
    model_path = input("Enter path to YOLOv8 model (.pt): ").strip()

    if not os.path.exists(model_path):
        print(f"File not found: {model_path}")
        sys.exit(1)

    model = YOLO(model_path)

    rclpy.init()
    node = YOLOTracker(model)

    try:
        while rclpy.ok() and not node.should_shutdown:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        print("Shutdown complete.")


if __name__ == '__main__':
    main()
