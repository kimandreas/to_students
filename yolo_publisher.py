import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import time
import os
import json
import csv
import math
from shapely.geometry import Polygon

class YoloPublisher(Node):
    def __init__(self):
        super().__init__('yolo_publisher')
        self.publisher_ = self.create_publisher(Image, 'processed_image', 10)
        self.timer = self.create_timer(0.1, self.publish_image)
        
        self.bridge = CvBridge()
        self.model = YOLO('path_to_your_model.pt')  # Replace with your model path
        self.coordinates = []
        self.output_dir = './output'
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.cap = cv2.VideoCapture('/dev/video0')  # Adjust this to your camera source
        self.cap.set(3, 640)
        self.cap.set(4, 480)

    def publish_image(self):
        success, img = self.cap.read()
        if not success:
            return
        
        # Detect objects and draw bounding boxes
        results = self.model(img, stream=True)
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = math.ceil(box.conf[0] * 100) / 100
                cls = int(box.cls[0])
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(img, f"{cls}: {confidence}", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        
        # Convert and publish image
        ros_image = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.publisher_.publish(ros_image)
        
    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
