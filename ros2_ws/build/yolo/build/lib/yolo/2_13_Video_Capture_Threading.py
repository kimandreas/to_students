import cv2
import threading
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Globals
shared_frame = None
threaded_frame = None
inline_frame = None
is_running = True
lock = threading.Lock()

threaded_count = 0
inline_count = 0

class SharedImageSubscriber(Node):
    def __init__(self):
        super().__init__('shared_image_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/robot0/oakd/rgb/preview/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        global shared_frame
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with lock:
            shared_frame = frame.copy()  # avoid race condition

def ros_thread_main():
    node = SharedImageSubscriber()
    try:
        while rclpy.ok() and is_running:
            rclpy.spin_once(node, timeout_sec=0.01)
    finally:
        node.destroy_node()

def main():
    global is_running, threaded_count, inline_count, threaded_frame, inline_frame

    rclpy.init()

    ros_thread = threading.Thread(target=ros_thread_main)
    ros_thread.start()

    last_time = time.time()

    try:
        while is_running:
            with lock:
                if shared_frame is not None:
                    threaded_frame = shared_frame.copy()
                    inline_frame = shared_frame.copy()

            if threaded_frame is not None:
                cv2.imshow("Threaded Display", threaded_frame)
                threaded_count += 1

            if inline_frame is not None:
                # simulate a different processing path if needed
                cv2.imshow("Inline Display", inline_frame)
                inline_count += 1

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                is_running = False

            now = time.time()
            if now - last_time >= 1.0:
                print(f"[FPS] Threaded: {threaded_count} | Inline: {inline_count}")
                threaded_count = 0
                inline_count = 0
                last_time = now

    finally:
        rclpy.shutdown()
        ros_thread.join()
        cv2.destroyAllWindows()
        print("Shutdown complete.")

if __name__ == '__main__':
    main()
