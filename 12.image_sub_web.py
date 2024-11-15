import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from flask import Flask, Response, render_template
import threading
import io
import time

app = Flask(__name__, template_folder='/home/rokey/ros3_ws/src/image_transfer/image_transfer/templates')
latest_frame = None
lock = threading.Lock()

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'camera_image',
            self.image_callback,
            10
        )
        self.subscription  # Prevent unused variable warning
        self.bridge = CvBridge()

    def image_callback(self, msg):
        global latest_frame
        self.get_logger().info('Received image')
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Acquire lock to update the global frame
        with lock:
            _, buffer = cv2.imencode('.jpg', frame)
            latest_frame = buffer.tobytes()

def generate():
    while True:
        time.sleep(0.1)  # Adjust as needed for frame rate
        with lock:
            if latest_frame is not None:
                frame = latest_frame
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

def start_flask_app():
    # app.run()
    # app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    app.run(port=5000, debug=False, threaded=True)

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()

    # Start Flask app in a separate thread
    flask_thread = threading.Thread(target=start_flask_app)
    flask_thread.daemon = True
    flask_thread.start()

    try:
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
