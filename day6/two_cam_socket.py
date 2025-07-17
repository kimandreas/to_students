from flask import Flask, render_template
from flask_socketio import SocketIO
import cv2
import base64
import threading
import time

app = Flask(__name__)
socketio = SocketIO(app)

def encode_frame(frame):
    _, buffer = cv2.imencode('.jpg', frame)
    return base64.b64encode(buffer).decode('utf-8')

def camera_thread(camera_id, event_name):
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        raise Exception(f"Camera {camera_id} could not be opened.")
    print(f"[INFO] Camera {camera_id} opened successfully.")

    while True:
        success, frame = cap.read()
        if not success:
            continue
        encoded_image = encode_frame(frame)
        socketio.emit(event_name, {'image': encoded_image})
        time.sleep(0.03)  # ~30 FPS

@app.route('/')
def index():
    return render_template('socketio_two_cam.html')

def start_threads():
    t1 = threading.Thread(target=camera_thread, args=(1, 'camera1_frame'))
    t2 = threading.Thread(target=camera_thread, args=(2, 'camera2_frame'))
    t1.daemon = True
    t2.daemon = True
    t1.start()
    t2.start()

if __name__ == '__main__':
    start_threads()
    # socketio.run(app, debug=True)
    socketio.run(app, debug=True, use_reloader=False)

