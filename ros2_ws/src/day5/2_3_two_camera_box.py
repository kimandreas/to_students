from flask import Flask, render_template, Response
import cv2
import numpy as np

app = Flask(__name__)

# List of store coordinates
pt_1 = (460, 0)
pt_2 = (640, 0)
pt_3 = (640, 120)
pt_4 = (460, 120)
coordinates = [pt_1, pt_2, pt_3, pt_4]

def generate_frames_box(camera_id):
    camera = cv2.VideoCapture(camera_id)

    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            # # Draw each collected coordinate on the frame
            # for (x, y) in coordinates:
            #     cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)  # Red circle with radius 5
            
            # Convert coordinates to the format required by cv2.polylines
            pts = np.array(coordinates, np.int32)
            pts = pts.reshape((-1, 1, 2))

            # Draw the quadrilateral
            cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
            # print ("drawed polyline")

            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            
def generate_frames(camera_id):
    camera = cv2.VideoCapture(camera_id)
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            
@app.route('/')
def index():
    return render_template('two_cam_index.html')

@app.route('/video_feed1')
def video_feed1():
    return Response(generate_frames_box(0), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed2')
def video_feed2():
    return Response(generate_frames(1), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(debug=True)
