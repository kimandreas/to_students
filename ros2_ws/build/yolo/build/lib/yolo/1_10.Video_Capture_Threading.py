import cv2
import threading

# Flag to control the video loop
is_running = True

# Function to capture video frames in a separate thread
def capture_video():
    global is_running, frame
    # Open video capture from the webcam (0 is the default camera)
    cap = cv2.VideoCapture(0)
    
    while is_running:
        ret, frame = cap.read()  # Capture frame-by-frame
        if not ret:
            break

    cap.release()  # Release the capture when done

# Start the video capture thread
frame = None
video_thread = threading.Thread(target=capture_video)
video_thread.start()

# Display video frames in the main thread
while is_running:
    if frame is not None:
        cv2.imshow("Webcam Video Stream", frame)
        
    # Press 'q' to exit the loop and stop the video
    if cv2.waitKey(1) & 0xFF == ord('q'):
        is_running = False

# Wait for the video capture thread to finish
video_thread.join()
cv2.destroyAllWindows()
