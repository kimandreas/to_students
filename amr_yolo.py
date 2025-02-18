from ultralytics import YOLO
import cv2

model = YOLO('/home/rokey8/rokey_ws/yolo/yolov8n.pt')

# trt_model = model.export (format="engine")
trt_model = YOLO('/home/rokey8/rokey_ws/yolo/yolov8n.engine', task="detect")

img = "/home/rokey8/rokey_ws/yolo/bus.jpg"

# Load the JPG image
frame = cv2.imread(img)  # Reads in BGR format

# Check if the image was loaded successfully
if frame is None:
    print("Error loading image")
else:
    print("Image shape:", frame.shape)  # (height, width, channels)

print("\n***standard inference")
results = model(frame)

print("\n***TensorFlow RT inference")
results = trt_model(frame)

print("\n***TensorFlow RT inference Tracking")
# Run tracking and get results
results = trt_model.track(source=frame, show=False, tracker='bytetrack.yaml')

for result in results:
    boxes = result.boxes  # Bounding boxes
    for box in boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])  # Convert to integers
        track_id = int(box.id[0]) if box.id is not None else None  # Tracking ID
        conf = float(box.conf[0])  # Confidence score
        cls = int(box.cls[0])  # Class ID
        
        # Get class name (optional)
        class_name = model.names[cls] if cls in model.names else "Unknown"

        # Draw bounding box
        color = (0, 255, 0)  # Green
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        # Display tracking ID, confidence, and class name
        label = f"ID: {track_id} {class_name} ({conf:.2f})"
        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


cv2.imshow('Tracked Image', frame)
cv2.waitKey(0)
cv2.destroyAllWindows()



