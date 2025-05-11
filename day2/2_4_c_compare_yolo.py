from ultralytics import YOLO
import cv2

try:
    model = YOLO('/home/rokey-kim/Documents/ros2_ws/src/day2/day2/yolov8n.pt')

    img = "/home/rokey-kim/Documents/ros2_ws/src/day2/day2/bus.jpg"

    # Load the JPG image
    frame = cv2.imread(img)  # Reads in BGR format

    # Check if the image was loaded successfully
    if frame is None:
        print("Error loading image")
        exit(1)
    else:
        print("Image shape:", frame.shape)

    print("\n***standard inference CPU")
    results = model(frame, device='cpu')

    print("\n***standard inference GPU")
    results = model(frame)

    print("\n***TensorFlow RT inference Tracking")
    results = model.track(source=frame, show=False, tracker='bytetrack.yaml')

    for result in results:
        boxes = result.boxes
        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            track_id = int(box.id[0]) if box.id is not None else None
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            class_name = model.names[cls] if cls in model.names else "Unknown"

            color = (0, 255, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            label = f"ID: {track_id} {class_name} ({conf:.2f})"
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    cv2.imshow('Tracked Image', frame)
    print("Press Ctrl+C to quit.")

    while True:
        if cv2.waitKey(10) == ord('q'):  # q can also exit
            break

except KeyboardInterrupt:
    print("\nCtrl+C detected. Exiting...")

finally:
    cv2.destroyAllWindows()
