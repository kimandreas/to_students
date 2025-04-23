import cv2
import numpy as np

coordinates = []

def get_coordinates(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(coordinates) < 4:
            coordinates.append((x, y))
            print(f"Coordinate {len(coordinates)}: ({x}, {y})")
        if len(coordinates) == 4:
            print("Final coordinates collected:", coordinates)
            cv2.setMouseCallback('Camera', lambda *args: None)

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    cv2.namedWindow('Camera')
    cv2.setMouseCallback('Camera', get_coordinates)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Draw red circles at each selected point
        for (x, y) in coordinates:
            cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)

        # Draw polygon if at least two points
        if len(coordinates) >= 2:
            pts = np.array(coordinates, np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

        cv2.imshow('Camera', frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('r'):
            print("Resetting points...")
            coordinates.clear()
            cv2.setMouseCallback('Camera', get_coordinates)

        elif key == ord('q'):
            print("Shutting down...")
            break

    cap.release()
    cv2.destroyAllWindows()
    print("Final coordinates collected:", coordinates)

if __name__ == '__main__':
    main()
