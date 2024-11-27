import cv2
import numpy as np

# List to store coordinates
coordinates = []

# Define a callback function to capture mouse click coordinates
def get_coordinates(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:  # Left mouse button click
        if len(coordinates) < 4:  # Only collect four clicks
            coordinates.append((x, y))
            print(f"Coordinate {len(coordinates)}: ({x}, {y})")
        
        # Check if we've collected four clicks
        if len(coordinates) == 4:
            # Stop further clicks by deactivating the callback
            cv2.setMouseCallback('Camera', lambda *args: None)  # Disables further clicks

# Start capturing video from the default camera
cap = cv2.VideoCapture(0)

# Set the mouse callback function on the camera feed window
cv2.namedWindow('Camera')
cv2.setMouseCallback('Camera', get_coordinates)

# Capture and display frames continuously
while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Draw each collected coordinate on the frame
    for (x, y) in coordinates:
        cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)  # Red circle with radius 5
    
    # Convert coordinates to the format required by cv2.polylines
    pts = np.array(coordinates, np.int32)
    pts = pts.reshape((-1, 1, 2))

    # Draw the quadrilateral
    cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

    cv2.imshow('Camera', frame)
    
    # Press 'Esc' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()

# Use the coordinates list as the return value
print("Final coordinates collected:", coordinates)
