import cv2

x1, y1, x2, y2 = 100, 100, 200, 200
color = (255, 0, 0)
thickness = 2

cap=cv2.VideoCapture(0) # Open PC Camera
cap.set(3, 640) #cv2.CAP_PROP_FRAME_WIDTH
cap.set(4, 680) #cv2.CAP_PROP_FRAME_HEIGHT


def display_box():

    while True:
        #Read a frame
        ret, frame=cap.read()
        
        if not ret:
            break

        cv2.rectangle(frame, (x1,y1), (x2,y2), color, thickness)

        #display a frame
        cv2.imshow("webcam", frame)

        #capture a key input
        key = cv2.waitKey(1)

        #if key input is q then quit
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

display_box()