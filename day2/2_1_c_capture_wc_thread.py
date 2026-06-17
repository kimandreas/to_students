import cv2
import threading
import time

# Shared variables
shared_frame = None
lock = threading.Lock()
is_running = True

def webcam_thread():
    global shared_frame, is_running
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Failed to open webcam.")
        is_running = False
        return

    while is_running:
        ret, frame = cap.read()
        if ret:
            with lock:
                shared_frame = frame.copy()
        time.sleep(0.001)

    cap.release()

def heavy_processing(frame):
    # Simulated heavy CPU work
    for _ in range(10):
        frame = cv2.GaussianBlur(frame, (9, 9), 0)
    return frame

def run_threaded():
    global is_running
    is_running = True
    t=threading.Thread(target=webcam_thread)
    t.start()
    print("Running THREADED version for 10 seconds...")

    frame_count = 0
    start = time.time()

    while time.time() - start < 10:
        with lock:
            frame = shared_frame.copy() if shared_frame is not None else None

        if frame is not None:
            processed = heavy_processing(frame)
            cv2.imshow("Threaded", processed)
            frame_count += 1

        cv2.waitKey(1)

    is_running = False
    end = time.time()
    t.join()
    cv2.destroyAllWindows()
    print(f"[THREADED] Avg FPS: {frame_count / (end - start):.2f}")

def run_inline():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Failed to open webcam.")
        return

    print("Running INLINE (non-threaded) version for 10 seconds...")

    frame_count = 0
    start = time.time()

    while time.time() - start < 10:
        ret, frame = cap.read()
        if not ret:
            continue

        processed = heavy_processing(frame)
        cv2.imshow("Inline", processed)
        frame_count += 1
        cv2.waitKey(1)

    cap.release()
    end = time.time()
    cv2.destroyAllWindows()
    print(f"[INLINE] Avg FPS: {frame_count / (end - start):.2f}")

if __name__ == "__main__":
    choice = input("Enter 't' for threaded, anything else for inline: ").strip().lower()
    if choice == 't':
        run_threaded()
    else:
        run_inline()
