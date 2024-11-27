#이 코드는 물체가 검출될 때마다 이미지를 저장하고, 각 물체의 바운딩 박스 좌표, 정확도, 클래스를 'output.csv'라는 이름의 CSV 파일로 저장하며, 
#같은 정보를 'output.json'이라는 이름의 JSON 파일로도 저장합니다. 또한 물체 감지 최고 숫자와 정확도 평균 값을 'statistics.csv'라는 이름의 CSV 파일에 저장합니다. 
#파일 이름은 원하는대로 수정하실 수 있습니다.

import json
import csv
import time
from ultralytics import YOLO
import cv2
import math 
import os
import shutil
import sys
import sys

def run_yolo(model,output_dir):
    # cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture(0) #set to USB Camera
    cap.set(3, 640)
    cap.set(4, 480)

    while True:
        success, img = cap.read()
        results = model(img, stream=True)

        object_count = 0  # 물체 카운트 초기화
                # classNames = ['Normal','Error']
        classNames = ['Truck','Dummy']

        csv_output = []
        confidences = []

        fontScale = 1  # fontScale을 반복문 밖에서 미리 정의
        max_object_count = 0

        for r in results:
            boxes = r.boxes

            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)

                confidence = math.ceil((box.conf[0]*100))/100
                cls = int(box.cls[0])
                confidences.append(confidence)

                org = [x1, y1]
                font = cv2.FONT_HERSHEY_SIMPLEX
                color = (255, 0, 0)
                thickness = 2

                # 탐지된 물체의 이름과 함께 정확도를 화면에 표시
                cv2.putText(img, f"{classNames[cls]}: {confidence}", org, font, fontScale, color, thickness)

                # COCO 데이터셋 형식의 JSON 데이터를 CSV로 변환
                csv_output.append([x1, y1, x2, y2, confidence, cls])

                object_count += 1  # 물체 카운트 증가

        max_object_count = max(max_object_count, object_count)

        # 탐지된 물체의 수를 화면에 표시
        cv2.putText(img, f"Objects_count: {object_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, fontScale, (0, 255, 0), 1)

        # 물체가 검출될 때마다 이미지 저장
        if object_count > 0:
            cv2.imwrite(os.path.join(output_dir,f'output_{int(time.time())}.jpg'), img)

        cv2.imshow('Webcam', img)

        if cv2.waitKey(1) == ord('q'):
            # 키보드 'q'를 누르면 CSV 파일과 JSON 파일을 저장
            with open(os.path.join(output_dir,'output.csv'), 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerows(csv_output)
            with open(os.path.join(output_dir,'output.json'), 'w') as file:
                json.dump(csv_output, file)
            # 물체 감지 최고 숫자와 정확도 평균 값을 CSV로 출력
            with open(os.path.join(output_dir,'statistics.csv'), 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Max Object Count', 'Average Confidence'])
                writer.writerow([max_object_count, sum(confidences) / len(confidences) if confidences else 0])
            break
    
    cap.release()
    cv2.destroyAllWindows()
    

def main(pt_file):

    # yolo_best_pt_path = './best.pt'
    # yolo_best_pt_path = pt_file

    # if os.path.exists(yolo_best_pt_path):    
    if os.path.exists(pt_file):
        # model = YOLO('./yolov8_obj_det_best/best.pt')
        # model = YOLO('./best.pt')
        model = YOLO(pt_file)

        output_dir = './output'

        # Check if the directory exists
        if os.path.exists(output_dir) and os.path.isdir(output_dir):
            # Delete the directory
            shutil.rmtree(output_dir)
            print(f"The directory {output_dir} has been deleted.")

        os.mkdir(output_dir)

        run_yolo(model,output_dir)
    else:
        print(f"Not found: {pt_file}")


if __name__ == "__main__":

    if len(sys.argv) < 2:
        print("Please provide an .pt filename as an argument")
    else:
        main(sys.argv[1])