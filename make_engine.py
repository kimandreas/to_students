from ultralytics import YOLO

model = YOLO('/home/rokey8/rokey_ws/yolo/yolov8n.pt')

trt_model = model.export (format="engine")
