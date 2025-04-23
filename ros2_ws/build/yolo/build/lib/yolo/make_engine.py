from ultralytics import YOLO
import torch

if torch.cuda.is_available():
    print(torch.version.cuda)
    best_pt = input("Enter file path to saved pt: ")

    model = YOLO(best_pt)
    trt_model = model.export (format="engine")
    print("CUDA version of engine file successfully created.  Look for corresponding .onnx and .engine files.")

else:
    print("CUDA is not available. Please check your GPU and CUDA installation.")
