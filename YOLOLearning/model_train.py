from ultralytics import YOLO
import torch

print(torch.version.cuda)
print(torch.cuda.is_available())  # Should print True if CUDA is enabled
print(torch.cuda.current_device())  # Shows the current device (GPU)
print(torch.cuda.get_device_name(torch.cuda.current_device()))  # Shows the GPU name

# Load a model
#model = YOLO("yolov8n.yaml")  # build a new model from YAML
#model = YOLO("yolov8n.pt")  # load a pretrained model (recommended for training)
#model = YOLO("yolov8n.yaml").load("yolov8n.pt")  # build from YAML and transfer weights

model = YOLO("/home/xplanter/YOLOLearning/runs/detect/train14/weights/best.pt")

device = 'cuda' if torch.cuda.is_available() else 'cpu'
model.to(device)

# Train the model
results = model.train(data="/home/xplanter/YOLOLearning/TomatoDatasetV2/data.yaml", batch=4,imgsz=640, resume = True)

# Export the model
results.export(format="onnx")

