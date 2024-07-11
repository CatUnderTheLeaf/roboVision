# from ultralytics import YOLOv10
from ultralytics import YOLOWorld
import cv2
import torch

print(torch.__version__)
print(torch.version.cuda)

print(torch.cuda.is_available())
device = 'cuda' if torch.cuda.is_available() else 'cpu'


# Load a pretrained YOLOv8s-worldv2 model
# model = YOLOWorld("yolov8s-worldv2.pt")

# Train the model on the COCO8 example dataset for 100 epochs
# results = model.train(data="lvis.yaml", epochs=1, imgsz=240)

# num_gpus = torch.cuda.device_count()
# for i in range(num_gpus):
#     print(f'GPU {i}: {torch.cuda.get_device_name(i)}')
#     print(f'  Memory Allocated: {torch.cuda.memory_allocated(i)} bytes')
#     print(f'  Memory Cached: {torch.cuda.memory_reserved(i)} bytes')
# torch
# model = YOLOv10.from_pretrained('jameslahm/yolov10x').to(device)
model = YOLOWorld("yolov8s-worldv2.pt").to(device)
print(model.device.type)
# source = 'http://images.cocodataset.org/val2017/000000039769.jpg'
source = '/home/cat/projects/roboVision/src/yolo/yolo/PXL_20240708_121649628.jpg'
stream = False
save = True
# source = 'http://192.168.0.30:4747/video?320x240'
# stream = True
# save = False


# uses openai/CLIP to use custom prompts
# downloads /home/cat/.cache/clip/ViT-B-32.pt 
model.set_classes([ "keyboard", "lego"])

results = model.predict(source=source, stream=stream, save=save, conf=0.01)
# for result in results:
#     det_annotated = result.plot(show=False)
#     # det_annotated[0].verbose()
#     cv2.imshow('image', det_annotated)
#     if cv2.waitKey(25) & 0xFF == ord('q'):
#       break 

# cv2.destroyAllWindows()
