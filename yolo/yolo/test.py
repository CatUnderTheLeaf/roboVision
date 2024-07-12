# from ultralytics import YOLOv10
from ultralytics import YOLOWorld
import cv2
import torch

print(torch.__version__)
print(torch.version.cuda)

print(torch.cuda.is_available())
device = 'cuda' if torch.cuda.is_available() else 'cpu'

# model = YOLOv10.from_pretrained('jameslahm/yolov10x').to(device)
model_weights = "yolov8s-worldv2.pt"
model_weights = '/home/cat/projects/roboVision/runs/detect/train12/weights/best.pt'
model = YOLOWorld(model_weights).to(device)
print(model.device.type)
# source = 'http://images.cocodataset.org/val2017/000000039769.jpg'
source = '/home/cat/projects/roboVision/src/yolo/yolo/PXL_20240708_121649628.jpg'
stream = False
save = True
# source = 'http://192.168.0.30:4747/video?320x240'
# stream = True
save = False


# uses openai/CLIP to use custom prompts
# downloads /home/cat/.cache/clip/ViT-B-32.pt 
model.set_classes([ "yellow lego", 'black lego', 'white lego', 'brown lego', 'blue lego', 'lego figure'])

results = model.predict(source=source, stream=stream, save=save, conf=0.01)
for result in results:
    det_annotated = result.plot(show=False)
    # det_annotated[0].verbose()
    cv2.imshow('image', det_annotated)
    if cv2.waitKey(25) & 0xFF == ord('q'):
      cv2.destroyAllWindows()
      break 

