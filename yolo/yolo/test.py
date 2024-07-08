from ultralytics import YOLOv10
import cv2
import torch

print(torch.cuda.is_available())
device = 'cuda' if torch.cuda.is_available() else 'cpu'
num_gpus = torch.cuda.device_count()
for i in range(num_gpus):
    print(f'GPU {i}: {torch.cuda.get_device_name(i)}')
    print(f'  Memory Allocated: {torch.cuda.memory_allocated(i)} bytes')
    print(f'  Memory Cached: {torch.cuda.memory_reserved(i)} bytes')
# torch
model = YOLOv10.from_pretrained('jameslahm/yolov10x').to(device)
print(model.device.type)
source = 'http://images.cocodataset.org/val2017/000000039769.jpg'
results = model.predict(source=source)
# print(results[0].orig_img.shape)
# det_annotated = results[0].plot(show=False)
# img = results[0].orig_img.shape
# cv2.imshow('image', det_annotated)
# cv2.waitKey(1)
# cv2.destroyAllWindows()
