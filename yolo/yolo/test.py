from ultralytics import YOLOv10
import cv2

model = YOLOv10.from_pretrained('jameslahm/yolov10x')
source = 'http://images.cocodataset.org/val2017/000000039769.jpg'
results = model.predict(source=source)
# print(results[0].orig_img.shape)
det_annotated = results[0].plot(show=False)
# img = results[0].orig_img.shape
cv2.imshow('image', det_annotated)
cv2.waitKey(0)
cv2.destroyAllWindows()
