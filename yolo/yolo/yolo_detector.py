import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
from ros2_numpy import numpify, msgify
from ultralytics import YOLOv10

class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_node')
        self.subscription = self.create_subscription(
            Image,
            'image',
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(Image, 'detected_image', 10)
        # self.bridge = CvBridge()
        self.model = YOLOv10.from_pretrained('jameslahm/yolov10x')
        self.get_logger().info('YOLO Node has been started.')

    def image_callback(self, msg):

        array = numpify(msg)
        # if self.publisher_.get_num_connections():
        det_result = self.model(array)
        det_annotated = det_result[0].plot(show=False)
        self.publisher_.publish(msgify(Image, det_annotated, encoding="rgb8"))
        # don't want to use CvBridge, it has some speed problems

        # try:
        #     # Convert ROS Image message to OpenCV image
        #     cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # except CvBridgeError as e:
        #     self.get_logger().error(f'Error converting image: {e}')
        #     return

        # # Perform object detection
        # results = self.model.predict(cv_image)

        # # Draw bounding boxes on the image
        # for box in results[0].boxes.xyxy:
        #     x1, y1, x2, y2 = map(int, box[:4])
        #     label = f'{self.model.names[int(box[5])]}: {box[4]:.2f}'
        #     color = (0, 255, 0)
        #     cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 2)
        #     cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # try:
        #     # Convert OpenCV image back to ROS Image message
        #     detected_image_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
        #     # Publish the detected image
        #     self.publisher_.publish(detected_image_msg)
        # except CvBridgeError as e:
        #     self.get_logger().error(f'Error converting image: {e}')

def main(args=None):
    rclpy.init(args=args)
    yolo = YoloDetector()
    rclpy.spin(yolo)
    yolo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
