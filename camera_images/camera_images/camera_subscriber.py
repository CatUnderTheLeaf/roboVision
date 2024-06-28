import rclpy
from rclpy.node import Node

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CamSubscriber(Node):

    def __init__(self):
        super().__init__("image_subscriber")

        # rate = self.declare_parameter('rate', 30).value # 30 Hz by default
        # camera_url = self.declare_parameter('camera_url', '0').value

        self.bridge = CvBridge()
        self.FPS_MS = int(0.1 * 1000)

        self.subscription = self.create_subscription(Image, '/image', self.img_callback, 1)
        
        
        # timer_period = 0.1 #1 / rate
        # self.timer = self.create_timer(timer_period, self.timer_callback)

    def img_callback(self, data):
        # self.get_logger().info('Receiving video frame')
        current_frame = self.bridge.imgmsg_to_cv2(data)
        cv2.imshow("camera", current_frame)   
        cv2.waitKey(self.FPS_MS)


def main(args=None):
    rclpy.init(args=args)

    cam_subscriber = CamSubscriber()

    rclpy.spin(cam_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # cam_subscriber.cap.release()
    cam_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()