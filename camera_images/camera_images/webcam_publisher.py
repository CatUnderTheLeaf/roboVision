import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImagePublisher(Node):

    def __init__(self):
        super().__init__("image_publisher")
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture('http://192.168.0.30:4747/video?320x240')
        self.publisher_ = self.create_publisher(Image, "/image", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.i += 1
        try:
            r, frame = self.cap.read()
            if not r:
                return
            self.publisher_.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

            # # BGR8
            # self.bgr8pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

            # # RGB8
            # frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # self.rgb8pub.publish(self.bridge.cv2_to_imgmsg(frame_rgb, "rgb8"))

            # # MONO8
            # frame_mono = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # self.mono8pub.publish(self.bridge.cv2_to_imgmsg(frame_mono, "mono8"))

        except CvBridgeError as e:
            print(e)


def main(args=None):
    rclpy.init(args=args)

    image_publisher = ImagePublisher()

    rclpy.spin(image_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_publisher.cap.release()
    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()