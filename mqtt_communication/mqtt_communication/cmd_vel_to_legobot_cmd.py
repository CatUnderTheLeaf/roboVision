import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import rclpy.time
from std_msgs.msg import String
from rosidl_runtime_py import message_to_yaml

class CmdVelConverter(Node):

    def __init__(self):
        super().__init__('cmd_vel_converter_node')
        self.subscription = self.create_subscription(
            TwistStamped,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.publisher_ = self.create_publisher(String, 'ros_bot_cmd/primitive', 10)
        
        # rate = 30
        # timer_period = 1 / rate
        # self.timer = self.create_timer(timer_period, self.image_callback)


    def cmd_vel_callback(self, msg):

        new_msg = String()
        new_msg.data = '\n'.join(['cmd: drive', f'sec: {msg.header.stamp.sec}', f'nanosec: {msg.header.stamp.nanosec}', f'linear: {msg.twist.linear.x}', f'angular: {msg.twist.angular.z}'])
        self.publisher_.publish(new_msg)        

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_converter = CmdVelConverter()
    rclpy.spin(cmd_vel_converter)
    cmd_vel_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
