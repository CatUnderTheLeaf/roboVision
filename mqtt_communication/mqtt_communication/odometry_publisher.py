import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import rclpy.time
from std_msgs.msg import String
from nav_msgs.msg import Odometry
# from rosidl_runtime_py import message_to_yaml
# import time
import yaml
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher_node')
        self.subscription = self.create_subscription(
            String,
            '/odom/primitive',
            self.robot_odom_callback,
            1)
        self.publisher_ = self.create_publisher(Odometry, '/odom', 1)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 1)
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def robot_odom_callback(self, msg):
        message = yaml.safe_load(msg.data)
        # TODO difference between time.time() and get_clock().now() ?????
        nodesec, nodenanosec = self.get_clock().now().seconds_nanoseconds()
        # sec, nanosec = int(message['sec']), int(message['nanosec'])
        sec, nanosec = map((int), str(message['time']).split('.'))
        self.get_logger().info(f"time difference: {nodesec-sec} seconds, {nodenanosec-nanosec} nanoseconds")

        # publish joints states
        joint_state = JointState()
        joint_state.header.stamp = rclpy.time.Time(seconds=sec, nanoseconds=nanosec).to_msg()
        joint_state.name = ['drivewhl_l_joint', 'drivewhl_r_joint']
        joint_state.position = [float(message['left_wheel']), float(message['right_wheel'])]
        joint_state.velocity = [float(message['left_wheel_vel']), float(message['right_wheel_vel'])]
        self.joint_pub.publish(joint_state)

        
        orient = quaternion_from_euler(0.0, 0.0, float(message['heading']));
        
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = rclpy.time.Time(seconds=sec, nanoseconds=nanosec).to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = float(message['x'])
        t.transform.translation.y = float(message['y'])
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        t.transform.rotation.x = orient[0]
        t.transform.rotation.y = orient[1]
        t.transform.rotation.z = orient[2]
        t.transform.rotation.w = orient[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

        # create and publish odometry message
        new_msg = Odometry()
        new_msg.header.stamp = rclpy.time.Time(seconds=sec, nanoseconds=nanosec).to_msg()
        new_msg.header.frame_id = 'odom'
        new_msg.child_frame_id = 'base_link'
        new_msg.pose.pose.position.x = float(message['x'])
        new_msg.pose.pose.position.y = float(message['y'])
        new_msg.pose.pose.position.z = 0.0
        new_msg.pose.pose.orientation.x = orient[0]
        new_msg.pose.pose.orientation.y = orient[1]
        new_msg.pose.pose.orientation.z = orient[2]
        new_msg.pose.pose.orientation.w = orient[3]
        new_msg.twist.twist.linear.x = float(message['linear'])
        new_msg.twist.twist.angular.z = float(message['angular'])
        self.publisher_.publish(new_msg)



def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)
    odometry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()