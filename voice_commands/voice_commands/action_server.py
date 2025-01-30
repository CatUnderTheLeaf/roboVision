import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.duration import Duration

from actions_ev3.action import EV3bot
from std_msgs.msg import String

from geometry_msgs.msg import TwistStamped


class EV3botActionServer(Node):

    def __init__(self):
        super().__init__('ev3bot_action_server')
        self._action_server = ActionServer(
            self,
            EV3bot,
            'EV3bot',
            self.execute_callback)
        # Publisher for the task message
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)

    def get_twist_msg(self, task):
        new_message = TwistStamped()        
        
        # Task numbers:
        # 0 - stop the robot
        # 1 - move forwards
        # 2 - move backwards
        # 3 - turn left degrees
        # 4 - turn right degrees
        # q - quit the robot
        # speak - greetings phrase
        match task:
            case "0":
                new_message.twist.linear.x = 0.0
                new_message.twist.angular.z = 0.0
            case "1":
                new_message.twist.linear.x = 0.2
                new_message.twist.angular.z = 0.0
            case "2":
                new_message.twist.linear.x = -0.2
                new_message.twist.angular.z = 0.0
            case "3":
                new_message.twist.linear.x = 0.0
                new_message.twist.angular.z = -3.14
            case "4":
                new_message.twist.linear.x = 0.0
                new_message.twist.angular.z = 3.14
            case "q":
                new_message.twist.linear.x = 0.0
                new_message.twist.angular.z = 0.0
            # case "speak":
            #     new_message.data = "cmd: speak"
            case _:
                new_message.twist.linear.x = 0.0
                new_message.twist.angular.z = 0.0

        return new_message

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # TODO how to use feedback in this case
        # feedback_msg = EV3bot.Feedback()
       
        

        self.get_logger().info(f"publishing cmd_vel for {goal_handle.request.task} task")

        cmd_vel = self.get_twist_msg(goal_handle.request.task)

        # dummy loop to simulate the robot moving
        start = self.get_clock().now()
        duration = Duration(seconds=10)
        while (self.get_clock().now() - start) < duration:
            cmd_vel.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(cmd_vel)
            self.get_clock().sleep_for(Duration(seconds=0.1))

        # make sure the robot stops after the loop
        cmd_vel = self.get_twist_msg('0')
        cmd_vel.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(cmd_vel)

        goal_handle.succeed()
        result = EV3bot.Result()
        result.success = True
        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = EV3botActionServer()
    rclpy.spin(action_server)
    action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()