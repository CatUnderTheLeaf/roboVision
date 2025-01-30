import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from actions_ev3.action import EV3bot
from std_msgs.msg import String


class EV3botActionClient(Node):

    def __init__(self):
        super().__init__('alexa_action_client')
        self._action_client = ActionClient(self, EV3bot, 'EV3bot')
        
        self.sub = self.create_subscription(String, '/task', self.send_goal, 10)

    def send_goal(self, msg):
        """
        Send goal to the action server

        :param msg(string): task to be executed
        """

        self.get_logger().info(f'Your have a {msg.data} task from client')
        goal_msg = EV3bot.Goal()
        goal_msg.task =  msg.data
    
        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    action_client = EV3botActionClient()
    rclpy.spin(action_client)
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()