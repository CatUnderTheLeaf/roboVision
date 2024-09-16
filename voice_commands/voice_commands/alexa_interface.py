#!/usr/bin/env python3

from flask import Flask
import yaml
import sys
import os
from os import strerror
from ask_sdk_core.skill_builder import SkillBuilder
from flask_ask_sdk.skill_adapter import SkillAdapter

from ask_sdk_core.dispatch_components import AbstractRequestHandler, AbstractExceptionHandler
from ask_sdk_core.utils import is_request_type, is_intent_name
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model import Response
from ask_sdk_model.ui import SimpleCard

import rclpy
from rclpy.node import Node
# from rclpy.action import ActionClient
# from std_msgs.msg import String

# from ev3_msgs.action import ev3Task

import threading

threading.Thread(target=lambda: rclpy.init()).start()

# load configuration
# ugly method, but at least it works
# idk why i can't read this path from args sent by launch
path = os.path.join('/'.join((sys.argv[-1]).split('/')[:-3]), 'share', 'voice_commands', 'config', 'alexa-skill-config.yaml')
try:
    with open(path, mode="r") as f:
        config = yaml.safe_load(f)
except OSError as error:
    print(strerror(error.errno))
except yaml.YAMLError as exc:
    print(exc)

if config is None:
    print('empty config file')
    sys.exit()


# action_client = ActionClient(Node("alexa_client"), ev3Task, "ev3_server")

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')

        # Publisher for the camera info
        # self.publisher_ = self.create_publisher(String, 'cmd_vel', 1)

#     def publish(self, msg):
#         # Add timestamp to camera info message
#         cmd_vel_msg = String()
#         # camera_info_msg.header.stamp = self.get_clock().now().to_msg()
#         # camera_info_msg.header.frame_id = 'camera'
#         cmd_vel_msg.data = msg
        
#         self.publisher_.publish(cmd_vel_msg)

pub = CmdVelPublisher()

class LaunchRequestHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_request_type("LaunchRequest")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Hi, you can now control the robot!"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Hello World", speech_text)).set_should_end_session(
            False)
        
        # goal = ev3Task.Goal()
        # goal.task_number=0
        # action_client.send_goal_async(goal)
        # pub.publish("Launch request to control")
        pub.get_logger().info("Launch request to control")

        return handler_input.response_builder.response

class DriveIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("DriveIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Ok, I'm moving"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Drive", speech_text)).set_should_end_session(
            False)

        # goal = ArduinobotTask.Goal()
        # goal.task_number = 1
        # action_client.send_goal_async(goal)
        pub.get_logger().info("DriveIntent in action")

        return handler_input.response_builder.response

class StopIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("AMAZON.CancelIntent")(handler_input) or is_intent_name("AMAZON.StopIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "ending control"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("End", speech_text)).set_should_end_session(
            True)
        
        # goal = ev3Task.Goal()
        # goal.task_number=0
        # action_client.send_goal_async(goal)

        # pub.publish("Request to stop")
        pub.get_logger().info("Request to end control")

        return handler_input.response_builder.response

class PauseIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("PauseIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "stop moving"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("stop moving", speech_text)).set_should_end_session(
            False)
        
        # goal = ev3Task.Goal()
        # goal.task_number=0
        # action_client.send_goal_async(goal)

        # pub.publish("Request to stop")
        pub.get_logger().info("Request to stop")

        return handler_input.response_builder.response

class AllExceptionHandler(AbstractExceptionHandler):

    def can_handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> bool
        return True

    def handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> Response

        speech = "Hmm, I don't know that. Can you please say it again?"
        handler_input.response_builder.speak(speech).ask(speech)
        return handler_input.response_builder.response

app = Flask(__name__)
skill_builder = SkillBuilder()
skill_builder.add_request_handler(LaunchRequestHandler())
skill_builder.add_request_handler(DriveIntentHandler())
skill_builder.add_request_handler(PauseIntentHandler())
skill_builder.add_request_handler(StopIntentHandler())
skill_builder.add_exception_handler(AllExceptionHandler())

skill_adapter = SkillAdapter(skill=skill_builder.create(), skill_id=config['skill_id'], app=app)

@app.route("/")
def invoke_skill():
    return skill_adapter.dispatch_request()

skill_adapter.register(app=app, route="/")


def main(args=None):
    app.run(port=config['flask_port'])


if __name__ == '__main__':
    main()