# #!/usr/bin/env python3

from flask import Flask
import yaml
import sys
import os
import signal
from threading import Thread
from os import strerror
from ask_sdk_core.skill_builder import SkillBuilder
from flask_ask_sdk.skill_adapter import SkillAdapter

from ask_sdk_core.dispatch_components import AbstractRequestHandler, AbstractExceptionHandler
from ask_sdk_core.utils import is_request_type, is_intent_name
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model import Response
from ask_sdk_model.ui import SimpleCard

import rclpy
from std_msgs.msg import String


# from actions_ev3.action import EV3bot

# Initializing the node
rclpy.init(args=None)
node = rclpy.create_node('alexa_publisher_node', namespace="/")

# Starting the Thread with a target in the node
Thread(target=lambda:node).start() 

pub = node.create_publisher(String, '/ping/primitive', 1)

def send_command(task):
    # Task numbers:
    # 0 - stop the robot
    # 1 - move forwards
    # 2 - move backwards
    # 3 - turn left degrees
    # 4 - turn right degrees
    # q - quit the robot
    # speak - greetings phrase
    rclpy.spin_once(node,timeout_sec=1.0)

    new_message = String()
    new_message.data = task
    pub.publish(new_message)


# config file
path = node.declare_parameter('config', '/').value
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
        # rclpy.spin_once(node,timeout_sec=1.0)

        send_command("speak")
        
        node.get_logger().info("Launch request to control")

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
        
        send_command("1")

        node.get_logger().info("DriveIntent in action")

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
        
        send_command("q")
        node.get_logger().info("Request to end control")

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
        
        send_command("0")
        node.get_logger().info("Request to stop")

        return handler_input.response_builder.response

class AllExceptionHandler(AbstractExceptionHandler):

    def can_handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> bool
        return True

    def handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> Response

        send_command("0")
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

## Function that finish the actual context
def signal_handler(signal, frame):
    rclpy.shutdown()
    sys.exit(0)

signal.signal(signal.SIGINT,signal_handler) # Calls the 'signal_handler' and finish the actual signal (like a Ctrl + C)