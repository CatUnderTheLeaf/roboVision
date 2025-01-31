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
from rclpy.action import ActionClient
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch import LaunchContext

from actions_ev3.action import EV3bot

import threading

def main(args=None):
    
    # init rclpy in a separate thread
    threading.Thread(target=lambda: rclpy.init(args=args)).start()

    # init action client connected to 'EV3bot' action server
    # check action_name param, it should be the same 
    action_client = ActionClient(Node('alexa_app_client'), EV3bot, "EV3bot")
    

    def send_goal(task='0'):
        """
        Send goal to the action server

        :param task(string): task to be executed

        # Tasks:
        # 0 - stop the robot
        # 1 - move forwards
        # 2 - move backwards
        # 3 - turn left degrees
        # 4 - turn right degrees
        # q - quit the robot
        # speak - greetings phrase
        """
        
        goal_msg = EV3bot.Goal()
        goal_msg.task = task
        
        action_client.wait_for_server()

        action_client.send_goal_async(goal_msg)


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
            
            # goal = EV3bot.Goal()
            # goal.task='speak'
            # action_client.send_goal('speak')

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

            send_goal('1')

            return handler_input.response_builder.response

    class RotateIntentHandler(AbstractRequestHandler):
        def can_handle(self, handler_input):
            # type: (HandlerInput) -> bool
            return is_intent_name("RotateIntent")(handler_input)

        def handle(self, handler_input):
            # type: (HandlerInput) -> Response
            speech_text = "Ok, I'm rotating"

            handler_input.response_builder.speak(speech_text).set_card(
                SimpleCard("Rotate", speech_text)).set_should_end_session(
                False)

            send_goal('4')

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
            
            send_goal('q')

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
            
            send_goal('0')

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
            

    # create the flask app
    # and add the skill builder to the app
    app = Flask(__name__)
    skill_builder = SkillBuilder()
    skill_builder.add_request_handler(LaunchRequestHandler())
    skill_builder.add_request_handler(DriveIntentHandler())
    skill_builder.add_request_handler(PauseIntentHandler())
    skill_builder.add_request_handler(StopIntentHandler())
    skill_builder.add_request_handler(RotateIntentHandler())
    skill_builder.add_exception_handler(AllExceptionHandler())
    
    # get the skill id from the config file from the share directory
    # this file contains secret skill id
    filename = PathJoinSubstitution([FindPackageShare('voice_commands'), 'config', 'alexa-skill-config.yaml']).perform(LaunchContext())
    try:
        with open(filename, mode="r") as f:
            config = yaml.safe_load(f)
    except OSError as error:
        print(strerror(error.errno))
    except yaml.YAMLError as exc:
        print(exc)

    if config is None:
        print('please add skill_id to your config file')
        sys.exit()

    skill_adapter = SkillAdapter(
        skill=skill_builder.create(), 
        skill_id=config['skill_id'], 
        app=app
    )

    skill_adapter.register(app=app, route="/")

    app.run()


if __name__ == '__main__':    
    main()