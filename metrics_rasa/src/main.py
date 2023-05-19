#!/usr/bin/env python3

# standard libraries
import sys
import rospy
import rospkg
import requests
from threading import Thread
from queue import Queue
import copy

# internal classes
from log import Log
from input_output import InputOutput
from csv_tools import CSVTools

# standard messages
from std_msgs.msg import String

# custom messages
from ronsm_messages.msg import dm_intent

# constants and parameters
RASA_WEBHOOK = 'http://localhost:5005/webhooks/rest/webhook'
RASA_TRACKER = 'http://localhost:5005/conversations/metrics_rasa/tracker'
RASA_MIN_CONF = 0.75

class Main():
    def __init__(self):
        # set up logger
        self.id = 'main'
        self.logger = Log(self.id)
        self.logger.startup_msg()

        # set up ROS
        rospack = rospkg.RosPack()
        self.rel_path = rospack.get_path('metrics_rasa')

        self.ros_sub_text = rospy.Subscriber('/robot_hsr_asr/text', String, callback=self.ros_callback_text)
        self.ros_sub_rasa_utterance_internal = rospy.Subscriber('/metrics_rasa/rasa_utterance_internal', String, callback=self.ros_callback_rasa_utterance_internal)
        self.ros_sub_external_input = rospy.Subscriber('/metrics_rasa/external_input', String, callback=self.ros_callback_external_input)
        self.ros_sub_listen_once = rospy.Subscriber('/metrics_rasa/listen_once', String, callback=self.ros_callback_listen_once)

        rospy.init_node('metrics_rasa')

        # set up classes
        self.io = InputOutput(self.rel_path)
        self.csv_tool = CSVTools()

        # ready
        self.logger.log_great('Ready.')

        self.spin()

    # spins

    def spin(self):
        while not rospy.is_shutdown():
            rospy.sleep(1)

    # RASA interaction

    def send_to_rasa(self, utterance):
        self.csv_tool.save_utterance(utterance)

        request = {'message': utterance, 'sender': 'metrics_rasa'}

        response = requests.post(RASA_WEBHOOK, json = request)

        response = response.json()

        valid = requests.get(RASA_TRACKER)

        valid = valid.json()

        log = 'RASA State - Intent: ' + valid['latest_message']['intent']['name'] + ', Confidence: ' + str(valid['latest_message']['intent']['confidence'])
        self.logger.log(log)
        try:
            log = 'RASA Entities - Entity: ' + valid['latest_message']['entities'][0]['entity'] + ', Value: ' + valid['latest_message']['entities'][0]['value']
            self.logger.log(log)
        except:
            self.logger.log_warn('No entities extracted by RASA. This may be expected depending on the intent above.')
            
        try:
            response = response[0]
            message = response['text']
            
            print(message)
            self.io.request(message)
        except:
            self.logger.log_warn('No response provided by RASA. This may be intentional behaviour in some cases.')

    # callbacks

    def ros_callback_offer_help(self, msg):
        self.offer_help(msg.intent, msg.args)

    def ros_callback_text(self, msg):
        self.send_to_rasa(msg.data)

    def ros_callback_rasa_utterance_internal(self, msg):
        self.send_to_rasa(msg.data)

    def ros_callback_external_input(self, msg):
        self.send_to_rasa(msg.data)

    def ros_callback_listen_once(self, msg):
        utterance = self.io.listen_once()
        self.send_to_rasa(utterance)

if __name__ == '__main__':
    m = Main()