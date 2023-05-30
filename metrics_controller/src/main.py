#!/usr/bin/env python3

# standard libraries
import rospy
import smach
from enum import Enum

# internal classes
from log import Log
from speak import Speak
from object_to_tf import ObjectToTF
from move_to_position import MoveToPosition

# standard messages
from std_msgs.msg import String
from hsrb_interface import Robot, exceptions

# custom messages
from ronsm_messages.msg import dm_intent

# constants and parameters
MAX_USER_INPUT_RETRIES = 3
MAX_USER_INPUT_WAIT = 30
MAX_PERSON_SEARCH_TRIES = 6
ICRA_POSITIONS = ['view_reading_room', 'view_hall_room', 'view_dining_room', 'view_living_room']

class State(Enum):
    IDLE = 0
    APPROACH_WAIT = 1
    APPROACH_AUTO = 2
    OFFER_BRING = 3
    FAIL = 99

class Main():
    def __init__(self):
        # set up logger
        self.id = 'main'
        self.logger = Log(self.id)
        self.logger.startup_msg()

        # set up ROS
        rospy.init_node('metrics_controller')

        self.rps_sub_intent_bus = rospy.Subscriber('/metrics_rasa_core/intent_bus', dm_intent, callback=self.ros_callback_intent_bus)
        self.ros_sub_start_signal = rospy.Subscriber('/metrics_controller/start_signal', String, callback=self.ros_callback_start_signal)

        self.ros_pub_service_listen_once = rospy.Publisher('/metrics_rasa/listen_once', String, queue_size=10)

        # set up HSR
        self.robot = Robot()
        try:
            self.logger.log('Waiting on robot resources...')
            self.base = self.robot.try_get('omni_base')
            self.body = self.robot.try_get('whole_body')
            self.grip = self.robot.try_get('gripper')
        except exceptions.ResourceNotFoundError:
            self.logger.log_warn('Unable to get one or more handles for HSR resources. Another process may be using them or the robot is in an error state.')
            self.speak.request('Controller failed to start, please check console.')
            exit(0)

        # set up classes
        self.speak = Speak()
        self.object_to_transform = ObjectToTF()
        self.move_to_position = MoveToPosition(self.body)

        # instance variables
        self.state = State.IDLE
        self.start = False
        self.intent = None

        # ready
        self.logger.log_great('Ready.')
        self.speak.request('Controller is ready.')

        # loop
        while not rospy.is_shutdown():
            self.loop()

    # main loop

    def loop(self):
        log = 'Current state: ' + str(self.state)
        self.logger.log(log)

        if self.state == State.IDLE:
            self.state_idle()
        elif self.state == State.APPROACH_WAIT:
            self.state_approach_wait()
        elif self.state == State.APPROACH_AUTO:
            self.state_approach_auto()
        elif self.state == State.OFFER_BRING:
            self.state_offer_bring()
        elif self.state == State.FAIL:
            self.state_fail()

    # states

    def state_idle(self):
        if self.start:
            self.state = State.APPROACH_AUTO
            self.start = False
        else:
            rospy.sleep(0.5)

    def state_approach_wait(self):
        pass

    def state_approach_auto(self):
        self.speak.request('I am searching for a person!')

        tries = 0
        success = False

        for position in ICRA_POSITIONS:
            self.move_to_position.request(position)
            while (tries < MAX_PERSON_SEARCH_TRIES):
                success = self.object_to_transform.check_for_object('person')
                if success:
                    break
                self.base.go_rel(yaw=1.0)
                self.body.move_to_joint_positions({'head_tilt_joint': 0.0})
                tries = tries + 1

        if not success:
            self.state = State.FAIL
            return

        self.logger.log('Found a person.')
        self.speak.request('Oh, there you are!')

        self.state = State.IDLE
        # self.state = State.OFFER_BRING

    def state_offer_bring(self):
        self.speak.request('Sorry to interrupt, but I was wondering if there is anything I can bring you at the moment?')
        
        self.intent = None
        success = self.service_listen_and_wait('Sorry, I did not understand. Is there anything I can get you?', ['intent_item'])

        if not success:
            self.state = State.FAIL
            return

        item = self.intent.args[0]
        say = 'Ok, I can bring you the ' + item + '. Where can I find that?'
        self.speak.request(say)

        self.intent = None
        success = self.service_listen_and_wait('Sorry, I did not understand. Where is the item located', ['intent_location'])

        if not success:
            self.state = State.FAIL
            return

        location = self.intent.args[0]
        say = 'Ok, I will fetch you the ' + item + ' from the ' + location + '.'
        self.speak.request(say)
        
        self.state = State.IDLE

    def state_fail(self):
        self.speak.request('Sorry, I was not able to understand what you wanted. I will now reset.')
        self.state = State.IDLE

    # service calls

    def service_listen_and_wait(self, repeat_message, valid_intents):
        msg = String()
        msg.data = ''

        valid = False
        tries = 0
        while (tries < MAX_USER_INPUT_RETRIES) and (not valid):
            if tries > 0:
                self.speak.request(repeat_message)

            self.ros_pub_service_listen_once.publish(msg)

            wait = 0
            response = False
            while (wait < MAX_USER_INPUT_WAIT) and (not response):
                self.logger.log('Waiting for user input...')

                if self.intent != None:
                    response = True

                rospy.sleep(1)
                wait = wait + 1

            if self.intent != None:
                if self.intent.intent in valid_intents:
                    valid = True

            tries = tries + 1

        return valid

    # callbacks

    def ros_callback_start_signal(self, msg):
        self.start = True

    def ros_callback_intent_bus(self, msg):
        self.intent = msg
        log = 'Received intent: ' + msg.intent
        self.logger.log(log)

if __name__ == '__main__':
    m = Main()
