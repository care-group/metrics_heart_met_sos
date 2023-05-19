#!/usr/bin/env python3

# standard libraries
import rospy
import smach
from enum import Enum

# internal classes
from log import Log
from speak import Speak

# standard messages
from std_msgs.msg import String
from hsrb_interface import Robot, exceptions

# custom messages
# none

# constants and parameters
# none

class State(Enum):
    IDLE = 0
    APPROACH_WAIT = 1
    APPROACH_AUTO = 2
    OFFER_BRING = 3

class Main():
    def __init__(self):
        # set up logger
        self.id = 'main'
        self.logger = Log(self.id)
        self.logger.startup_msg()

        # set up ROS
        rospy.init_node('metrics_controller')

        self.ros_sub_start_signal = rospy.Subscriber('/metrics_controller/start_signal', String, callback=self.ros_callback_start_signal)
        self.ros_pub_service_listen_once = rospy.Publisher('/metrics_rasa/listen_once', String, queue_size=10)

        # set up classes
        self.speak = Speak()

        # set up HSR
        self.robot = Robot()
        try:
            self.logger.log('Waiting on robot resources...')
            self.base = self.robot.try_get('omni_base')
            self.body = self.robot.try_get('whole_body')
            self.grip = self.robot.try_get('gripper')
        except:
            self.logger.log_warn('Unable to get one or more handles for HSR resources. Another process may be using them or the robot is in an error state.')
            self.speak.request('Controller failed to start, please check console.')
            exit(0)

        # instance variables
        self.state = State.IDLE
        self.start = False

        # ready
        self.logger.log_great('Ready.')
        self.speak.request('Controller is ready.')

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
        # approach
        self.state = State.OFFER_BRING

    def state_offer_bring(self):
        self.speak.request('Sorry to interrupt, but I was wondering if there is anything I can bring you at the moment?')
        self.service_listen_once()
        self.state = State.IDLE

    # service calls
    def service_listen_once(self):
        msg = String()
        msg.data = ''
        self.ros_pub_service_listen_once.publish(msg)

    # callbacks

    def ros_callback_start_signal(self, msg):
        self.start = True

if __name__ == '__main__':
    m = Main()
