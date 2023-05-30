#!/usr/bin/env python3

# standard libraries
import rospy
import sys
import usb.core
import usb.util

# internal classes
from log import Log
from tuning import Tuning

# standard messages
from std_msgs.msg import Int32

# custom messages
# none

# constants and parameters
# none

class Main():
    def __init__(self):
        # set up logger
        self.id = 'main'
        self.logger = Log(self.id)
        self.logger.startup_msg()

        # set up ROS
        rospy.init_node('metrics_speech_direction')

        self.ros_pub_direction_raw = rospy.Publisher('/metrics_speech/direction_raw', Int32, queue_size=10)

        # set up classes
        # none

        # set up HSR
        # none

        # instance variables
        try:
            self.device = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        except:
            self.logger.log_warn('Could not connect to microphone, ensure device is attached.')
            sys.exit(0)

        self.mic = Tuning(self.device)

        # ready
        self.logger.log_great('Ready.')

        # loop
        self.run()

    def run(self):
        while not rospy.is_shutdown():
            direction = self.mic.direction

            log = 'Direction: ' + str(direction)
            self.logger.log(log)

            msg = Int32()
            msg.data = direction
            self.ros_pub_direction_raw.publish(direction)

            rospy.sleep(1)

if __name__ == '__main__':
    m = Main()