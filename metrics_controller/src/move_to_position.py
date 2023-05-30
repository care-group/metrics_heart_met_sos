#! /usr/bin/env python3

# standard libraries
from hsrb_interface import Robot
import rospy
import actionlib
import tf

# internal classes
from log import Log

# standard messages
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# custom messages
# none

# constants and parameters
ICRA_MAP = {
    'view_reading_room' : (2.24, -1.94, 0.0),
    'view_hall_room': (0.0, -3.14, 0.0),
    'view_dining_room': (1.34, -4.04, 0.0),
    'view_living_room': (2.38, -4.88, 0.0),
    'pick_reading_table' : (0.0, 0.0, 0.0),
    'pick_dining_table' : (0.0, 0.0, 0.0),
    'pick_hall_table' : (0.0, 0.0, 0.0),
    'pick_bookshelf' : (0.0, 0.0, 0.0),
}

class MoveToPosition():
    def __init__(self, body):
        self.id = 'move_to_position'
        self.logger = Log(self.id)
        self.logger.startup_msg()

        # set up ROS
        self.ros_sub_move_to_room = rospy.Subscriber('/metrics_positions/request', String, callback=self.ros_callback_move_to_room)
        self.ros_ac_move_base = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
        self.ros_ac_move_base.wait_for_server()

        # set up classes
        # none

        # instance variables
        self.body = body
        self.current_room = None

        # ready
        self.logger.log_great('Ready.')

    # core logic

    def request(self, room):
        if room == self.current_room:
            return True

        log = 'Attempting to move to:' + room
        self.logger.log(log)

        self.body.move_to_go()

        destination = ''
        try:
            destination = ICRA_MAP[room]
        except:
            log = 'Invalid location: ' + room
            self.logger.log_warn(log)
            return False

        goal_x = destination[0]
        goal_y = destination[1]
        goal_r = destination[2]

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.position = Point(goal_x, goal_y, 0)
        quat = tf.transformations.quaternion_from_euler(0, 0, goal_r)
        pose.pose.orientation = Quaternion(*quat)

        goal = MoveBaseGoal()
        goal.target_pose = pose

        self.ros_ac_move_base.send_goal(goal)

        self.ros_ac_move_base.wait_for_result(rospy.Duration(60))

        action_state = self.ros_ac_move_base.get_state()
        if action_state == GoalStatus.SUCCEEDED:
            self.logger.log_great('Action completed successfully.')
            return True
        else:
            self.logger.log_warn('Action failed to complete. Ensure path to location is not obstructed.')
            return False

    # callbacks

    def ros_callback_move_to_room(self, msg):
        room = msg.data
        room = room.lower()
        self.request(room)