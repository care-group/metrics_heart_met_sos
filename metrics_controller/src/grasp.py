#!/usr/bin/env python3
from hsrb_interface import Robot
import rospy
from hsrb_interface import geometry
import tmc_interactive_grasp_planner.srv
import geometry_msgs.msg


class Grasp():

    def __init__(self,gripper,omni_base,whole_body):
        self.get_grasp_pattern = rospy.ServiceProxy("/get_grasp_pattern", tmc_interactive_grasp_planner.srv.GetGraspPattern)

        # hsr control groups
        self.gripper = gripper
        self.omni_base = omni_base
        self.whole_body = whole_body
        self.object_pose = None
        self.goal_frame = 'map'

        self.OPENGRIP = 1.2
        self.GRIPFORCE = 1

    def grasp(self, point):

        try:
            self.object_pose = point
            self.whole_body.move_to_go()

            self.lookAtObject(point)

            pat = self.generateGraspPattens()

            app, gra = self.generateGraspPoses(pat)

            self.performGraspPattern(app,gra)
        except:
            print("Grasp Failed")
            return False
        
        return True

    # method to get the robot to look at a the object pose
    def lookAtObject(self):
        print("Looking at Object")
        v3_point = geometry.Vector3(x=self.object_pose[0], y =self.object_pose[1], z =self.object_pose[2])
        self.whole_body.gaze_point(point=v3_point, ref_frame_id='map')


        # method to generate the grasp patterns from the object pose
    def generateGraspPattens(self):
        print ("Generating Grasp Patterns")

        target_point = geometry_msgs.msg.PointStamped()
        target_point.header.stamp = rospy.Time.now()
        target_point.header.frame_id = "map"
        target_point.point.x = self.object_pose[0]
        target_point.point.y = self.object_pose[1]
        target_point.point.z = self.object_pose[2]

        patterns = self.get_grasp_pattern([target_point], 2, "", geometry_msgs.msg.Vector3(), [], False)

        # print(patterns)
        return patterns
    
    # method to calculate the positions the arm will move to for grasp operation
    def generateGraspPoses(self, patterns):
        print("Calculating Grasp Poses")

        # get the object pose from the pattern test
        object_pose = patterns.object_pose.pose

        # select a grasp pattern
        #TODO this is selecting the first some selection process should be done here to determine the best one
        grasp_pattern = patterns.grasp_patterns[0]
        
        # get the hand pose of the robot
        hand_pose = grasp_pattern.hand_frame
        # calculate the grasp pose of the robot
        grasp_pose= geometry.multiply_tuples(geometry.pose_to_tuples(object_pose),
                                            geometry.pose_to_tuples(hand_pose))
        
        # set the pre-grasp pose of the robot arm
        approach_distance = 0.1
        approach_pose = geometry.multiply_tuples(grasp_pose, geometry.pose(z=-approach_distance))

        # return the pre-grasp and grasp pose of the robot
        return approach_pose, grasp_pose

    # method to perform the grasp sequence along
    def performGraspPattern(self, approach_pose, grasp_pose):
        print("Performing Grasp Poses")
        # Move the end effector
        print("Picking up")
        self.whole_body.move_end_effector_pose(approach_pose, self.goal_frame)

        # Open gripper
        print("Open gripper")
        self.gripper.command(self.OPENGRIP)

        # Approach
        print("Approach")
        self.whole_body.move_end_effector_pose(grasp_pose, self.goal_frame)

        # Grasp
        print("Grasp")
        self.gripper.apply_force(self.GRIPFORCE)
        self.whole_body.move_to_neutral()
