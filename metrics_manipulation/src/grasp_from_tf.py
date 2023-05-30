#!/usr/bin/env python
import rospy
import hsrb_interface
import gazebo_msgs.srv
import geometry_msgs.msg
import std_srvs.srv
import tf2_ros
import tf_conversions
from hsrb_interface import geometry
import tmc_interactive_grasp_planner.srv
from hsr_tools.srv import GraspFromTF, GraspFromTFResponse
import numpy as np


class GraspFromTFSrv():

    def __init__(self,gripper,omni_base,whole_body):
        
        # hsr control groups
        self.gripper = gripper
        self.omni_base = omni_base
        self.whole_body = whole_body

        # ROS service proxies
        self.gazebo_spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", gazebo_msgs.srv.SpawnModel)
        self.gazebo_delete_model = rospy.ServiceProxy("/gazebo/delete_model", gazebo_msgs.srv.DeleteModel)
        self.get_grasp_pattern = rospy.ServiceProxy("/get_grasp_pattern", tmc_interactive_grasp_planner.srv.GetGraspPattern)
        
        #tf2 i/o
        self.tf2_buffer =tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.tf2_broadcaster = tf2_ros.TransformBroadcaster()

        # ROS service decleration
        self.s = rospy.Service('grasp_from_tf', GraspFromTF, self.graspSequence)

        # variables
        self.object_pose = [None,None,None]
        self.goal_frame = 'odom'
        self.robot_frame = 'base_link'
        print("Service Ready")
        self.OPENGRIP = 1.2
        self.GRIPFORCE = 1
    
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
        self.whole_body.move_end_effector_pose(approach_pose, self.goal_frame )

        # Open gripper
        print("Open gripper")
        self.gripper.command(self.OPENGRIP)

        # Approach
        print("Approach")
        self.whole_body.move_end_effector_pose(grasp_pose, self.goal_frame )

        # Grasp
        print("Grasp")
        self.gripper.apply_force(self.GRIPFORCE)
        self.whole_body.move_to_neutral()

    # method to get the transform from one frame to another
    #TODO Requires some form of failure handling if no TF exists    
    def getObjectTF(self, object_tf, goal_frame):
        # if the transform exists
        # else we warn and return to TF for handling in called method
        if self.tf2_buffer.can_transform(goal_frame, object_tf, rospy.Time(), timeout=rospy.Duration(5)):
            # lookup the transform and return it
            tf = self.tf2_buffer.lookup_transform(goal_frame, object_tf, rospy.Time()) 
            return tf
        else:
            rospy.logwarn("COULD NOT GENERATE TF")
            return None

    # method to get the robot to face in direction of object TF
    def rotationAllign(self, object_tf, reference_tf):
        # move arm to neutral position
        self.whole_body.move_to_neutral()
        # get the tf between the reference_tf (hand_palm) and base_link and get the eular angles of it
        ref = self.getObjectTF(reference_tf, self.robot_frame)
        ref_angles = tf_conversions.transformations.euler_from_quaternion([ref.transform.rotation.x,ref.transform.rotation.y,ref.transform.rotation.z,ref.transform.rotation.w])
        self.whole_body.move_to_go()

        # get the transfrm between the object and base_link and get the eular angles of it
        obj = self.getObjectTF(object_tf, self.robot_frame)
        obj_angles = tf_conversions.transformations.euler_from_quaternion([obj.transform.rotation.x,obj.transform.rotation.y,obj.transform.rotation.z,obj.transform.rotation.w])

        # deciede on the rotation of spin of the robot (left or right)
        if obj.transform.translation.y<0:
            dir = 1
        else:
            dir = -1

        #if the object is behind us then we need to double the rotation
        if obj.transform.translation.x<0:
            dir = dir * 2

        # calculate how much the robot needs to rotate to face the object
        delta = np.asarray(ref_angles) - np.asarray(obj_angles)

        #perform the rotation allignment
        self.omni_base.go_rel(0,0,dir*delta[1]) 
    
    # method to select and allign the hsr to one of the object TF axis
    def axisAllign(self, object_tf):
        # get the transform betwee the base_link and the odometry tf
        ref = self.getObjectTF(self.robot_frame, self.goal_frame)
        base_map_angle = tf_conversions.transformations.euler_from_quaternion([ref.transform.rotation.x,ref.transform.rotation.y,ref.transform.rotation.z,ref.transform.rotation.w])
        z = base_map_angle[2]


        # set the angles in ranges to check the z angle lies within
        # forward and back are split into two to help with discontinuity checks as it passes x axis
        forward_l = [-0.785, 0] # x
        forward_r = [0, 0.785]
        back_l = [2.355, 3.14] 
        back_r = [-3.14, -2.355,] 
        left  = [0.785,2.355] # y
        right = [-2.355,-0.785]

        # combine the angle ranges into one array for processing
        dir = [forward_l, forward_r, back_l, back_r, left, right]
        # array to hold which axis the robot needs to allign with
        allign_axis = [0,0,0,0,0,0]
        j = 0

        # check if the given angle lies between the angles ranges
        # if it lies between one set that range to be array element to true
        for i in (dir):
            if z >=i[0] and z<=i[1]:
                allign_axis[j] = 1
            # increment counter
            j = j+1
        
        # set offest to be 1 unit away the tf
        offset = 1

        movement_pose =None
        # allign the robot based on the axis determined to do it with
        if(np.any(allign_axis[0:2])):
            print("FORWARD ALLIGN")
            movement_pose = geometry.pose(x=-offset)
            # self.omni_base.go_pose(geometry.pose(x=-offset),ref_frame_id=object_tf)
        elif(np.any(allign_axis[1:4])): 
            print("REAR ALLIGN")
            movement_pose = geometry.pose(x=offset, ek= 3.14)
            # self.omni_base.go_pose(geometry.pose(x=offset, ek= 3.14), ref_frame_id=object_tf)
        elif(allign_axis[4] ==1):
            print("LEFT ALLIGN")
            movement_pose = geometry.pose(y=-offset, ek= 1.57)
            # self.omni_base.go_pose(geometry.pose(y=-offset, ek= 1.57), ref_frame_id=object_tf)
        elif(allign_axis[5]==1):
            print("RIGHT ALLIGN")
            movement_pose = geometry.pose(y=offset, ek= -1.57)
            # self.omni_base.go_pose(geometry.pose(y=offset, ek= -1.57), ref_frame_id=object_tf)
        else:
            print("Could not pick axis to allign with")
        
        self.omni_base.go_pose(movement_pose, ref_frame_id=object_tf)

    #TODO tidy this chunk of code up
    def allignObject(self, object_tf, reference_tf):
        self.rotationAllign(object_tf, reference_tf)
        self.axisAllign(object_tf)


    # method to reset the simulation for another go around
    def resetSim(self):
        print("Resetting")
        # home the robot and open the gripper
        self.omni_base.go_abs(0,0,0)
        self.gripper.command(1.2)

        rospy.sleep(1)

        # delete the bottle model
        self.gazebo_delete_model("small_bottle")
        
        # close the gripper and return to the starting position
        self.gripper.command(0)
        self.whole_body.move_to_go()

    # method to handle the service request
    #TODO Return if the grasp was sucessful or not
    def graspSequence(self, req):
        tf = self.getObjectTF(req.tf_frame, self.goal_frame)
        self.object_pose[0] = tf.transform.translation.x
        self.object_pose[1] = tf.transform.translation.y
        self.object_pose[2] = tf.transform.translation.z

        # maybe change from something else so robot does not need to move hand out then in
        self.allignObject(req.tf_frame, 'hand_palm_link') 
        self.lookAtObject()

        tf = self.getObjectTF(req.tf_frame, self.goal_frame)
        self.object_pose[0] = tf.transform.translation.x
        self.object_pose[1] = tf.transform.translation.y
        self.object_pose[2] = tf.transform.translation.z

        pat = self.generateGraspPattens()
        app, gra = self.generateGraspPoses(pat)
        self.performGraspPattern(app,gra)
        self.resetSim()
        print("Done")

        return True

# main method
if __name__ == "__main__":
    # init node
    rospy.init_node("graspy")
    # start physics in gazebo sim. Not needed for real world tests
    rospy.wait_for_service("/gazebo/unpause_physics")
    unpause_physics = rospy.ServiceProxy("/gazebo/unpause_physics", std_srvs.srv.Empty)
    unpause_physics()

    # Prepare hsrb_interface and get ensure the robot is in position
    robot = hsrb_interface.Robot()
    wb = robot.get('whole_body')
    g = robot.get('gripper')
    ob = robot.get('omni_base')
    wb.move_to_go()

    # initialise the service call
    grasp = GraspFromTFSrv(g,ob,wb)

    # spin
    rospy.spin()
