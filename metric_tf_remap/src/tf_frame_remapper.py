#!/usr/bin/env python
import rospy
import tf_conversions
import tf2_ros, tf2_geometry_msgs
import geometry_msgs.msg

class TFRemap:

    def __init__(self):

        print("Init")
        self.global_frame  = 'map'
        self.tf2_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf2_buffer =tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)

    
    # method to transform the relative position of the object to a "global" frame and broadcast it 
    def transformToMap(self, tf):
        # cast the tf to a stamped pose
        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose.position = tf.transform.translation
        pose_stamped.pose.orientation = tf.transform.rotation
        pose_stamped.header.frame_id = tf.header.frame_id
        pose_stamped.header.stamp = rospy.Time.now()

        # try to get the transform between the point and the global frame
        try:
            
            #TODO should have a check for if its possible to transform first
            # get the transform between the point and the global frame
            output_pose_stamped = self.tf2_buffer.transform(pose_stamped, self.global_frame, rospy.Duration(1))
            # create a new empty TF message
            tfn = geometry_msgs.msg.TransformStamped()

            # populate time and ID fields
            tfn.header.stamp = rospy.Time.now()
            tfn.header.frame_id = self.global_frame
            tfn.child_frame_id = "g_" + tf.child_frame_id

            #  copy the position info from the pose transformation
            tfn.transform.translation = output_pose_stamped.pose.position

            # set the angle of the TF
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
            tfn.transform.rotation.x = q[0]
            tfn.transform.rotation.y = q[1]
            tfn.transform.rotation.z = q[2]
            tfn.transform.rotation.w = q[3]
            
            # broadcast the TF 
            self.tf2_broadcaster.sendTransform(tfn)


        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Exception occured during TF Lookup")

    def getSourceTF(self, tf_frame, location):
            try:
                now = rospy.Time.now()
                tf = self.tf2_buffer.lookup_transform(tf_frame, location, now)
                return tf
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException):
                rospy.logwarn("Get TF did not work")
                return 0 
    
if __name__ == "__main__":
    rospy.init_node("TFremap")
    tm = TFRemap()

    rate = rospy.Rate(5)

    object = 'kinect_mount_link'
    frame = 'base_link'

    while not rospy.is_shutdown():
        try:
            
            t = tm.getSourceTF(frame,object) 
            tm.transformToMap(t)
                
        except rospy.ServiceException as exc:
            print("Something Fucked up" + str(exc))
        rate.sleep()
