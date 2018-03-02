#!/usr/bin/env python  
import roslib
import rospy
import math
import tf

if __name__ == '__main__':
    rospy.init_node('align_save')

    listener = tf.TransformListener()

    # create out alignment dictionary
    alignment = {}

    while not rospy.is_shutdown():
        # Go through each of our cameras and save the current transform to our
        # alignment yaml

        try:
            (trans,rot) = listener.lookupTransform( '/chassis_link','/stereo0_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("TF Exception. Not saving shit.")
            exit()

        rot = tf.transformations.euler_from_quaternion(rot)

        alignment['a_s0_x']     = trans[0]
        alignment['a_s0_y']     = trans[1]
        alignment['a_s0_z']     = trans[2]
        alignment['a_s0_roll']  = rot[0]
        alignment['a_s0_pitch'] = rot[1]
        alignment['a_s0_yaw']   = rot[2]

        rospy.loginfo(alignment)


        exit()
