#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import time
import cPickle as pickle
from shutil import copyfile
import datetime
import os
import rospkg

if __name__ == '__main__':
    rospy.init_node('align_save')

    listener = tf.TransformListener()

    # create out alignment dictionary
    alignment = {}

    # banner greetings
    rospy.loginfo("starting up Paul's awesome alignment save script!")

    # we gotta give our transform listener some time to collect transforms
    time.sleep(2)

    while not rospy.is_shutdown():
        # Go through each of our cameras and save the current transform to our
        # alignment yaml

        # stereo0
        try:
            (trans,rot) = listener.lookupTransform( '/chassis_link','/stereo0_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("TF exception on stereo0. Not saving shit")
            exit()

        rot = tf.transformations.euler_from_quaternion(rot)

        alignment['a_s0_x']     = trans[0]
        alignment['a_s0_y']     = trans[1]
        alignment['a_s0_z']     = trans[2]
        alignment['a_s0_roll']  = rot[0]
        alignment['a_s0_pitch'] = rot[1]
        alignment['a_s0_yaw']   = rot[2]

        rospy.loginfo("stereo0_link acquired...")

        # stereo1
        try:
            (trans,rot) = listener.lookupTransform( '/chassis_link','/stereo1_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("TF exception on stereo1. Not saving shit")
            exit()

        rot = tf.transformations.euler_from_quaternion(rot)

        alignment['a_s1_x']     = trans[0]
        alignment['a_s1_y']     = trans[1]
        alignment['a_s1_z']     = trans[2]
        alignment['a_s1_roll']  = rot[0]
        alignment['a_s1_pitch'] = rot[1]
        alignment['a_s1_yaw']   = rot[2]
        
        rospy.loginfo("stereo1_link acquired...")

        # stereo2
        try:
            (trans,rot) = listener.lookupTransform( '/chassis_link','/stereo2_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("TF exception on stereo2. Not saving shit")
            exit()

        rot = tf.transformations.euler_from_quaternion(rot)

        alignment['a_s2_x']     = trans[0]
        alignment['a_s2_y']     = trans[1]
        alignment['a_s2_z']     = trans[2]
        alignment['a_s2_roll']  = rot[0]
        alignment['a_s2_pitch'] = rot[1]
        alignment['a_s2_yaw']   = rot[2]

        rospy.loginfo("stereo2_link acquired...")
        
        # stereo3
        try:
            (trans,rot) = listener.lookupTransform( '/chassis_link','/stereo3_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("TF exception on stereo3. Not saving shit")
            exit()

        rot = tf.transformations.euler_from_quaternion(rot)

        alignment['a_s3_x']     = trans[0]
        alignment['a_s3_y']     = trans[1]
        alignment['a_s3_z']     = trans[2]
        alignment['a_s3_roll']  = rot[0]
        alignment['a_s3_pitch'] = rot[1]
        alignment['a_s3_yaw']   = rot[2]

        rospy.loginfo("stereo3_link acquired...")
        
        # color0
        try:
            (trans,rot) = listener.lookupTransform( '/chassis_link','/color0_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("TF exception on color0. Not saving shit")
            exit()

        rot = tf.transformations.euler_from_quaternion(rot)

        alignment['a_c0_x']     = trans[0]
        alignment['a_c0_y']     = trans[1]
        alignment['a_c0_z']     = trans[2]
        alignment['a_c0_roll']  = rot[0]
        alignment['a_c0_pitch'] = rot[1]
        alignment['a_c0_yaw']   = rot[2]

        rospy.loginfo("color0_link acquired...")

        # Done building our alignment dictionary
        #===========================================
        
        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()

        # get the file path for rospy_tutorials
        pack_dir = rospack.get_path('primo_description')
        rospy.loginfo(pack_dir)

        # Backup current alignment file
        rospy.loginfo("backing up current alignment file...")
        copyfile(pack_dir+'/config/align.yaml', pack_dir+'/config/align_orig.yaml')

        # Write our new alignment file
        align_file = open(pack_dir+'/config/align.yaml', 'w')
        align_file.write("# alignment file autogenerated by Paul's awesome script \n"+
                datetime.datetime.now().strftime("# %I:%M%p on %B %d, %Y")+
                "\n# ========================================================\n")


        for key, value in alignment.items():
            align_file.write(key+":  "+('%0.3f' % round(value,3))+"\n")

        rospy.loginfo("saved alignment yaml file. All done!")

        exit()
