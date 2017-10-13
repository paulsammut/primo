#!/usr/bin/env python

# This node is to simulate the encoder data coming from the motors. I use the wheel
# angle data coming from gazebo and convert it to encoder values. The reason I built
# this simulator is so I can test the coordinate transform wheel odom.
import rospy

import math
import tf2_ros
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('sim_wheel_encoder')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # Calculate the wheel conversion
    ticks_meter = rospy.get_param('ticks_meter', 69000)

    rate = rospy.Rate(30.0)

    while not rospy.is_shutdown():

        # Get the transform for the wheels
        try:
            trans_left_wheel  = tfBuffer.lookup_transform('chassis_link', 'left_wheel',
                    rospy.Time())
            trans_right_wheel  = tfBuffer.lookup_transform('chassis_link', 'right_wheel',
                    rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        euler_left_wheel = tf.transformations.euler_from_quaternion(
                [trans_left_wheel.transform.rotation.x, 
                trans_left_wheel.transform.rotation.y, 
                trans_left_wheel.transform.rotation.z, 
                trans_left_wheel.transform.rotation.w]) 
        
        euler_right_wheel = tf.transformations.euler_from_quaternion(
                [trans_right_wheel.transform.rotation.x, 
                trans_right_wheel.transform.rotation.y, 
                trans_right_wheel.transform.rotation.z, 
                trans_right_wheel.transform.rotation.w]) 
        
        rospy.loginfo("LEFT: %f \t RIGHT: %f" %(euler_left_wheel[1],
            euler_right_wheel[1]))
        rate.sleep()
