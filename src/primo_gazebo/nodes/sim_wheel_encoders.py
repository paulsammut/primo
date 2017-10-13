#!/usr/bin/env python

# This node is to simulate the encoder data coming from the motors. I use the wheel
# angle data coming from gazebo and convert it to encoder values. The reason I built
# this simulator is so I can test the coordinate transform wheel odom.
import rospy

import math
import tf2_ros
import tf
import geometry_msgs.msg
from std_msgs.msg import Float32

if __name__ == '__main__':
    rospy.init_node('sim_wheel_encoder')

    pub_lwa = rospy.Publisher('left_wheel_a', Float32, queue_size=10)
    pub_lwa_r = rospy.Publisher('left_wheel_a_roll', Float32, queue_size=10)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # Calculate the wheel conversion
    ticks_meter = rospy.get_param('ticks_meter', 69000)

    rate = rospy.Rate(30.0)

    # Previous left wheel angle
    prev_lwa = 0

    # Previous right wheel angle
    prev_rwa = 0

    # Direction of wheel. 1 = forward, -1 = backward
    dir_lw = 1
    dir_rw = 1

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

        # left wheel angle
        lwa = euler_left_wheel[1]
        lwa_roll = euler_left_wheel[0]

        # right wheel angle
        rwa = euler_right_wheel[1]

        pub_lwa.publish(lwa)
        pub_lwa_r.publish(lwa_roll)

        dl = lwa - prev_rwa 

        # Figure out the direction of the wheel
        if lwa_roll < 0.01 and lwa_roll > -0.01:
            rospy.loginfo("stable")
            if dl > 0:
                dir_lw = 1
            else:
                dir_lw = -1
        else:
            rospy.loginfo("Floppy")
            if dl > 0:
                dir_lw = -1
            else:
                dir_lw = 1

        # rospy.loginfo("DIrection is: %d" %dir_lw )
        # Ok so now I have the angle of the left and right wheel, problem is that I
        # have to make it a continuous angle.

        rate.sleep()
