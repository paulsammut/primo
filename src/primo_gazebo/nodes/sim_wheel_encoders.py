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

class EncoderSim:
    previous_raw_angle = 0
    ticks_per_rad = 0
    encoder_value = 0

    def __init__(self, ticks_meter, wheel_radius):
        """ Initialization function that sets the ticks per radian value
        """
        self.ticks_per_rad = ticks_meter * wheel_radius

    def update(self, cur_raw_angle, raw_roll):
        
        delta_rads = cur_raw_angle - previous_raw_angle 

        previous_raw_angle = cur_raw_angle

        # rospy.loginfo("Delta is: %f \t Prev is: %f \t Current is : %f "%(dl,prev_lwa, lwa))

        # Figure out the direction of the wheel
        if raw_roll < 0.01 and raw_roll > -0.01:
            if delta_rads > 0:
                direction = 1
            else:
                direction = -1
        else:
            if delta_rads > 0:
                direction = -1
            else:
                direction = 1

        delta_rads = abs(delta_rads)*direction

        # Set the encoder value
        encoder += delta_rads * ticks_per_rad

        # Properly wrap the encoder to simulate a 16 bit encoder
        if encoder > 32767:
            encoder = -32768
        if encoder < -32768:
            encoder = 32767


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

    # Running totals
    left_angle = 0;
    right_angle = 0;


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

        dl = lwa - prev_lwa 

        # rospy.loginfo("Delta is: %f \t Prev is: %f \t Current is : %f "%(dl,prev_lwa, lwa))

        # Figure out the direction of the wheel
        if lwa_roll < 0.01 and lwa_roll > -0.01:
            if dl > 0:
                dir_lw = 1
            else:
                dir_lw = -1
        else:
            if dl > 0:
                dir_lw = -1
            else:
                dir_lw = 1

        dl = abs(dl)*dir_lw
        left_angle += dl


        prev_lwa = lwa

        rospy.loginfo("Left total angle: %f" %left_angle)
        # Ok so now I have the angle of the left and right wheel, problem is that I
        # have to make it a continuous angle.

        rate.sleep()
