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
from std_msgs.msg import Int32

class EncoderSim:
    previous_raw_angle = 0
    ticks_per_rad = 0
    encoder = 0

    def __init__(self, ticks_meter, wheel_radius):
        """ Initialization function that sets the ticks per radian value
        """
        self.ticks_per_rad = ticks_meter * wheel_radius

    def update(self, cur_raw_angle, raw_roll):
        
        delta_rads = cur_raw_angle - self.previous_raw_angle 

        self.previous_raw_angle = cur_raw_angle

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
        self.encoder += delta_rads * self.ticks_per_rad

        # Properly wrap the encoder to simulate a 16 bit encoder
        if self.encoder > 32767:
            self.encoder = -32768
        if self.encoder < -32768:
            self.encoder = 32767

        self.encoder = int(self.encoder)

        return self.encoder


if __name__ == '__main__':
    rospy.init_node('sim_wheel_encoder')

    pub_left = rospy.Publisher('encoder_left', Int32, queue_size=10)
    pub_right = rospy.Publisher('encoder_right', Int32, queue_size=10)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # Calculate the wheel conversion
    ticks_meter = rospy.get_param('ticks_meter', 69000)
    wheel_radius = rospy.get_param('wheel_radius', 0.151)

    left_enc_sim = EncoderSim(ticks_meter,wheel_radius) 
    right_enc_sim = EncoderSim(ticks_meter,wheel_radius)

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

        # Get the euler angles of the left and right wheels
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


        # Update the left encoder with the raw wheel angle and roll
        left_encoder = left_enc_sim.update(euler_left_wheel[1], euler_left_wheel[0])
        right_encoder = right_enc_sim.update(euler_right_wheel[1], euler_right_wheel[0])

        rospy.loginfo("Encoder Left: %d \t Right: %d" %(left_encoder, right_encoder))

        # Publisher the left and right encoder values
        pub_left.publish(left_encoder)
        pub_right.publish(right_encoder)

        rate.sleep()
