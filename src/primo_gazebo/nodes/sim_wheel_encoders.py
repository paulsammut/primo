#!/usr/bin/env python
import rospy

import math
import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('sim_wheel_encoder')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # Calculate the wheel conversion
    ticks_meter = rospy.get_param('ticks_meter', 69000))

    rate = rospy.Rate(50.0)

    while note rospy.is_shutdown():
        rate.sleep()
