#!/usr/bin/env python
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
        try:
            trans  = tfBuffer.lookup_transform('chassis_link', 'left_wheel',
                    rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        euler = tf.transformations.euler_from_quaternion([trans.transform.rotation.x, 
            trans.transform.rotation.y, 
            trans.transform.rotation.z, 
            trans.transform.rotation.w]) 
        
        rospy.loginfo("The transform is:%f" %euler[1])
        rate.sleep()
