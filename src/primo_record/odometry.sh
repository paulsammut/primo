#!/bin/bash

# Mapping recording script

echo "Recording outing"

rosbag record \
    /odometry/filtered \
    /wheel/odom \
    /stereo_odom/odom \
    /imu \
    /battery \
    /tf \
    /tf_static 
