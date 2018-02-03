#!/bin/bash

# Mapping recording script

rosbag record \
    /stereo2/left/image_raw \
    /stereo2/right/image_raw \
    /stereo2/left/camera_info \
    /stereo2/right/camera_info \
    /odometry/filtered \
    /tf_static \
    /tf

