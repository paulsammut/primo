#!/bin/bash

# Mapping recording script

rosbag record --duration 1m\
    /odometry/filtered \
    /stereo2/left/image_rect_color \
    /stereo2/depth \
    /stereo2/left/image_rect \
    /stereo2/left/camera_info \
    /tf \
    /tf_static
