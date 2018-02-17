#!/bin/sh

rosbag record   \
                /stereo0/left/image_raw \
                /stereo0/left/image_rect \
                /stereo0/left/camera_info \
                /stereo1/left/image_raw \
                /stereo1/left/camera_info \
                /stereo1/left/image_rect \
                /stereo2/left/image_raw \
                /stereo2/left/camera_info \
                /stereo2/left/image_rect \
                /stereo2/left/image_rect_color \
                /stereo2/depth \
                /stereo3/left/image_raw_color \
                /stereo3/left/image_rect_color \
                /stereo3/left/camera_info \
                /stereo0/vxl/output \
                /stereo1/vxl/output \
                /stereo2/vxl/output \
                /stereo3/vxl/output \
                /odometry/filtered \
                /wheel/odom \
                /stereo_odom/odom \
                /imu \
                /battery \
                /tf \
                /tf_static 
                /odometry/filtered \
                /goal \
                /goal_node \
