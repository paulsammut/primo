#!/bin/sh

rosbag record   \
                /stereo0/left/image_raw \
                /stereo0/left/camera_info \
                /stereo0/right/image_raw \
                /stereo0/right/camera_info \
                /stereo1/left/image_raw \
                /stereo1/left/camera_info \
                /stereo1/right/image_raw \
                /stereo1/right/camera_info \
                /stereo2/left/image_raw \
                /stereo2/left/camera_info \
                /stereo2/right/image_raw \
                /stereo2/right/camera_info \
                /stereo3/left/image_raw_color
                /stereo3/left/camera_info
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
