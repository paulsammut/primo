#!/bin/sh

rosbag record   /stereo1/left/image_raw \
                /stereo1/left/camera_info \
                /stereo1/right/image_raw \
                /stereo1/right/camera_info \
