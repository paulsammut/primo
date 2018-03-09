#!/bin/bash 

# This script filters out all tfs except for the map to odom and odom to
# base_link transforms.

USAGE="Usage: filter_robo_tf.sh rosbag_in.bag rosbag_out.bag"

if [ $# != 2 ] ; then
    echo $USAGE
    EXIT 1;
fi

rosbag filter $1 $2 "topic != '/tf' or m.transforms[0].child_frame_id == 'base_link' or m.transforms[0].child_frame_id == 'odom'"
