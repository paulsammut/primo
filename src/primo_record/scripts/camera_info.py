#!/usr/bin/env python

# I grabbed this script off the internet to fix the color0 camera in the
# autodrive runs. In the original bag file there are camera_info message but
# they are empty. 

import rospy
import rosbag
import sys

topicCamInfo    = "/color0/camera_info"
topicImgRaw     = "/color0/image_raw"
newLink         = "color0_link"
newHeight       = 480
newWidth        = 864

if __name__ == '__main__':

    # Make sure we have the right number of arguments
    if len(sys.argv) == 3:
        print("Opening bag ..")
        with rosbag.Bag(sys.argv[2],'w') as outbag:

            print("Iterating through messages..")
            # Iterate through the messages
            for topic, msg, t in rosbag.Bag(sys.argv[1]).read_messages():

                # camera_info
                if topic == topicCamInfo:
                    # Lets rewrite the camere_info message
                    msg.header.frame_id = newLink
                    msg.height = newHeight
                    msg.width = newWidth

                    msg.distortion_model = "plumb_bob"
                    msg.D = [0.057168, -0.105575, 0.005278, -0.00021, 0.0]
                    msg.K = [667.992222, 0.0, 428.0704, 0.0, 670.288747,
                             254.611786, 0.0, 0.0, 1.0]
                    msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                    msg.P = [672.2948, 0.0, 427.771502, 0.0, 0.0, 675.529114,
                             256.500945, 0.0, 0.0, 0.0, 1.0, 0.0]

                    outbag.write(topic, msg, t)

                elif topic == topicImgRaw:
                    msg.header.frame_id = newLink
                    outbag.write(topic, msg, t)
                    
                else:
                    outbag.write(topic, msg, t)

    # Complain about not having the right amount of arguments
    else:
        print "usage: %s input output" % sys.argv[0]

# header:
#     seq: 1132
#     stamp:
#     secs: 0
#     nsecs:         0
#     frame_id: color0_link
# height: 480
# width: 864
# distortion_model: plumb_bob
# D: [0.057168, -0.105575, 0.005278, -0.00021, 0.0]
# K: [667.992222, 0.0, 428.0704, 0.0, 670.288747,
#         254.611786, 0.0, 0.0, 1.0]
# R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
# P: [672.2948, 0.0, 427.771502, 0.0, 0.0, 675.529114,
#         256.500945, 0.0, 0.0, 0.0, 1.0, 0.0]
# binning_x: 0
# binning_y: 0
# roi:
#     x_offset: 0
#     y_offset: 0
#     height: 0
#     width: 0
#     do_rectify: False
