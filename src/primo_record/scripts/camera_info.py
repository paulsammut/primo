#!/usr/bin/env python

# I grabbed this script off the internet to fix the color0 camera in the
# autodrive runs. In the original bag file there are camera_info message but
# they are empty. 

import rospy
import rosbag
import fileinput
import os

new_link    = "color0_link"
new_height  = 480
new_width   = 864

def fix(inbags):
  for b in inbags:
    print "Trying to migrate file: %s"%b
    rebag = rosbag.Bag(outbag,'w')
    inbag = rosbag.Bag(b)
    try:
        for (topic, msg, t) in bag.read_messages():
            if msg._type == 'sensor_msgs/CameraInfo' or msg._type == 'sensor_msgs/Image':
              msg.header.frame_id = new_link
              msg.height = 480
              msg.width = 864
              rebag.write(topic, msg, t, raw=False)
            else:
              rebag.add(topic, msg, t, raw=False)
      rebag.close()
    except rosrecord.ROSRecordException, e:
      print " Migration failed: %s"%(e.message)
      os.remove(outbag)
      continue

    oldnamebase = b+'.old'
    oldname = oldnamebase
    i = 1
    while os.path.isfile(oldname):
      i=i+1
      oldname = oldnamebase + str(i)
    os.rename(b, oldname)
    os.rename(outbag, b)
    print " Migration successful.  Original stored as: %s"%oldname

if __name__ == '__main__':
  import sys
  if len(sys.argv) >= 2:
    fix(sys.argv[1:])
  else:
    print "usage: %s bag1 [bag2 bag3 ...]" % sys.argv[0]


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

