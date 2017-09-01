#!/usr/bin/env python3
import fileinput

import sys
from subprocess import call

# print 'Number of arguments:', len(sys.argv), 'arguments.'
# print 'Argument List:', str(sys.argv)

#one is gonna be either stereo0 or stereo1
#two is gonna be the calibration

camera = str(sys.argv[1])
packed_cal = str(sys.argv[2])

# unpack the files
call(['tar','-xvzf', packed_cal, "left.yaml","right.yaml"])

# rename the left.yaml link from narrow_stereo/left to "camera"+_link

with fileinput.FileInput("left.yaml", inplace=True ) as file:
    for line in file:
                print(line.replace("narrow_stereo/left", camera+"_link"), end='')
