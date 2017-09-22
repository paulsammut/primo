#!/usr/bin/env python3

# This is my first ever python program that does something!! 
# This project is meant to run inside the archive directory 
import fileinput

import shutil
import sys
import os,os.path
from subprocess import call

if len(sys.argv) != 2:
    print("Invalid entry. Must specify calibration tar.gz file")
    sys.exit()


packed_cal = str(sys.argv[1])

# get the parent directory
curfilePath = os.path.abspath(__file__)
curDir = os.path.abspath(os.path.join(curfilePath,os.pardir)) # this will return current directory in which python file resides.
print(curDir)
sys.exit()
parentDir = os.path.abspath(os.path.join(curDir,os.pardir)) # this will return parent directory.
camera = str(os.path.split(parentDir)[1])

print(camera)

if camera not in ["stereo0", "stereo1"]:
    print("You must run this file in the archive folder of the stereo camera you are trying to calibrate")
    sys.exit()


# unpack the files
call(['tar','-xvzf', packed_cal, "left.yaml","right.yaml"])

# rename the left.yaml link from narrow_stereo/left to "camera"+_link

with fileinput.FileInput("left.yaml", inplace=True ) as file:
    for line in file:
                print(line.replace("narrow_stereo/left", camera+"_link"), end='')

# Now lets move the files!
shutil.move(os.path.join(curDir, "left.yaml"), os.path.join(parentDir, "left.yaml"))
shutil.move(os.path.join(curDir, "right.yaml"), os.path.join(parentDir, "right.yaml"))

