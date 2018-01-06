#!/usr/bin/env python3

# This is my first ever python program that does something!! 
# Have the calibration file inside stereo0 or stereo1. Run this command and pass in
# the location of that calibration file. Thats it.

import fileinput
import shutil
import sys
import os,os.path
from subprocess import call

# Do an arguments check
if len(sys.argv) != 2:
    print("Invalid entry. Must specify calibration tar.gz file")
    sys.exit()


packed_cal = str(sys.argv[1])

# Get the current working directory from where the script was called
curWorkingDir = os.path.abspath(os.getcwd())

# Get the directory of the calibration file 
calPath = os.path.join(curWorkingDir,packed_cal)
calDir = os.path.abspath(os.path.join(calPath, os.pardir))

# Get the parent directory, which is either stereo0 or stereo1
calParentDir = os.path.abspath(os.path.join(calPath, os.pardir))
camera = str(os.path.split(calParentDir)[1])

# Now we have the absolute directory of the calibration, the name of the parent
# directory, which tells us which stereo camera we have along with this directory's
# path, which is where the yaml files are going to go

if camera not in ["stereo0", "stereo1", "stereo2"]:
    print("You must run this file in the archive folder of the stereo camera you are trying to calibrate")
    sys.exit()

print("====================================================")
print("== RiverRun ========================================")
print("====================================================")
print("Sammut Tech LLC Stereo Calibration Unpacking Utility")
print("----------------------------------------------------")
print("Calibration for: " + camera)

# unpack the files
call(['tar','-xvz', '-C', calDir, '-f', calPath, "left.yaml","right.yaml"])

leftPath = os.path.join(calDir,"left.yaml")
rightPath = os.path.join(calDir,"right.yaml")

# rename the left.yaml link from narrow_stereo/left to "camera"+_link
with fileinput.FileInput(leftPath, inplace=True ) as file:
    for line in file:
        print(line.replace("narrow_stereo/left", camera+"_link"), end='')

# rename the right.yaml link from narrow_stereo/right to "camera"+_right_link
with fileinput.FileInput(rightPath, inplace=True ) as file:
    for line in file:
        print(line.replace("narrow_stereo/right", camera+"_right_link"), end='')

# This is optional, and is only here if I ever decide to have the calibrations in a
# different directory.
# Now lets move the files!
shutil.move(leftPath, os.path.join(calParentDir, "left.yaml"))
shutil.move(rightPath, os.path.join(calParentDir, "right.yaml"))

print("All done. " + camera + " is set up AF!")
