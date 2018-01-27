#!/bin/bash

searchStr="e-con's 1MP Monochrome Camera"

# search for the cameras
if xinput list | grep -q "$searchStr"
then
    echo "Found the offending e-con camera(s)";
else
    echo "No e-con cameras found!";
    exit 0
fi


xinput list | grep "$searchStr" | while read -r line ; do
    var=${line#*=}
    id=$(awk '{print $1}' <<< $var)

    echo "Disabling $var"
    xinput --disable $id


done
