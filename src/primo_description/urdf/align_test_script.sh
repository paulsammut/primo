#!/bin/bash
rosrun xacro xacro --inorder primo_real.xacro > test.urdf \
    align_s0:=false \
    align_s1:=true \
    align_s2:=false \
    align_s3:=true \
    align_c0:=false \
    && urdf_to_graphiz test.urdf && evince primo.pdf
