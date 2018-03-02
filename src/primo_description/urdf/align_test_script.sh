#!/bin/bash
rosrun xacro xacro --inorder primo_real.xacro > test.urdf \
    align_s0:=true \
    align_s1:=true \
    align_s2:=true \
    align_s3:=true \
    align_c0:=true \
    && urdf_to_graphiz test.urdf && evince primo.pdf && \
    rm test.urdf primo.pdf primo.gv
