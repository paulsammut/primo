#include <ros/ros.h>
#include "stereo_mask/stereo_mask.h"

int main (int argc, char **argv) 
{
    ros::init(argc, argv, "stereo_mask");

    stereo_mask::StereoMask stereoMask(ros::NodeHandle());

    ros::spin();
    return 0;
}
