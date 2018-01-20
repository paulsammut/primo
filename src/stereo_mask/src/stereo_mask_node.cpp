#include "ros/ros.h"
#include "nodelet/loader.h"

int main(int argc, char **argv)
{
    // This is boilerplate ros nodelet conversion code
    ros::init(argc,argv, "stereo_mask_node");
    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();
    nodelet.load(nodelet_name, "stereo_mask/stereo_mask_nodelet", remap, nargv);
    ros::spin();
    return 0;
}
