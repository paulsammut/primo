#include <ros/ros.h>
#include <tf2/transform_broadcaster.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "primo_diff_tf");
    ros::NodeHandle node;

    tf2::TransformBroadcaster odom_braodcaster;
    tf
