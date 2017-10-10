#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "primo_diff_tf");
    ros::NodeHandle node;

    tf2_ros::TransformBroadcaster odom_braodcaster;


    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
} 
