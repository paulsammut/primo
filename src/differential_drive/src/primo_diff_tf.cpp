#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "primo_diff_tf");
    ros::NodeHandle node;

    tf2_ros::TransformBroadcaster odom_braodcaster;

    ros::Rate loop_rate(10);
   
    ROS_INFO("Primo Diff TF running!");

    double rate;
    double tickst_meter;
    double base_width;

    std::string base_frame_id, odom_frame_id, wheel_frame_id;

    long encoder_min, encoder_max, encoder_low_wrap, encoder_high_wrap;


    while(ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
} 
