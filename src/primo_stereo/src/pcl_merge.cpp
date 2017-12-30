#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include "tf/transform_datatypes.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <stdio.h>
#include <ctime>

static sensor_msgs::PointCloud2 channel0_buffer, channel1_buffer, output;

static pcl::PCLPointCloud2 channel0_temp, channel1_temp, output_temp;

void channel0_Cb(sensor_msgs::PointCloud2 PCmsg)
{
    channel0_buffer = PCmsg;
}

void channel1_Cb(sensor_msgs::PointCloud2 PCmsg)
{
    channel1_buffer = PCmsg;
}

int main (int argc, char **argv)
{
    // topic names to subscribe to
    std::string channel0_topic, 
                channel1_topic,
                channel0_frame,
                channel1_frame;

    double pub_rate;

    // ROS init stuff
    ros::init(argc,argv,"pcl_merge"); 
    ros::NodeHandle ns; 
    
    // create our transform listener
    tf::TransformListener listener;
    tf::StampedTransform transform;

    // Get the parameters
    ros::param::param<std::string>("channel0", channel0_topic, "/stereo0/cbx/output");
    ros::param::param<std::string>("channel1", channel1_topic, "/stereo1/cbx/output");
    ros::param::param<std::string>("channel0_frame", channel0_frame, "stereo0_link");
    ros::param::param<std::string>("channel1_frame", channel1_frame, "stereo1_link");
    ros::param::param<double>("pub_rate", pub_rate, 10);


    // Make the subscribers
    ros::Subscriber channel0_sub = ns.subscribe(channel0_topic,10, channel0_Cb);
    ros::Subscriber channel1_sub = ns.subscribe(channel1_topic,10, channel1_Cb);

    ros::Publisher pcl_pub = ns.advertise<sensor_msgs::PointCloud2>("output",10);

    ros::Rate loop_rate(pub_rate); 
    
    // Get our transform
    try{
        listener.lookupTransform("/stereo0_link", "/stereo1_link",  
                ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    ros::spinOnce();
    while(ros::ok())
    {       
        
        int start_s=clock();
        //convert channel1 to be in the channel0 frame
        pcl_ros::transformPointCloud("/stereo0_link", channel1_buffer, channel1_buffer, listener);
        // printf(timer1.format().c_str());
        int stop_s=clock();
        std::cout << "time: " << (stop_s-start_s)/double(CLOCKS_PER_SEC)*1000 << std::endl;

        // convert to PCL from sensor_msgs:PCL
        pcl_conversions::toPCL(channel0_buffer, channel0_temp);
        pcl_conversions::toPCL(channel1_buffer, channel1_temp);

        // Do the merge to the output variable
        pcl::concatenatePointCloud(channel0_temp, channel1_temp, output_temp);

        // convert back to sensor_msgs:PCL
        pcl_conversions::fromPCL(output_temp, output);

        // publish our merged point cloud!
        pcl_pub.publish(output);

        // Do the ROS
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Fin suscriptor");
    return 0;
}
