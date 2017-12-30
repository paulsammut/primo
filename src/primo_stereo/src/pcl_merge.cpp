#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include <string>

static sensor_msgs::PointCloud2 channel0_buffer, channel1_buffer;

static pcl::PCLPointCloud2 pclPCacumulada;


void chatterCallback( sensor_msgs::PointCloud2 PCmsg)
{
    // pcl::PCLPointCloud2 pclmsg;
    // pcl_conversions::toPCL(PCmsg,pclmsg);
    // pcl::concatenatePointCloud (pclPCacumulada, pclmsg, pclPCacumulada);
    // pcl_conversions::fromPCL(pclPCacumulada, PCacumulada);  
}

int main (int argc, char **argv)
{
    // topic names to subscribe to
    std::string channel0, channel1;

    ros::init(argc,argv,"pcl_merge"); 
    ros::NodeHandle ns; 
    ros::NodeHandle np; 

    // Get the parameters
    // ros::param

    // ros::Subscriber subscriptor = ns.subscribe("/stereo0/cbx/output",10, chatterCallback);
    ros::Publisher publicador= ns.advertise<sensor_msgs::PointCloud2>("velodyne_points2",10);
    ros::Rate loop_rate(10); 
    ros::spinOnce();
    while(ros::ok())
    {       
        // publicador.publish(PCacumulada);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Fin suscriptor");
    return 0;
}
