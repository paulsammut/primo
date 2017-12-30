#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include <string>

static sensor_msgs::PointCloud2 channel0_buffer, channel1_buffer;

static pcl::PCLPointCloud2 pclPCacumulada;


void channel0_Cb(sensor_msgs::PointCloud2 PCmsg)
{
    // pcl::PCLPointCloud2 pclmsg;
    // pcl_conversions::toPCL(PCmsg,pclmsg);
    // pcl::concatenatePointCloud (pclPCacumulada, pclmsg, pclPCacumulada);
    // pcl_conversions::fromPCL(pclPCacumulada, PCacumulada);  
}

void channel1_Cb(sensor_msgs::PointCloud2 PCmsg)
{
}

int main (int argc, char **argv)
{
    // topic names to subscribe to
    std::string channel0_topic, channel1_topic;

    double pub_rate;

    ros::init(argc,argv,"pcl_merge"); 
    ros::NodeHandle ns; 
    ros::NodeHandle np; 

    // Get the parameters
    ros::param::param<std::string>("channel0", channel0_topic, "channel0");
    ros::param::param<std::string>("channel1", channel1_topic, "channel1");
    ros::param::param<double>("pub_rate", pub_rate, 10);

    // Make the subscribers
    ros::Subscriber channel0_sub = ns.subscribe(channel0_topic,10, channel0_Cb);
    ros::Subscriber channel1_sub = ns.subscribe(channel1_topic,10, channel1_Cb);

    ros::Publisher pcl_pub = ns.advertise<sensor_msgs::PointCloud2>("output",10);

    ros::Rate loop_rate(pub_rate); 

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
