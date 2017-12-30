#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include <string>

static sensor_msgs::PointCloud2 channel0_buffer, channel1_buffer, output;

static pcl::PCLPointCloud2 channel0_temp, channel1_temp, output_temp;


void channel0_Cb(sensor_msgs::PointCloud2 PCmsg)
{
    channel0_buffer = PCmsg;
    // pcl::PCLPointCloud2 pclmsg;
    // pcl_conversions::toPCL(PCmsg,pclmsg);
    // pcl::concatenatePointCloud (pclPCacumulada, pclmsg, pclPCacumulada);
    // pcl_conversions::fromPCL(pclPCacumulada, PCacumulada);  
}

void channel1_Cb(sensor_msgs::PointCloud2 PCmsg)
{
    channel1_buffer = PCmsg;
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
    ros::param::param<std::string>("channel0", channel0_topic, "/stereo0/cbx/output");
    ros::param::param<std::string>("channel1", channel1_topic, "/stereo1/cbx/output");
    ros::param::param<double>("pub_rate", pub_rate, 10);

    // Make the subscribers
    ros::Subscriber channel0_sub = ns.subscribe(channel0_topic,10, channel0_Cb);
    ros::Subscriber channel1_sub = ns.subscribe(channel1_topic,10, channel1_Cb);

    ros::Publisher pcl_pub = ns.advertise<sensor_msgs::PointCloud2>("output",10);

    ros::Rate loop_rate(pub_rate); 

    ros::spinOnce();
    while(ros::ok())
    {       
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
