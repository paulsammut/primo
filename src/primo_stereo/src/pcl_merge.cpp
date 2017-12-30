#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"

static sensor_msgs::PointCloud2 PCacumulada;  

static pcl::PCLPointCloud2 pclPCacumulada;


void chatterCallback( sensor_msgs::PointCloud2 PCmsg)
{
    pcl::PCLPointCloud2 pclmsg;
    pcl_conversions::toPCL(PCmsg,pclmsg);
    pcl::concatenatePointCloud (pclPCacumulada, pclmsg, pclPCacumulada);
    pcl_conversions::fromPCL(pclPCacumulada, PCacumulada);  
}

int main (int argc, char **argv)
{
    ros::init(argc,argv,"nodeProcesador"); 
    ros::NodeHandle ns; 
    ros::NodeHandle np; 
    ros::Subscriber subscriptor = ns.subscribe("/stereo0/cbx/output",10, chatterCallback);
    ros::Publisher publicador= ns.advertise<sensor_msgs::PointCloud2>("velodyne_points2",10);
    ros::Rate loop_rate(10); 
    ros::spinOnce();
    while(ros::ok())
    {       
        publicador.publish(PCacumulada);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Fin suscriptor");
    return 0;
}
