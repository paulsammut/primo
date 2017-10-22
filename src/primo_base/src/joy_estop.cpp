#include <ros/ros.h>
#include "sabertooth_simple/SabertoothEstop.h"
#include <sensor_msgs/Joy.h>

ros::ServiceClient client;
sabertooth_simple::SabertoothEstop srv;


void joyCb(const sensor_msgs::Joy::ConstPtr& msg)
{
    bool estop_button = msg->buttons[3];
    
    if(estop_button)
    {
        srv.request.estop = true;
        if(client.call(srv))
            ROS_INFO("ESTOP succesfully TURNED ON"); 
        else
            ROS_INFO("ESTOP TURN ON FAILED"); 
    }
    else if(msg->buttons[2])
    {
        srv.request.estop = false;
        if(client.call(srv))
            ROS_INFO("ESTOP succesfully cleared"); 
        else
            ROS_INFO("ESTOP CLEAR FAILED"); 
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_estop");

    ros::NodeHandle n;
    client = n.serviceClient<sabertooth_simple::SabertoothEstop>("motor_estop");

    ros::Subscriber joySub = n.subscribe("/joy",1000, joyCb);

    ros::spin();

    return 1;
}
