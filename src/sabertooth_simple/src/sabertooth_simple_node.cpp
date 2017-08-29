#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "sabertooth_simple.h"

/**
 * This is the main function of the sabertooth_simple node that communicates to the a
 * sabertooth 2x** device via Plain Text Serial.
 */

/**
 * sabertooth_simple class
 * * device
 * * void sendM(1 or 2, int16 motor value)
 * * int getM(1 or 2, *int16 motor value)
 * * constructor
 *      - opens the serial port at 9600 baud
 *      - do an error if you don't find anything
 * * destructor
 *      - close the port
 *
 */

SabertoothSimple sabertooth;

void leftMotorCb(const std_msgs::Int16::ConstPtr& msg)
{
    // ROS_INFO("I heard: [%d] for the left motor!",msg->data);
    sabertooth.setM(1, msg->data);
}

void rightMotorCb(const std_msgs::Int16::ConstPtr& msg)
{
    // ROS_INFO("I heard: [%d] for the right motor!",msg->data);
    sabertooth.setM(2, msg->data);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv, "sabertooth_simple");
    
    ros::NodeHandle n;

    ros::Subscriber subLeft = n.subscribe("sabertooth/left",1000, leftMotorCb);
    ros::Subscriber subRight = n.subscribe("sabertooth/right",1000, rightMotorCb);

    sabertooth.connect();

    ros::spin();

    return 0;
}

