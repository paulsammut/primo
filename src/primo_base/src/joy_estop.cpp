#include <ros/ros.h>
#include "sabertooth_simple/SabertoothEstop.h"

int main(int argc, char **argv)
{
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<sabertooth_simple::SabertoothEstop>("motor_estop");

    sabertooth_simple::SabertoothEstop srv;

    return 1;
}
