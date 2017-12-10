#include <ros/ros.h>
#include "sabertooth_simple/SabertoothEstop.h"
#include <sensor_msgs/Joy.h>
// This is so we can make the robot make sound
#include "unistd.h"
#include "sound_play/sound_play.h"
#include <std_msgs/Empty.h>

// Sound client as a pointer so that it doesn't get initialized till later
sound_play::SoundClient *sc;
ros::ServiceClient client;
ros::Publisher waypoint_pub;
bool curState = false;


void joyCb(const sensor_msgs::Joy::ConstPtr& msg)
{
    sabertooth_simple::SabertoothEstop srv;
    bool estop_button = (msg->buttons[3] || msg->buttons[10] || msg->buttons[9]);
    
    if(estop_button)
    {
        srv.request.estop = true;
        if(client.call(srv))
        {
            // sc->say("Eeee STOP ACTIVATED. I aint goin NOWHERE");
            sc->say("ESPOT Activated");
            ROS_INFO("ESTOP succesfully TURNED ON"); 
        }
        else
        {
            sc->say("FAKKKKKK");
            ROS_ERROR("ESTOP FAILED"); 
        }
        curState = true;
    }
    // Emergency Stop clear button
    else if(msg->buttons[7] && curState)
    {
        srv.request.estop = false;
        if(client.call(srv))
        {
            sc->say("PRIMO TORQUED AND READY TO FUCK SHIT UP");
            ROS_INFO("ESTOP succesfully cleared"); 
            curState = false;
        }
        else
        {
            sc->say("SOMETHING FUCKED UP");
            ROS_ERROR("ESTOP CLEAR FAILED"); 
        }
    }
    // Waypoint follow button
    else if(msg->buttons[2])
    {
        std_msgs::Empty msg;
        sc->say("Starting mission. Let's do this!");
        ROS_INFO("Starting mission"); 
        waypoint_pub.publish(msg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_estop");

    ros::NodeHandle n;
    sc = new sound_play::SoundClient;
    client = n.serviceClient<sabertooth_simple::SabertoothEstop>("motor_estop");
    waypoint_pub = n.advertise<std_msgs::Empty>("/path_ready", 1000);

    ros::Subscriber joySub = n.subscribe("/joy",1000, joyCb);

    ros::spin();

    delete sc;

    return 1;
}
