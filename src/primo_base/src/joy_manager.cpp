#include <ros/ros.h>
#include "sabertooth_simple/SabertoothEstop.h"
#include <sensor_msgs/Joy.h>
// This is so we can make the robot make sound
#include "unistd.h"
#include "sound_play/sound_play.h"
#include <std_msgs/Empty.h>

// Sound client as a pointer so that it doesn't get initialized till later
sound_play::SoundClient *sc;
// Service client that receive the estop requests
ros::ServiceClient client;
// Waypoing follower publisher so we can send a message to start the waypoints.
ros::Publisher waypoint_pub;

bool estopState = false;

void joyCb(const sensor_msgs::Joy::ConstPtr& msg)
{
    sabertooth_simple::SabertoothEstop srv;

    // Buttons
    bool btn_a = msg->buttons[0];
    bool btn_b = msg->buttons[1];
    bool btn_x = msg->buttons[2];
    bool btn_y = msg->buttons[3];
    bool btn_l_paddle = msg->buttons[4];
    bool btn_r_paddle = msg->buttons[5];
    bool btn_back = msg->buttons[6];
    bool btn_start = msg->buttons[7];
    bool btn_left_stick  = msg->buttons[9];
    bool btn_right_stick = msg->buttons[10];

    bool estop_button = (btn_left_stick || btn_right_stick);
    
    // Emergency Stop Activate
    if(estop_button)
    {
        // set our service object to be true so we engage the estop
        srv.request.estop = true;
        if(client.call(srv))
        {
            sc->say("Emergency stop activated");
            ROS_INFO("ESTOP succesfully TURNED ON"); 
        }
        else
        {
            sc->say("Emergency stop failed to activate");
            ROS_ERROR("ESTOP failed to activate"); 
        }
        estopState = true;
    }
    // Emergency Stop clear button
    else if(btn_start && estopState)
    {
        // set our service object to be false so we can clear the estop
        srv.request.estop = false;
        if(client.call(srv))
        {
            sc->say("Primo torqued and ready to fuck shit uup");
            ROS_INFO("ESTOP succesfully cleared"); 
            estopState = false;
        }
        else
        {
            sc->say("Emergency stop failed to clear");
            ROS_ERROR("ESTOP CLEAR FAILED"); 
        }
    }
    // Waypoint follow button
    else if(btn_x)
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

    // Cleanup the pointer 
    delete sc;

    return 1;
}
