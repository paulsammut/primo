#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "uimu.h"
#include "sensor_msgs/Imu.h"
#include <sensor_msgs/MagneticField.h>

/**
 * This is the main function of the uimu_simple node that communicates to the a
 * uimu 2x** device via Plain Text Serial.
 */

/**
 * uimu_simple class
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

UimuClass uimu;

int main(int argc, char **argv)
{
    ros::init(argc,argv, "uimu");
    
    ros::NodeHandle n;

    uimu.imu_publisher = n.advertise<sensor_msgs::Imu>("imu",1000);
    uimu.mag_publisher = n.advertise<sensor_msgs::MagneticField>("mag",1000);

    uimu.connect();

    while(ros::ok())
    {
        uimu.readPort();
        ros::spinOnce();
    }

    return 0;
}

