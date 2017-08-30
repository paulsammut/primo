#include <ros.h>
#include <sensor_msgs/BatteryState.h>

#define PIN_VOLTAGE A0
#define PIN_CURRENT A1

ros::NodeHandle nh;

unsigned long timePub = 0;

sensor_msgs::BatteryState msg_batt;

ros::Publisher pub_battState("battery", &msg_batt);


void setup() 
{
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(pub_battState);
}

void loop() 
{
    if(millis() > timePub)
    {
       
    msg_batt.voltage = analogRead(PIN_VOLTAGE);
    msg_batt.current = analogRead(PIN_CURRENT);
    pub_battState.publish(&msg_batt);
    nh.spinOnce();
    }
}

