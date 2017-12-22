#include <ros.h>
#include <std_msgs/Float32.h>
#include <ros/time.h>
#include <stdio.h>

#define PIN_TRIGGER 2

ros::NodeHandle nh;

// Interval in milliseconds for publishing the battery data
float rate = 20;
unsigned long period_ms = 1 / rate * 1000;
unsigned long timeLast = millis();

void setup()
{

    // Set the pin modes
    pinMode(PIN_TRIGGER, OUTPUT);
    
    // Set the trigger low. (active low)
    digitalWrite(PIN_TRIGGER, HIGH);

    // ROS initialize
    nh.getHardware()->setBaud(115200);
    nh.initNode();

    char info_msg[20] = "";
    sprintf(info_msg,"Stereo loaded with period %d ms", period_ms);

    // while(!nh.connected()) { nh.spinOnce();}
    // nh.loginfo(info_msg);
}

void loop()
{
    nh.spinOnce();

    if ((timeLast + period_ms) < millis())
    {
        timeLast = millis();
        pulse();
    }
}

void pulse()
{
    // set the trigger high
    digitalWrite(PIN_TRIGGER, LOW);
    delayMicroseconds(500);
    digitalWrite(PIN_TRIGGER, HIGH);
}
