#include <ros.h>
#include <std_msgs/Float32.h>
#include <ros/time.h>

#define PIN_TRIGGER 2

ros::NodeHandle nh;

// Interval in milliseconds for publishing the battery data
float rate = 10;
float period_ms = 1 / rate * 1000;
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
    nh.param("rate", rate, 10);
}

void loop()
{
    nh.spinOnce();

    if timeLast + period_ms > millis()
    {
        timeLast = millis();
        pulse();
    }
}

void pulse()
{
    // set the trigger high
    digitalWrite(PIN_TRIGGER, LOW);
    delayus(500);
    digitalWrite(PIN_TRIGGER, HIGH);
}
