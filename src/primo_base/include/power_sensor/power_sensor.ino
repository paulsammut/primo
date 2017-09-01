#include <ros.h>
#include <sensor_msgs/BatteryState.h>
#include <ros/time.h>

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
        // This is from the conversion of the power sensor
        // 50V/90A = 63.69mV / Volt 36.60mV / Amp
        // msg_batt.voltage = (1024.0/((float)analogRead(PIN_VOLTAGE)))*5.0;//0.06369;
        msg_batt.voltage = (((float)analogRead(PIN_VOLTAGE))/1024.0*5.0)/0.06369;
        msg_batt.current = (((float)analogRead(PIN_CURRENT))/1024.0*5.0)/0.03660;

        // This are my own calibration offsets
        msg_batt.voltage += -0.48;

        msg_batt.header.stamp = nh.now();

        pub_battState.publish(&msg_batt);
        nh.spinOnce();
        timePub = 50 + millis();
    }
}
