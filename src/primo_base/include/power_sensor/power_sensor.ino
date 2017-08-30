#include <ros.h>
#include <sensor_msgs/BatteryState.h>

#define PIN_VOLTAGE A0
#define PIN_CURRENT A1
ros::NodeHandle nh;

sensor_msgs::BatteryState msg_batt;

ros::Publisher pub_battState("battery", &msg_batt);


void setup() {
  msg_batt.voltage = 23;
}

void loop() {
  
}

