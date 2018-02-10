#include <ros.h>
#include <sensor_msgs/BatteryState.h>
#include <ros/time.h>

#define PIN_VOLTAGE A0
#define PIN_CURRENT A1
#define PIN_LED_GREEN 2
#define PIN_LED_RED 3
#define PIN_SWITCH 4

ros::NodeHandle nh;

unsigned long timePub = 0;

// Interval in milliseconds for publishing the battery data
int pubInterval = 50;
float voltLow = 19.0;

sensor_msgs::BatteryState msg_batt;

ros::Publisher pub_battState("battery", &msg_batt);


void setup()
{
    // Set the pin modes
    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_SWITCH, OUTPUT);
    
    // Set everything low to be nice and safe
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_SWITCH, LOW);

    // Do an initial protection check
    protectionCheck();

    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(pub_battState);
}

void loop()
{
    if(millis() > timePub)
    {
        msg_batt.voltage = readVoltage();
        msg_batt.current = readCurrent();


        msg_batt.header.stamp = nh.now();

        pub_battState.publish(&msg_batt);
        nh.spinOnce();
        timePub = pubInterval + millis();

        protectionCheck();
    }
}

float readVoltage(void)
{
    // This is from the conversion of the power sensor
    // Volt 36.60mV / Amp
    float retVolts = (((float)analogRead(PIN_VOLTAGE))/1024.0*5.0)/0.06369;
    // This is my own calibration offsets
    retVolts += -0.48;
    /* retVolts += -1.48; */
    return(retVolts);
}

float readCurrent(void)
{
    // 50V/90A = 63.69mV 
    float retCurrent = (((float)analogRead(PIN_CURRENT))/1024.0*5.0)/0.03660;
    return retCurrent;
}

bool protectionCheck(void)
{
    // System is down
    if(readVoltage() <= voltLow)
    {
        digitalWrite(PIN_LED_RED, HIGH);
        digitalWrite(PIN_LED_GREEN, LOW);

        // Wait 5 minutes
        // delay(300000);

        digitalWrite(PIN_SWITCH, LOW);
        return true;
    } 

    // System is good
    else 
    {
        digitalWrite(PIN_SWITCH, HIGH);
        digitalWrite(PIN_LED_RED, LOW);
        digitalWrite(PIN_LED_GREEN, HIGH);
        return false;
    }
}
