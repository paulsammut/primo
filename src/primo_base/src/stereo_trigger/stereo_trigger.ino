#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <ros/time.h>
#include <stdio.h>
#include <TimerOne.h>

#define PIN_TRIGGER 2
#define TRIGGER_HOLD_US 1500

ros::NodeHandle nh;

// Rate in Hz at which to trigger the cameras
float rate = 10;
unsigned long period_us;
bool pulse_state = false;

void calcPeriod() { period_us = 1 / rate * 1000000; }

void stopMsgCb( const std_msgs::Empty& empty_msg){
    Timer1.stop();
    // Set the line to idle
    digitalWrite(PIN_TRIGGER, LOW);
    digitalWrite(13, LOW);
    pulse_state = false;
}

void startMsgCb( const std_msgs::Empty& empty_msg){
    Timer1.attachInterrupt(pulse);
    Timer1.initialize(period_us);
    digitalWrite(13, HIGH);
    pulse_state = true;
}


/**
 * @brief Callback function for the rate message. This sets the rate in hertz,
 * calculates the period and sets up our timer based pulse generator.
 *
 * @param rate_msg
 */
void rateMsgCb( const std_msgs::Float32& rate_msg){
    digitalWrite(13, LOW);
    Timer1.stop();
    rate = rate_msg.data;
    calcPeriod();

    // Only restart the pulse train if it was started before
    if(pulse_state)
    {
        Timer1.attachInterrupt(pulse);
        Timer1.initialize(period_us);
        digitalWrite(13, HIGH);
    }
}


/**
 * @brief Sets up our node to subscribe to settings and runs the pulse train
 */
void setup()
{
    // Set the pin modes
    pinMode(PIN_TRIGGER, OUTPUT);
    pinMode(13, OUTPUT);    
    
    // Set the trigger low.
    digitalWrite(PIN_TRIGGER, LOW);

    // Subscribe to the message
    ros::Subscriber<std_msgs::Empty> sub1("~stop", stopMsgCb );
    ros::Subscriber<std_msgs::Empty> sub2("~start", startMsgCb );
    ros::Subscriber<std_msgs::Float32> sub3("~rate", rateMsgCb );

    // ROS initialize
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(sub1);
    nh.subscribe(sub2);
    nh.subscribe(sub3);

    // Calculate the period
    calcPeriod();

    // The timer that will handle the pulsing
    Timer1.initialize(period_us);
    Timer1.attachInterrupt(pulse);
    pulse_state = true;
}

void loop()
{
    nh.spinOnce();
}

/**
 * @brief Pushes a pulse out that triggers a frame capture on the stereo cameras
 */
void pulse()
{
    // set the trigger high
    digitalWrite(13, HIGH);
    digitalWrite(PIN_TRIGGER, HIGH);
    delayMicroseconds(TRIGGER_HOLD_US);
    digitalWrite(PIN_TRIGGER, LOW);
    digitalWrite(13, LOW);
}
