/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <sstream>
#include "../include/primo_dash/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace primo_dash {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

void QNode::ros_comms_init() {
	ros::NodeHandle n;

    // Here are the exposure and brightness settings
	pub_s0_expo             = n.advertise<std_msgs::Float64>("/stereo0/set_exposure", 1000);
    pub_s0_bright           = n.advertise<std_msgs::Float64>("/stereo0/set_brightness", 1000);
    pub_s1_expo             = n.advertise<std_msgs::Float64>("/stereo1/set_exposure", 1000);
    pub_s1_bright           = n.advertise<std_msgs::Float64>("/stereo1/set_brightness", 1000);
    pub_s2_left_expo        = n.advertise<std_msgs::Float64>("/stereo2/set_exposure_left", 1000);
    pub_s2_left_bright      = n.advertise<std_msgs::Float64>("/stereo2/set_brightness_left", 1000);
    pub_s2_right_expo       = n.advertise<std_msgs::Float64>("/stereo2/set_exposure_right", 1000);
    pub_s2_right_bright     = n.advertise<std_msgs::Float64>("/stereo2/set_brightness_right", 1000);
    pub_s2_trig     		= n.advertise<std_msgs::Float32>("/stereo_trigger/rate", 1000);

}

bool QNode::init() {
    ros::init(init_argc,init_argv,"primo_dash", ros::init_options::AnonymousName);
	if ( ! ros::master::check() ) {
		return false;
	}

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros_comms_init();
    start();

	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
    ros::init(remappings,"primo_dash", ros::init_options::AnonymousName);
	if ( ! ros::master::check() ) {
		return false;
	}

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros_comms_init();
    start();

	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;

    // Just run ros::spin and shutdown when it gets the shutdown signal
    ros::spin();

	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::pubCamSetting(CamSetting camSetting, double num2Send)
{
    // Go through all this crap 
    std_msgs::Float64 msg;
    std_msgs::Float32 msg32;
    msg.data = num2Send;
    msg32.data = num2Send;

    switch(camSetting) {
        case s0_expo:           pub_s0_expo.publish(msg); break;
        case s0_bright:         pub_s0_bright.publish(msg); break;
        case s1_expo:           pub_s1_expo.publish(msg); break;
        case s1_bright:         pub_s1_bright.publish(msg); break;
        case s2_left_expo:      pub_s2_left_expo.publish(msg); break;
        case s2_left_bright:    pub_s2_left_bright.publish(msg); break;
        case s2_right_expo:     pub_s2_right_expo.publish(msg); break;
        case s2_right_bright:   pub_s2_right_bright.publish(msg); break;
        case s2_trig:   		pub_s2_trig.publish(msg32); break;
    }
}

}  // namespace primo_dash
