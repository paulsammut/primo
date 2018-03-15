/**
 * @file /include/primo_dash/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef primo_dash_QNODE_HPP_
#define primo_dash_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/BatteryState.h>
#endif


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace primo_dash {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	enum CamSetting {
            s0_expo,
            s0_bright,
            s1_expo,
            s1_bright,
            s2_left_expo,
            s2_left_bright,
            s2_right_expo,
            s2_right_bright,
            s2_trig,
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

    /**
     * @brief Publish a float to a topic
     *
     * @param topic String of the topic to publish to  
     * @param num2Send the float that we will be sending over
     */
    void pubCamSetting(CamSetting camSetting, double num2Send);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void battUpdate(double bVoltage, double bCurrent);

private:
	int init_argc;
	char** init_argv;

    /**
     * @brief Initializes all the pubs and subs. Have it in one function to make
     * the two types of ros inits generic
     */
    void ros_comms_init();

    void batteryCb(const sensor_msgs::BatteryState::ConstPtr& msg);

    ros::Subscriber sub_batt;

    /**
     * @brief Here we have all our publishers for the camera settings
     */
	ros::Publisher pub_s0_expo,
                   pub_s0_bright,
                   pub_s1_expo,
                   pub_s1_bright,
                   pub_s2_left_expo,
                   pub_s2_left_bright,
                   pub_s2_right_expo,
                   pub_s2_right_bright,
                   pub_s2_trig;

    QStringListModel logging_model;
};

}  // namespace primo_dash

#endif /* primo_dash_QNODE_HPP_ */
