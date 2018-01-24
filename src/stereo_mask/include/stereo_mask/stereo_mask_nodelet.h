#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <pluginlib/class_list_macros.h>

namespace stereo_mask
{
class StereoMaskNodelet : public nodelet::Nodelet
{
public:
    
    enum Camera
    {
        stereo0,
        stereo1
    };

    /**
     * @brief Callback for receiving the image
     *
     * @param msg Image message to be passed to the callback
     */
    void imageLeftCb(const sensor_msgs::ImageConstPtr& msg);

    /**
     * @brief Callback for receiving the image
     *
     * @param msg Image message to be passed to the callback
     */
    void imageRightCb(const sensor_msgs::ImageConstPtr& msg);

private:
    virtual void onInit();

    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    image_transport::Publisher pubLeft;
    image_transport::Publisher pubRight;
    image_transport::Subscriber subLeft;
    image_transport::Subscriber subRight;

};
}
