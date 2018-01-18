#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace stereo_mask
{
class StereoMask
{
public:
    /**
     * @brief constructor that takes in the nodehandle.
     *
     * @param nh
     */
    StereoMask(ros::NodeHandle _nh);

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
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Publisher pubLeft;
    image_transport::Publisher pubRight;
    image_transport::Subscriber subLeft;
    image_transport::Subscriber subRight;
};
}
