#include "stereo_mask/stereo_mask.h"

PLUGINLIB_EXPORT_CLASS(stereo_mask::StereoMask, nodelet::Nodelet)

namespace stereo_mask
{

StereoMask::StereoMask()
{
    // ================================
    // == Stereo Mask Init ============
    // ================================

    // Declare the points for us to fill
    cv::Point ptsLeft [1][4];
    cv::Point ptsRight[1][4];

    // Left Points
    ptsLeft[0][0] = cv::Point(   0, 479 );
    ptsLeft[0][1] = cv::Point( 400, 222 );
    ptsLeft[0][2] = cv::Point( 500, 300 );
    ptsLeft[0][3] = cv::Point( 700, 479 );

    // Right Points
    ptsRight[0][0] = cv::Point(   0, 479 );
    ptsRight[0][1] = cv::Point( 400, 222 );
    ptsRight[0][2] = cv::Point( 500, 300 );
    ptsRight[0][3] = cv::Point( 624, 479 );

    // Set up the points for the filly poly function
    pptLeft  =  ptsLeft[0]  ;
    pptRight =  ptsRight[0] ;

    npt[] = {4};
}

void StereoMask::onInit()
{
    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();

    // topics we will subscribe to
    std::string topicLeft, topicRight;

    // Get the parameters
    private_nh.param<std::string>("left_image_topic",   topicLeft,  "left/image_raw" );
    private_nh.param<std::string>("right_image_topic",  topicRight, "right/image_raw");

    // Debug info
    ROS_INFO("param left_image_topic with value:%s",    topicLeft.c_str());
    ROS_INFO("param right_image_topic with value:%s",   topicRight.c_str());

    // Handle the image transport subscribers and publishers
    subLeft     =   it.subscribe(topicLeft,   1, this->imageLeftCb);
    subRight    =   it.subscribe(topicRight,  1, this->imageRightCb);

    pubLeft = it.advertise("left/image_raw_mask", 1);
    pubRight = it.advertise("right/image_raw_mask", 1);
}



void StereoMask::imageLeftCb(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat image = cv_bridge::toCvShare(msg, "mono8")->image;
        // Ok cool, now we have an image, and now we need to draw on it!
        // Make the polygon

        cv::Point pts[1][4];
        pts[0][0] = cv::Point(   0, 479 );
        pts[0][1] = cv::Point( 400, 222 );
        pts[0][2] = cv::Point( 500, 300 );
        pts[0][3] = cv::Point( 700, 479 );

        // Set up the points for the filly poly function
        const cv::Point* ppt[1] = { pts[0] };
        int npt[] = {4};

        // boost::timer::cpu_timer timer1;
        cv::fillPoly(image, ppt, npt, 1, cv::Scalar(0, 0, 0), cv::LINE_8);
        // printf(timer1.format().c_str());

        // Convert to ros image while keeping the header
        sensor_msgs::ImagePtr msgOut = cv_bridge::CvImage(msg->header, "mono8", image).toImageMsg();

        // Publish the image
        pubLeft.publish(msgOut);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
    }
}

void StereoMask::imageRightCb(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat image = cv_bridge::toCvShare(msg, "mono8")->image;
        // Ok cool, now we have an image, and now we need to draw on it!
        // Make the polygon

        cv::Point pts[1][4];
        pts[0][0] = cv::Point(   0, 479 );
        pts[0][1] = cv::Point( 400, 222 );
        pts[0][2] = cv::Point( 500, 300 );
        pts[0][3] = cv::Point( 624, 479 );

        // Set up the points for the filly poly function
        const cv::Point* ppt[1] = { pts[0] };
        int npt[] = {4};

        // boost::timer::cpu_timer timer1;
        cv::fillPoly(image, ppt, npt, 1, cv::Scalar(0, 0, 0), cv::LINE_8);
        // printf(timer1.format().c_str());

        // Convert to ros image while keeping the header
        sensor_msgs::ImagePtr msgOut = cv_bridge::CvImage(msg->header, "mono8", image).toImageMsg();

        // Publish the image
        pubRight.publish(msgOut);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
    }
}
}

}

