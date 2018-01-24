#include "stereo_mask/stereo_mask_nodelet.h"

PLUGINLIB_EXPORT_CLASS(stereo_mask::StereoMaskNodelet, nodelet::Nodelet)

namespace stereo_mask
{

void StereoMaskNodelet::onInit()
{
    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();

    NODELET_INFO("Initializing stereo_mask nodelet");

    // initialize the image trasnport
    image_transport::ImageTransport it(nh);

    // topics we will subscribe to
    std::string topicLeft, topicRight;

    // Get the parameters
    private_nh.param<std::string>("left_image_topic",   topicLeft,  "left/image_raw" );
    private_nh.param<std::string>("right_image_topic",  topicRight, "right/image_raw");


    // Debug info
    NODELET_INFO("param left_image_topic with value:%s",    topicLeft.c_str());
    NODELET_INFO("param right_image_topic with value:%s",   topicRight.c_str());

    // Handle the image transport subscribers and publishers
    subLeft     =   it.subscribe(topicLeft,   1, & StereoMaskNodelet::imageLeftCb, this);    
    subRight    =   it.subscribe(topicRight,  1, & StereoMaskNodelet::imageRightCb, this);

    pubLeft = it.advertise("left/image_raw_mask", 1);
    pubRight = it.advertise("right/image_raw_mask", 1);
}



void StereoMaskNodelet::imageLeftCb(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat image = cv_bridge::toCvShare(msg, "mono8")->image;
        // Ok cool, now we have an image, and now we need to draw on it!
        // Make the polygon

        cv::Point pts[1][7];

        //Stereo1 - Left
        pts[0][0] = cv::Point( 328, 479 );
        pts[0][1] = cv::Point( 418, 399 );
        pts[0][2] = cv::Point( 422, 366 );
        pts[0][3] = cv::Point( 486, 325 );
        pts[0][4] = cv::Point( 548, 370 );
        pts[0][5] = cv::Point( 546, 404 );
        pts[0][6] = cv::Point( 630, 479 );

        //Stereo0 - Left
        pts[0][0] = cv::Point( 276, 479 );
        pts[0][1] = cv::Point( 371, 365 );
        pts[0][2] = cv::Point( 355, 324 );
        pts[0][3] = cv::Point( 436, 283 );
        pts[0][4] = cv::Point( 492, 332 );
        pts[0][5] = cv::Point( 493, 374 );
        pts[0][6] = cv::Point( 605, 477 );

        // Set up the points for the filly poly function
        const cv::Point* ppt[1] = { pts[0] };
        int npt[] = {7};

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
        NODELET_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
    }
}

void StereoMaskNodelet::imageRightCb(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat image = cv_bridge::toCvShare(msg, "mono8")->image;
        // Ok cool, now we have an image, and now we need to draw on it!
        // Make the polygon
        //
        cv::Point pts[1][7];
         
        //Stereo1 - Right
        pts[0][0] = cv::Point( 270, 479 );
        pts[0][1] = cv::Point( 364, 394 );
        pts[0][2] = cv::Point( 360, 354 );
        pts[0][3] = cv::Point( 429, 320 );
        pts[0][4] = cv::Point( 480, 355 );
        pts[0][5] = cv::Point( 476, 394 );
        pts[0][6] = cv::Point( 563, 479 );

        //Stereo0 - Right
        pts[0][0] = cv::Point( 208, 479 );
        pts[0][1] = cv::Point( 320, 350 );
        pts[0][2] = cv::Point( 311, 308 );
        pts[0][3] = cv::Point( 381, 273 );
        pts[0][4] = cv::Point( 429, 300 );
        pts[0][5] = cv::Point( 448, 361 );
        pts[0][6] = cv::Point( 576, 479 );

        // Set up the points for the filly poly function
        const cv::Point* ppt[1] = { pts[0] };
        int npt[] = {7};

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
        NODELET_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
    }
}
}
