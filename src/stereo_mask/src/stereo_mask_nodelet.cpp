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
    std::string topicLeft, topicRight, camera_str;

    // Get the parameters
    private_nh.param<std::string>("left_image_topic",   topicLeft,  "left/image_raw" );
    private_nh.param<std::string>("right_image_topic",  topicRight, "right/image_raw");
    private_nh.param<std::string>("camera",             camera_str, "stereo0");

    // Set the right camera enum
    if(camera_str == "stereo0" || camera_str == "/stereo0")
    {
        camera = stereo0;
        NODELET_INFO("Loading stereo0 mask");
    }
    else if(camera_str == "stereo1" || camera_str == "/stereo1")
    {
        camera = stereo1;
        NODELET_INFO("Loading stereo1 mask");
    }
    else
    {
        camera = stereo0;
        NODELET_ERROR("No camera given to the mask. Using stereo0.");
    }

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

        if(camera == stereo0)
        {
            //Stereo0 - Left
            pts[0][0] = cv::Point( 215, 479 );
            pts[0][1] = cv::Point( 337, 334 );
            pts[0][2] = cv::Point( 343, 294 );
            pts[0][3] = cv::Point( 411, 279 );
            pts[0][4] = cv::Point( 488, 324 );
            pts[0][5] = cv::Point( 494, 375 );
            pts[0][6] = cv::Point( 611, 479 );
        }

        else if(camera == stereo1)
        {
            //Stereo1 - Left
            pts[0][0] = cv::Point( 265, 479 );
            pts[0][1] = cv::Point( 366, 395 );
            pts[0][2] = cv::Point( 372, 346 );
            pts[0][3] = cv::Point( 445, 320 );
            pts[0][4] = cv::Point( 551, 360 );
            pts[0][5] = cv::Point( 548, 408 );
            pts[0][6] = cv::Point( 611, 479 );
        }

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

        if(camera == stereo0)
        {
            //Stereo0 - Right
            pts[0][0] = cv::Point( 215, 479 );
            pts[0][1] = cv::Point( 337, 334 );
            pts[0][2] = cv::Point( 343, 294 );
            pts[0][3] = cv::Point( 411, 279 );
            pts[0][4] = cv::Point( 488, 324 );
            pts[0][5] = cv::Point( 494, 375 );
            pts[0][6] = cv::Point( 611, 479 );
        }

        else if(camera == stereo1)
        {
            //Stereo1 - Right
            pts[0][0] = cv::Point( 265, 479 );
            pts[0][1] = cv::Point( 366, 395 );
            pts[0][2] = cv::Point( 372, 346 );
            pts[0][3] = cv::Point( 445, 320 );
            pts[0][4] = cv::Point( 551, 360 );
            pts[0][5] = cv::Point( 548, 408 );
            pts[0][6] = cv::Point( 611, 479 );
        }

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
