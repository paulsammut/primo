#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

image_transport::Publisher pubLeft;
image_transport::Publisher pubRight;

void imageLeftCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        // Ok cool, now we have an image, and now we need to draw on it!
        // Make the polygon

        cv::Point pts[1][3];
        pts[0][0] = cv::Point(   0, 479 );
        pts[0][1] = cv::Point( 400, 222 );
        pts[0][2] = cv::Point( 624, 479 );

        const cv::Point* ppt[1] = { pts[0] };
        int npt[] = {3};

        // boost::timer::cpu_timer timer1;
        cv::fillPoly(image, ppt, npt, 1, cv::Scalar(0, 0, 0), cv::LINE_8);
        // printf(timer1.format().c_str());

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

        pubLeft.publish(msg);

        // cv::imshow("view", image);
        // cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void imageRightCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        // Ok cool, now we have an image, and now we need to draw on it!
        // Make the polygon

        cv::Point pts[1][3];
        pts[0][0] = cv::Point(   0, 479 );
        pts[0][1] = cv::Point( 400, 222 );
        pts[0][2] = cv::Point( 624, 479 );

        const cv::Point* ppt[1] = { pts[0] };
        int npt[] = {3};

        // boost::timer::cpu_timer timer1;
        cv::fillPoly(image, ppt, npt, 1, cv::Scalar(0, 0, 0), cv::LINE_8);
        // printf(timer1.format().c_str());

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

        pubRight.publish(msg);
        
        // cv::imshow("view", image);
        // cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    // cv::namedWindow("view");
    // cv::startWindowThread();

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subLeft = it.subscribe("/stereo0/left/image_raw", 1, imageLeftCallback);
    image_transport::Subscriber subRight = it.subscribe("/stereo0/right/image_raw", 1, imageRightCallback);

    pubLeft = it.advertise("/stereo0/left/image_raw_mask", 1);
    pubRight = it.advertise("/stereo0/right/image_raw_mask", 1);

    ros::spin();
    // cv::destroyWindow("view");
}
