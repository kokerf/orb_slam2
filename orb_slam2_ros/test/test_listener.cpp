#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <orb_slam2_msgs/Frame.h>
#include "orb_slam2_ros/ORBSLAM2_Convertor.hpp"

cv::Mat image;

void listenerCallback(const orb_slam2_msgs::Frame& message)
{
    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::Mat> descriptors;
    std::vector<uint16_t> depths;
    double time;
    ORBSLAM2_Convertor::fromMessage(message, keypoints, descriptors, depths, time);

    cv::Mat image_with_keypoints;
    cv::drawKeypoints( image, keypoints, image_with_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
    cv::imshow("takler", image_with_keypoints);


    ros::Time t;
    t.fromSec(time);
    ROS_INFO("%ld", t.toNSec());
    cv::waitKey(10);

}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "orb_slam2_listener");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("orb_slam2/frame", 1000, listenerCallback);

    image = cv::imread("rgb.png", CV_LOAD_IMAGE_UNCHANGED);

    ros::spin();

    return 0;
}