#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <ros/ros.h>
#include <orb_slam2_msgs/Frame.h>
#include "orb_slam2_ros/ORBSLAM2_Convertor.hpp"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "orb_slam2_talker");
	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<orb_slam2_msgs::Frame>("orb_slam2/frame", 1000);

    cv::Mat img_rgb = cv::imread("rgb.png", CV_LOAD_IMAGE_UNCHANGED);
    cv::Mat img_depth = cv::imread("depth.png", CV_LOAD_IMAGE_UNCHANGED);
    if(img_rgb.empty() || img_depth.empty())
    {
        printf("Can't read one of the images\n");
        return -1;
    }

    cv::Mat img_gray;
    if(img_rgb.channels() == 3)
       cvtColor(img_rgb, img_gray, CV_RGBA2GRAY);

    cv::ORB orb(500, 1.2f, 8);

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    orb(img_gray, cv::Mat(), keypoints, descriptors);

    cv::Mat img_show;
    cv::drawKeypoints( img_rgb, keypoints, img_show, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
    cv::imshow("takler", img_show);
    cv::waitKey(0);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::Time t= ros::Time::now();
        orb_slam2_msgs::Frame msg_frame;
        ORBSLAM2_Convertor::toMessage(cv::Size(img_gray.rows, img_gray.cols), keypoints, descriptors, img_depth , t.toSec(), msg_frame);

        chatter_pub.publish(msg_frame);

        ROS_INFO("%ld", msg_frame.header.stamp.toNSec());

        loop_rate.sleep();
    }

	return 0;
}
