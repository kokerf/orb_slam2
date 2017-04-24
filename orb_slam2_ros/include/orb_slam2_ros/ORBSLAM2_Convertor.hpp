#ifndef _ORB_SLAM2_CONVERTOR_HPP_
#define _ORB_SLAM2_CONVERTOR_HPP_

#include <iostream>
#include <assert.h>
#include <vector>
#include <stdint.h>

#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include <ros/ros.h>
#include <orb_slam2_msgs/Frame.h>

class ORBSLAM2_Convertor
{
public:
    struct Frame
    {
        std::vector<cv::KeyPoint> keypoints;
        std::vector<uint16_t> depths;
        std::vector<cv::Mat> descriptors;
    };

public:
    ORBSLAM2_Convertor(){}
    ~ORBSLAM2_Convertor(){}

    //! ===================================================
    //!             convert to struct Frame
    //! ===================================================
    static bool toFrame(const std::vector<cv::KeyPoint>& keypoints, const cv::Mat& descriptors, const std::vector<uint16_t>& depths, Frame& frame)
    {
        uint16_t N = keypoints.size();
        assert( N == descriptors.rows && N == depths.size());

        frame.keypoints.resize(N);
        frame.depths.resize(N);
        frame.descriptors.resize(N);
        for(uint16_t i = 0; i < N; ++i)
        {
            frame.keypoints[i] = keypoints[i];
            frame.descriptors[i] = descriptors.row(i);
            frame.depths[i] = depths[i];
        }

        return true;
    }

    static bool toFrame(const std::vector<cv::KeyPoint>& keypoints, const cv::Mat& descriptors, const cv::Mat& img_depth, Frame& frame)
    {
        uint16_t N = keypoints.size();
        assert( N == descriptors.rows && img_depth.type() == CV_16UC1);

        frame.keypoints.resize(N);
        frame.depths.resize(N);
        frame.descriptors.resize(N);
        for(uint16_t i = 0; i < N; ++i)
        {
            frame.keypoints[i] = keypoints[i];
            frame.descriptors[i] = descriptors.row(i);

            const float &v = keypoints[i].pt.y;
            const float &u = keypoints[i].pt.x;
            const uint16_t d = img_depth.at<uint16_t>(v,u);
            frame.depths[i] = d;
        }

        return true;
    }

    //! ===================================================
    //!             convert to ROS message
    //! ===================================================
    static bool toMessage(const cv::KeyPoint& keypoint, orb_slam2_msgs::KeyPoint& message)
    {
        message.pt.x = keypoint.pt.x;
        message.pt.y = keypoint.pt.y;
        message.size = keypoint.size;
        message.angle = keypoint.angle;
        message.response = keypoint.response;
        message.octave = keypoint.octave;
        message.class_id = keypoint.class_id;
        return true;
    }

    static bool toMessage(const cv::Mat& descriptor, orb_slam2_msgs::Descriptor& message)
    {
        assert(descriptor.type() == CV_8UC1);
        assert(descriptor.cols==1 || descriptor.rows==1);

        uint16_t  N = (descriptor.cols==1)? descriptor.rows: descriptor.cols;

        message.data.resize(N);
        for(uint16_t i = 0; i < N; ++i)
        {
            message.data[i] = descriptor.data[i];
        }

        return true;
    }

    static bool toMessage(const Frame& frame, const double& timestamp, orb_slam2_msgs::Frame& message)
    {
        uint16_t N = frame.keypoints.size();
        assert( N == frame.descriptors.size() && N == frame.depths.size());

        message.header.stamp.fromSec(timestamp);
        message.header.frame_id = "ORB_SLAM2";
        message.N = N;

        message.keypoints.clear();
        message.descriptors.clear();
        message.depths.clear();
        for(uint16_t i = 0; i < N; ++i)
        {
            orb_slam2_msgs::KeyPoint kp;
            toMessage(frame.keypoints[i], kp);
            message.keypoints.push_back(kp);

            orb_slam2_msgs::Descriptor dpt;
            toMessage(frame.descriptors[i], dpt);
            message.descriptors.push_back(dpt);

            message.depths.push_back(frame.depths[i]);
        }

        return true;
    }

    static bool toMessage(const std::vector<cv::KeyPoint>& keypoints, const cv::Mat& descriptors, const cv::Mat& img_depth, const double& timestamp, orb_slam2_msgs::Frame& message)
    {
        Frame frame;
        toFrame(keypoints, descriptors, img_depth ,frame);
        toMessage(frame, timestamp, message);
    }

    //! ===================================================
    //!             convert from ROS message
    //! ===================================================
    static bool fromMessage(const orb_slam2_msgs::KeyPoint& message, cv::KeyPoint& keypoint)
    {
        keypoint.pt.x = message.pt.x;
        keypoint.pt.y = message.pt.y;
        keypoint.size = message.size;
        keypoint.angle = message.angle;
        keypoint.response = message.response;
        keypoint.octave = message.octave;
        keypoint.class_id = message.class_id;
        return true;
    }

    static bool fromMessage(const orb_slam2_msgs::Descriptor& message, cv::Mat& descriptor)
    {
        uint16_t N = message.data.size();
        cv::Mat decp = cv::Mat::zeros(1, N, CV_8UC1);
        for(uint16_t i = 0; i < N; ++i)
        {
            decp.data[i] = message.data[i];
        }
        descriptor = decp.clone();

        return true;
    }

    static bool fromMessage(const orb_slam2_msgs::Frame& message, Frame& frame, double& timestamp)
    {
        uint16_t N = message.N;
        timestamp = message.header.stamp.toSec();

        frame.keypoints.resize(N);
        frame.depths.resize(N);
        frame.descriptors.resize(N);
        for(uint16_t i = 0; i < N; ++i)
        {
            fromMessage(message.keypoints[i], frame.keypoints[i]);
            fromMessage(message.descriptors[i], frame.descriptors[i]);

            frame.depths[i] = message.depths[i];
        }

        return true;
    }

    static bool fromMessage(const orb_slam2_msgs::Frame& message, std::vector<cv::KeyPoint>& keypoints, std::vector<cv::Mat>& descriptors, std::vector<uint16_t>& depths, double& timestamp)
    {
        Frame frame;
        fromMessage(message, frame, timestamp);

        uint16_t N = message.N;
        keypoints.resize(N);
        descriptors.resize(N);
        depths.resize(N);
        for(uint16_t i = 0; i < N; ++i)
        {
            keypoints[i] = frame.keypoints[i];
            descriptors[i] = frame.descriptors[i];
            depths[i] = frame.depths[i];
        }
    }
};

#endif