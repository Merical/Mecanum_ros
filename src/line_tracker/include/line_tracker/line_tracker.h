//
// Created by lishenghao on 19-8-11.
//

#ifndef LINE_TRACKER_LINE_TRACKER_H
#define LINE_TRACKER_LINE_TRACKER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sc_msgs/vision_line.h>
#include "std_msgs/String.h"
//#include "geometry_msgs/Twist.h"

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace std;

//static const string COLOR_WINDOW = "COLOR FRAME";

class LineTracker{
private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber color_sub;

    cv::Scalar_<double> rangeLow = cv::Scalar(156, 43, 46);
    cv::Scalar_<double> rangeHigh = cv::Scalar(180, 255, 255);

    cv::Scalar_<double> rangeLowSecond = cv::Scalar(0, 43, 46);
    cv::Scalar_<double> rangeHighSecond = cv::Scalar(25, 255, 255);

    cv::Size shape = cv::Size((int) 640, (int) 480);
    int gap = shape.height / 20;

public:
    ros::Publisher chatter_pub;
    image_transport::Publisher color_pub;

//    cv::Mat ColorFrame = cv::Mat::zeros(cv::Size2f(640, 480), CV_64F);
//    cv::Mat LineFrame = cv::Mat::zeros(cv::Size2f(640, 480), CV_64F);
    cv::Mat ColorFrame;
    cv::Mat LineFrame;
    sensor_msgs::ImagePtr color_msg;
//    sc_msgs::vision_line line_msg;

    LineTracker(): it(nh){
//        color_sub = it.subscribe("/ob_vision/line/image_raw", 1, &LineTracker::colorGrab, this);
        color_sub = it.subscribe("/usb_cam/image_raw", 1, &LineTracker::colorGrab, this);
        chatter_pub = nh.advertise<sc_msgs::vision_line>("/ob_vision/line_tracker/chatter", 1000);
        color_pub = it.advertise("ob_vision/line_tracker/color", 1);

    }

    ~LineTracker(){
    }

    void colorGrab(const sensor_msgs::ImageConstPtr& msgs);
    void run();

    cv::Point detect_by_row(cv::Mat &frame_, cv::Mat &redLine_, int height_);
    void largest_connectted_component(const cv::Mat &srcImage, cv::Mat &dstImage);
    void centers_to_message(const vector<cv::Point>& centers);
};



#endif //LINE_TRACKER_LINE_TRACKER_H
