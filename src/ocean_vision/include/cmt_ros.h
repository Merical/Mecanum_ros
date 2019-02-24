//
// Created by lishenghao on 18-11-25.
//

#ifndef CMT_ROS_H
#define CMT_ROS_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <dirent.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/Twist.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "CMT.h"
#include "gui.h"

using std::cout;
using std::endl;

static const std::string COLOR_WINDOW = "COLOR FRAME";
static const std::string DEPTH_WINDOW = "DEPTH FRAME";

class CmtFrameTrans
{
private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber color_sub;
    image_transport::Subscriber depth_sub;

    double TargetDepth = 0.0;
//    double dist_val[5];

public:
    ros::Publisher chatter_pub;
    std_msgs::String CMTmsg;

    cv::Mat ColorFrame = cv::Mat::zeros(Size2f(640, 480), CV_64F);
    cv::Mat GrayFrame = cv::Mat::zeros(Size2f(640, 480), CV_64F);
    cv::Mat DepthFrame = cv::Mat::zeros(Size2f(640, 480), CV_64F);
    cv::Rect Rect;

    bool fPreview   =   true;
    bool fCMT       =   false;
    bool fRenewRect =   false;
    bool fDepth     =   false;

    cmt::CMT cmt;
    std::stringstream ss;

    CmtFrameTrans(): it(nh)
    {
        color_sub = it.subscribe("/camera/color/image_raw", 1, &CmtFrameTrans::colorGrab, this);
        depth_sub = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &CmtFrameTrans::depthGrab, this);
        chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

        cv::namedWindow(COLOR_WINDOW, CV_WINDOW_NORMAL);
        cv::namedWindow(DEPTH_WINDOW, CV_WINDOW_NORMAL);
    }

    ~CmtFrameTrans()
    {
        cv::destroyAllWindows();
    }

    void colorGrab(const sensor_msgs::ImageConstPtr& msgs);
    void depthGrab(const sensor_msgs::ImageConstPtr& msg);
    void run();


};


#endif //CMT_ROS_H
