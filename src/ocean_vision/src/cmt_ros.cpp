#include <iostream>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "CMT.h"
#include "gui.h"
#include "cmt_ros.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/video/video.hpp"

using std::cout;
using std::endl;
using cv::Scalar;
using cv::Point;

int main(int argc, char** argv) {
    ros::init(argc, argv, "cmt_frame_processer");
    CmtFrameTrans cft;
    ros::Rate loop_rate(20);
    sleep(1);

    while(ros::ok())
    {
        cft.chatter_pub.publish(cft.CMTmsg);
        cft.color_pub.publish(cft.color_msg);
        cv::startWindowThread();
//        cout << "LCH: the ColorFrame is " << cft.ColorFrame << endl;
//        cv::imshow(COLOR_WINDOW, cft.ColorFrame);
//        cv::waitKey(1);
        cft.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}


void CmtFrameTrans::colorGrab(const sensor_msgs::ImageConstPtr& msgs) {
//    cout << "LCH: The color grab callback ..." << endl;
    try {
        cv_bridge::toCvShare(msgs, "bgr8")->image.copyTo(ColorFrame);
//            cv::resize(ColorFrame, ColorFrame, CvSize(320, 240));
        cv::cvtColor(ColorFrame, GrayFrame, CV_BGR2GRAY);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: &s", e.what());
        return;
    }
}



void CmtFrameTrans::depthGrab(const sensor_msgs::ImageConstPtr& msg) {
//    cout << "LCH: depthGrab callback ! " << endl;
//        cv_bridge::CvImagePtr cv_ptr;
//    double dist_val[5];
    try
    {
//            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
//            cv_ptr->image.copyTo(DepthFrame);
        cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image.copyTo(DepthFrame);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.", msg->encoding.c_str());
    }

    if (fDepth)
    {
//        dist_val[0] = DepthFrame.at<float>(cmt.center)/100;
//        dist_val[1] = DepthFrame.at<float>(cmt.center.x+cmt.bb_rot.size.width/5, cmt.center.y+cmt.bb_rot.size.height/5)/100;
//        dist_val[2] = DepthFrame.at<float>(cmt.center.x-cmt.bb_rot.size.width/5, cmt.center.y+cmt.bb_rot.size.height/5)/100;
//        dist_val[3] = DepthFrame.at<float>(cmt.center.x-cmt.bb_rot.size.width/5, cmt.center.y-cmt.bb_rot.size.height/5)/100;
//        dist_val[4] = DepthFrame.at<float>(cmt.center.x+cmt.bb_rot.size.width/5, cmt.center.y-cmt.bb_rot.size.height/5)/100;
//        int num_depth_active = 5;
//        float temp_dist = 0;
//
//        for(int i = 0; i < 5; i++)
//        {
//            if (dist_val[i] > 0.1 && dist_val[i] < 15.0)
//                temp_dist += dist_val[i];
//            else
//                num_depth_active --;
//        }
//        TargetDepth = temp_dist / num_depth_active;
        TargetDepth = DepthFrame.at<float>(cmt.center)/1000;
//        cout << "LCH: the Target Depth is " << TargetDepth << " m" << endl;
    }
//    cv::imshow(DEPTH_WINDOW, DepthFrame);
//    cv::waitKey(5);
}


int CmtFrameTrans::display_ros_copy(Mat im, const string win_name) {
    //Visualize the output
    //It is ok to draw on im itself, as CMT only uses the grayscale image
    if (cmt.has_result) {
        for (size_t i = 0; i < cmt.points_active.size(); i++) {
            circle(im, cmt.points_active[i], 2, Scalar(0, 255, 0));
        }

        Point2f vertices[4];
        cmt.bb_rot.points(vertices);
        for (int i = 0; i < 4; i++) {
            line(im, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0));
        }
    }

    im.copyTo(ColorResult);
    color_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", im).toImageMsg();
    imshow(win_name, im);

    return cv::waitKey(10);
}


void CmtFrameTrans::getTopicRect(const sc_msgs::vision_rect& rect) {
    Rect = cv::Rect(Point(rect.topleft_x,rect.topleft_y), Point(rect.bottomright_x, rect.bottomright_y));
    cout << "LCH: the rect is ("<< Rect.x << ',' << Rect.y << "), with "<< Rect.width << "x" << Rect.height << endl;
}


void CmtFrameTrans::run()
{
    cv::Mat color;
    cv::Mat gray;
    ColorFrame.copyTo(color);
    GrayFrame.copyTo(gray);

    if(fPreview)
    {
        screenLog(color, "Press a key to start selecting an object. ");
        color.copyTo(ColorResult);
        color_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color).toImageMsg();
        cv::imshow(COLOR_WINDOW, color);
//        cout << "LCH: the rect is ("<< Rect.x << ',' << Rect.y << "), with "<< Rect.width << "x" << Rect.height << endl;

        char key = cv::waitKey(10);
        if (key == 'q'){exit(0); }
        else if (Rect.width != 0 && Rect.height != 0){
            cout << "LCH: the rect is ("<< Rect.x << ',' << Rect.y << "), with "<< Rect.width << "x" << Rect.height << endl;
            fPreview = false;
            fCMT = true;
            fDepth = true;
            cmt.initialize(gray, Rect);
            cv::waitKey(5);
        }
        else if (key == 'a'){
            fPreview = false;
            fCMT = true;
            fDepth = true;
            Rect = getRect(color, COLOR_WINDOW);
            cout << "LCH: the rect is ("<< Rect.x << ',' << Rect.y << "), with "<< Rect.width << "x" << Rect.height << endl;
            cmt.initialize(gray, Rect);
            cv::waitKey(5);
        }
    }
    if (fCMT)
    {
        cmt.processFrame(gray);

        if (cmt.has_result){
            ss << "$$CMT CENTER " << cmt.center << " Depth " << TargetDepth << " Scale " << cmt.scale << "&&";
            fDepth = true;
        }
        else{
            ss << "$$Target Lost 0 &&";
            fDepth = false;
        }
        CMTmsg.data = ss.str();
//        ROS_INFO("%s", CMTmsg.data.c_str());
        chatter_pub.publish(CMTmsg);
        char key = display_ros_copy(color, COLOR_WINDOW);
        if (key == 'q') exit(0);
        else if (key == 'r') {
            fCMT = false;
            fDepth = false;
            fPreview = true;
        }
    }
}
