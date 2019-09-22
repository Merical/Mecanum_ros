/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
//#include<vector>
//#include <direct.h> //mkdir path
#include <sys/stat.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//#include "ViconUtils.h"


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
//#include"../../../include/System.h"

using namespace std;

int counters = 0;
string save_path;
string savergb;
string savedepth;

string imagergb = "image";
string imaged = "depth";
string lie;
vector<string> rgbfiles;
vector<string> depthfiles;


//3将ros格式的图像转为opencv的Mat格式
void ImageSave(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);  //将ROS RGB消息转化为Opencv支持的RGB格式
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);  //将ROS深度消息转化为Opencv支持的深度格式
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    counters++;
    string lie;

    const string imagergb = "rgb1";
    cv::namedWindow(imagergb, cv::WINDOW_AUTOSIZE);

    cv::waitKey(1);
    cv::Mat image_rgb = cv_ptrRGB->image;
    cv::cvtColor(image_rgb, image_rgb, cv::COLOR_RGB2BGR);
    cv::imshow(imagergb,image_rgb);
    string picname_rgb =to_string( cv_ptrRGB->header.stamp.toSec());

    picname_rgb =save_path + "/rgb/" + picname_rgb + ".png";

    imwrite(picname_rgb, image_rgb);//保存rgb图片
    lie = to_string( cv_ptrRGB->header.stamp.toSec()); //时间戳

    string rgbfile = lie + " " + "rgb/" + lie + ".png";
    rgbfiles.push_back(rgbfile);


//深度
    const string imaged = "depth1";
    cv::namedWindow(imaged,  cv::WINDOW_AUTOSIZE);
    cv::imshow(imaged,cv_ptrD->image);
    cv::waitKey(1);

    cv::Mat image_depth = cv_ptrD->image;
    string picname_depth =to_string( cv_ptrD->header.stamp.toSec());//时间戳作为图片名

    picname_depth =save_path + "/depth/" + picname_depth + ".png";


    imwrite(picname_depth,  image_depth);//保存depth图片
    lie = to_string( cv_ptrD->header.stamp.toSec());//时间戳

    string depthfile = lie + " " + "depth/" + lie + ".png";
    depthfiles.push_back(depthfile);

    cout << "图片数："<<counters<<endl;

}


//2订阅话题
int main(int argc, char **argv)
{

    ros::init(argc, argv, "RGBD");
    ros::start();
    cout<<"start..."<<endl;

    ros::NodeHandle nh("ros_rgbd");

    std::string rgb_topic;
    std::string depth_topic;

    nh.param<std::string>("rgb_topic", rgb_topic, "/camera/rgb/image_raw");
    nh.param<std::string>("depth_topic", depth_topic, "/camera/depth/image_raw");
    nh.param<std::string>("dataset_path", save_path, "./dataset");

    // mkdir path
    savergb = save_path+"/rgb";
    savedepth = save_path+"/depth";

    mkdir(save_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);   // 返回 0 表示创建成功，-1 表示失败
    cout<<"创建文件夹: "<<save_path<<endl;

    mkdir(savergb.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);   // 返回 0 表示创建成功，-1 表示失败
    cout<<"创建文件夹: "<<savergb<<endl;

    mkdir(savedepth.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);   // 返回 0 表示创建成功，-1 表示失败
    cout<<"创建文件夹: "<<savedepth<<endl;

    message_filters::Subscriber<sensor_msgs::Image> rgb1_sub(nh, rgb_topic, 1);
    message_filters::Subscriber<sensor_msgs::Image> depth1_sub(nh, depth_topic, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb1_sub,depth1_sub);
    sync.registerCallback(boost::bind(&ImageSave,_1,_2));  //_1,_2表示占位符,分别用第1,2个参数代替

    ros::spin();

    //============================================
    //4保存图像到对应路径，并且生成depth.txt和rgb.txt
    ofstream frgb(save_path + "/rgb.txt");
    ofstream fdepth(save_path + "/depth.txt");

    for(uint64 i=0;i<rgbfiles.size();i++)
    {
        frgb<<rgbfiles[i]<<endl;
        fdepth<<depthfiles[i]<<endl;
    }

    frgb.close();
    fdepth.close();
    cout << "保存图片成功。\n\n";


    ros::shutdown();

    return 0;
}
