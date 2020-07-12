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
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/video/video.hpp"
using cmt::CMT;
using namespace cv;
using namespace std;
void readme();
bool is_rect(vector<Point2f> corners);
int display(Mat im, CMT & cmt, string fps);
Rect cornersToRect(vector<Point2f> corners);
/* @function main */
int main( int argc, char** argv )
{
    bool org_flag;
    if( argc == 2 ) {
        org_flag = true;
        cout << "Using the template ..." << endl;
        }
    else{
        org_flag = false;
        cout << "Please press button a when you want to choose an object ..." << endl;
    }

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(10);

    FileStorage fs2("/home/robot/ros_workspace/ocean_ws/src/ocean_vision/config/config.yml", FileStorage::READ);
    int video_source, video_fps;
    uint16_t FRAME_WIDTH, FRAME_HEIGHT;
    string img_org_path;
    fs2["Video"]["FRAME_WIDTH"] >> FRAME_WIDTH;
    fs2["Video"]["FRAME_HEIGHT"] >> FRAME_HEIGHT;
    fs2["Video"]["source"] >> video_source;
    fs2["Video"]["fps"] >> video_fps;

    double fps = 0;
    char fps_str[10];
    CMT cmt;
    Rect rect;

    VideoCapture cap(video_source);
    cap.set(CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    cap.set(CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    cap.set(CAP_PROP_FPS, video_fps);
    Mat img_scene;
    Mat frame;
    double max_dist = 0;
    double min_dist = 100;
    double dist;

    if(!cap.isOpened()){
        cout << "Fail to open camera!" << endl;
        return -1;
    }

    namedWindow("Detect", WINDOW_NORMAL);
    moveWindow("Detect", 300, 200);
    double tic;
    if(org_flag){

        vector<Point2f> obj_corners(4);
        vector<Point2f> scene_corners(4);

        Mat img_object = imread( argv[1], IMREAD_GRAYSCALE );
        resize(img_object, img_object, Size(FRAME_WIDTH, FRAME_HEIGHT));
        if( !img_object.data )
        { cout<< " --(!) Error reading images " << endl; return -1; }

        Ptr<ORB> detector = ORB::create();
        vector<KeyPoint> keypoints_object, keypoints_scene;
        Mat descriptors_object, descriptors_scene;
        detector->detectAndCompute( img_object, Mat(), keypoints_object, descriptors_object );
        cv::Ptr<DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
        vector<DMatch> matches;
        int good_count = 0;

        while(true) {
            tic = (double)getTickCount();
            string fpsString("FPS:");
            sprintf(fps_str, "%.2f", fps);
            fpsString += fps_str;
            cap >> frame;
            Mat img_matches;
            frame.copyTo(img_matches);
            cvtColor(frame, img_scene, CV_BGR2GRAY);

            detector->detectAndCompute(img_scene, Mat(), keypoints_scene, descriptors_scene);
            if(!descriptors_scene.empty()){
                matcher->match(descriptors_object, descriptors_scene, matches);
                for (int i = 0; i < descriptors_object.rows; i++) {
                    dist = matches[i].distance;
                    if (dist < min_dist) min_dist = dist;
                    if (dist > max_dist) max_dist = dist;
                }
            //            printf("-- Max dist : %f \n", max_dist);
            //            printf("-- Min dist : %f \n", min_dist);

                vector<DMatch> good_matches;
                for (int i = 0; i < descriptors_object.rows; i++) {
                    if (matches[i].distance <= 3 * min_dist) { good_matches.push_back(matches[i]); }
                }
                vector<Point2f> obj;
                vector<Point2f> scene;
                for (size_t i = 0; i < good_matches.size(); i++) {
                    //-- Get the keypoints from the good matches
                    obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
                    scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
                }
                if(!scene.empty()){
                    Mat H = findHomography(obj, scene, RANSAC);
                    //-- Get the corners from the image_1 ( the object to be "detected" )
                    obj_corners[0] = cvPoint(0, 0);
                    obj_corners[1] = cvPoint(img_object.cols, 0);
                    obj_corners[2] = cvPoint(img_object.cols, img_object.rows);
                    obj_corners[3] = cvPoint(0, img_object.rows);

                    if (!H.empty()){
                        perspectiveTransform(obj_corners, scene_corners, H);
                        if(is_rect(scene_corners)){
            //                        cout << "LCH: scene_corners are: " << scene_corners << endl;
                            line(img_matches, scene_corners[0], scene_corners[1], Scalar(0, 255, 0), 4);
                            line(img_matches, scene_corners[1], scene_corners[2], Scalar(0, 255, 0), 4);
                            line(img_matches, scene_corners[2], scene_corners[3], Scalar(0, 255, 0), 4);
                            line(img_matches, scene_corners[3], scene_corners[0], Scalar(0, 255, 0), 4);
                            good_count ++;
                        }
                    }
                    else {good_count = 0;}
                }
                else {good_count = 0;}
            }
            else {good_count = 0;}

            putText(img_matches, fpsString, Point(5, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0));
            imshow("Detect", img_matches);
            char k = waitKey(10);
            tic = ((double)getTickCount() - tic) / getTickFrequency();
            fps = 1.0 / tic;
            if (k == 'q'){exit(0); }
            if (good_count > 10){
                break;
            }
        }
        rect = cornersToRect(scene_corners);
        cout << "LCH: the rect is ("<< rect.x << ',' << rect.y << "), with "<< rect.width << "x" << rect.height << endl;
    }
    else{
        bool show_preview = true;
        while (show_preview) {
            cap >> frame;

            screenLog(frame, "Press a key to start selecting an object.");
            imshow("Detect", frame);

            char k = waitKey(10);
            if (k == 'q'){exit(0); }
            if (k == 'a') {
                show_preview = false;
            }
        }
        cap >> frame;
        rect = getRect(frame, "Detect");
        cout << "LCH: the rect is ("<< rect.x << ',' << rect.y << "), with "<< rect.width << "x" << rect.height << endl;
        cvtColor(frame, img_scene, CV_BGR2GRAY);
    }


    cmt.initialize(img_scene, rect);
    destroyAllWindows();
    namedWindow("Tracker", WINDOW_NORMAL);
    moveWindow("Tracker", 300, 200);
    int count = 0;
    while (ros::ok())
    {
        tic = (double)getTickCount();
        string fpsString("FPS:");
        sprintf(fps_str, "%.2f", fps);
        fpsString += fps_str;

        std_msgs::String msg;
        std::stringstream ss;
        //If loop flag is set, reuse initial image (for debugging purposes)
        cap >> frame; //Else use next image in stream

        if (frame.empty()) break; //Exit at end of video stream

        Mat im_gray;
        cvtColor(frame, im_gray, CV_BGR2GRAY);

        cmt.processFrame(im_gray);

        char key = display(frame, cmt, fpsString);
        tic = ((double)getTickCount() - tic) / getTickFrequency();
        fps = 1.0 / tic;
        if(key == 'q') break;

        if (cmt.has_result){
            ss << "CMT CENTER " << cmt.center << " Scale " << cmt.scale;
        }
        else{
            ss << "Target Lost";

        }
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
  /* @function readme */
  void readme()
  { cout << " Usage: ./orb_descriptor <img_object>" << endl; }

  bool is_rect(vector<Point2f> corners)
  {
      double delta1 = norm(corners[2] - corners[0]);
      double delta2 = norm(corners[3] - corners[1]);
      double rectness = abs(delta1 - delta2);
      if(rectness < 10 && delta1 > 50 && delta2 > 50){
          return true;
      }
      else{
          return false;
      }
  }

int display(Mat im, CMT & cmt, string fps)
{
    //Visualize the output
    //It is ok to draw on im itself, as CMT only uses the grayscale image
    if (cmt.has_result) {
        for (size_t i = 0; i < cmt.points_active.size(); i++) {
            circle(im, cmt.points_active[i], 2, Scalar(255, 0, 0));
        }

        Point2f vertices[4];
        cmt.bb_rot.points(vertices);
        for (int i = 0; i < 4; i++) {
            line(im, vertices[i], vertices[(i + 1) % 4], Scalar(255, 0, 0));
        }
    }

    putText(im, fps, Point(5, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0));
    imshow("Tracker", im);

    return waitKey(1);
}

Rect cornersToRect(vector<Point2f> corners){
    vector<float> x_coord = {corners[0].x, corners[1].x, corners[2].x, corners[3].x};
    vector<float> y_coord = {corners[0].y, corners[1].y, corners[2].y, corners[3].y};

    auto x_max = max_element(x_coord.begin(), x_coord.end());
    auto x_min = min_element(x_coord.begin(), x_coord.end());
    auto y_max = max_element(y_coord.begin(), y_coord.end());
    auto y_min = min_element(y_coord.begin(), y_coord.end());

    return Rect(int16_t (*x_min - 5), int16_t (*y_min - 5), int16_t (*x_max-*x_min + 10), int16_t (*y_max-*y_min + 10));
}
