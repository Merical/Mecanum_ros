//
// Created by lishenghao on 19-8-11.
//

#include "line_tracker.h"

cv::Point LineTracker::detect_by_row(cv::Mat &frame_, cv::Mat &redLine_, int height_){
    cv::Mat diff;
    cv::Mat signed_thresh(redLine_.row(height_));
    cv::Sobel(signed_thresh, diff, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
//    cout << "the diff size is " << diff.rows << " x " << diff.cols << endl;

    double minValue, maxValue;
    cv::Point minLoc, maxLoc, middle;
    cv::minMaxLoc(diff, &minValue, &maxValue, &minLoc, &maxLoc);
//    cout << "minValue is " << minValue << "\nmaxValue is " << maxValue << endl;
//    cout << "minLoc is " << minLoc << "\nmaxLoc is " << maxLoc << endl;
//    cout << diff << endl;
    cv::line(frame_, cv::Point(0, height_), cv::Point(frame_.cols, height_), cv::Scalar(0, 255, 0), 2);
    if (minValue != 0 and maxValue != 0) {
        middle = cv::Point((minLoc.x + maxLoc.x) / 2, height_);
        cv::circle(frame_, cv::Point(minLoc.x, height_), 3, cv::Scalar(255, 0, 0), -1);
        cv::circle(frame_, cv::Point(maxLoc.x, height_), 3, cv::Scalar(255, 0, 0), -1);
        cv::circle(frame_, middle, 5, cv::Scalar(0, 0, 255), -1);
    }

    return cv::Point(middle.x, shape.height - middle.y);
}

void LineTracker::largest_connectted_component(const cv::Mat &srcImage, cv::Mat &dstImage){
    cv::Mat labels;

    int n_comps = cv::connectedComponents(srcImage, labels, 4, CV_16U);
    vector<int> histogram_of_labels;
    for (int i = 0; i < n_comps; i++)
    {
        histogram_of_labels.push_back(0);
    }

    int rows = labels.rows;
    int cols = labels.cols;
    for (int row = 0; row < rows; row++)
    {
        for (int col = 0; col < cols; col++)
        {
            histogram_of_labels.at(labels.at<unsigned short>(row, col)) += 1;
        }
    }
    histogram_of_labels.at(0) = 0;

    int maximum = 0;
    int max_idx = 0;
    for (int i = 0; i < n_comps; i++)
    {
        if (histogram_of_labels.at(i) > maximum)
        {
            maximum = histogram_of_labels.at(i);
            max_idx = i;
        }
    }

    for (int row = 0; row < rows; row++)
    {
        for (int col = 0; col < cols; col++)
        {
            if (labels.at<unsigned short>(row, col) == max_idx)
            {
                labels.at<unsigned short>(row, col) = 255;
            }
            else
            {
                labels.at<unsigned short>(row, col) = 0;
            }
        }
    }

    labels.convertTo(dstImage, CV_8U);
}

void LineTracker::colorGrab(const sensor_msgs::ImageConstPtr& msgs) {
    try {
        cv_bridge::toCvShare(msgs, "bgr8")->image.copyTo(ColorFrame);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: &s", e.what());
        return;
    }
}

void LineTracker::centers_to_message(const vector<cv::Point>& centers){
    sc_msgs::vision_line line_msg;

    if (centers[0].x != 0){ // only publish line if the baseline detected the line
        int translate_x = centers[0].x - shape.width/2;
        int translate_y = gap * 1;
        line_msg.translate_x = translate_x;
        line_msg.translate_y = translate_y;

        int rotate_x = (centers[3].x + centers[4].x + centers[5].x) / 3 - shape.width / 2 - translate_x;
        int rotate_y = gap * 5;
        line_msg.rotate_x = rotate_x;
        line_msg.rotate_y = rotate_y;
    }


    chatter_pub.publish(line_msg);
}

void LineTracker::run()
{
    cv::Mat frame, frame_hsv;
    ColorFrame.copyTo(frame);
    if(frame.rows == 480) {
        cv::cvtColor(frame, frame_hsv, cv::COLOR_BGR2HSV);

        cv::Mat redLine, redLineSecond, connected;
        cv::inRange(frame_hsv, rangeLow, rangeHigh, redLine);
        cv::inRange(frame_hsv, rangeLowSecond, rangeHighSecond, redLineSecond);
        redLine = redLine + redLineSecond;
//        cv::imshow("RedLineFinal", redLine);
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(redLine, redLine, cv::MORPH_OPEN, element, {-1, -1}, 2);
        largest_connectted_component(redLine, connected);


        vector <cv::Point> centers; // 19 centers
        for (int i = 1; i < shape.height / gap; i++) {
            //            centers.push_back(detect_by_row(frame, redLine, gap*i));
            centers.push_back(detect_by_row(frame, connected, gap * (20 - i)));
        }
//        cout << centers[0] << endl;
        frame.copyTo(LineFrame);
        color_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", LineFrame).toImageMsg();
        color_pub.publish(color_msg);
        if (not centers.empty()) centers_to_message(centers);
    }
}