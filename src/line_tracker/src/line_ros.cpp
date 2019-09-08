//
// Created by lishenghao on 19-8-11.
//

#include "line_tracker.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "line_tracker_processer");
    LineTracker lt;
    ros::Rate loop_rate(20);
    sleep(1);

    while(ros::ok())
    {
//        lt.chatter_pub.publish(cft.CMTmsg);
//        cv::startWindowThread();
        lt.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}
