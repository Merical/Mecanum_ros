#include <boost/regex.hpp>
#include <string.h>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    float cmd_para[3];
    std::string message = msg->data;
    ROS_INFO("I heard: [%s]", message.c_str());
    boost::regex pattern("[\\d.]+");
    boost::sregex_iterator it(message.begin(), message.end(), pattern);
    boost::sregex_iterator last;
    for(int i=0; it != last; ++i){
        std::string str_message = it->str();
        cmd_para[i] = atof(str_message.c_str());
        ++it;
    }
    std::cout << "the array is " << cmd_para[0] << " " << cmd_para[1] << " " << cmd_para[2] << std::endl;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
    ros::spin();

    return 0;
}