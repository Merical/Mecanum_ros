#!/bin/bash

export SC_ROBOT_MODEL=mecanum
export HANDSFREE_ROBOT_MODEL=stone_v2
source /opt/ros/kinetic/setup.bash
source /home/obt-sc/ros_workspace/SC0_ws/devel/setup.bash

export ROS_MASTER_URI=http://192.168.10.11:11311
export ROS_HOSTNAME=192.168.10.11

while(true)
do
#    mate-terminal -x roslaunch ocean_audio server_ros.launch
    roslaunch ocean_audio server_pad.launch
    echo "\n\n\n Loop Done \n\n\n"
    sleep 1
    echo "Start a New Server"
done

