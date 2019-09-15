#!/bin/bash

INIT_DIR = $(pwd)

echo "export ROS_MASTER_URI=http://192.168.10.11:11311" >> ~/.bashrc 
echo "export ROS_HOSTNAME=192.168.10.11" >> ~/.bashrc 

cd ~/
mkdir ros_workspace
cd ros_workspace
mkdir SC0_ws
CATKIN_DIR = $(pwd)
cp -r ${INIT_DIR}/src ./

echo "123456789o" | sudo apt-get update
sudo cp src/Documentation/sc_udev/* /ect/udev/rules.d/
sudo /etc/init.d/udev restart

sudo usermod -a -G dialout $USER
s
sudo apt-get install -y ros-kinetic-turtlebot-bringup ros-kinetic-turtlebot-create-desktop ros-kinetic-openni-* ros-kinetic-openni2-* ros-kinetic-freenect-* ros-kinetic-usb-cam ros-kinetic-laser-* ros-kinetic-hokuyo-node ros-kinetic-audio-common gstreamer0.10-pocketsphinx ros-kinetic-pocketsphinx ros-kinetic-slam-gmapping ros-kinetic-joystick-drivers python-rosinstall ros-kinetic-orocos-kdl ros-kinetic-python-orocos-kdl python-setuptools ros-kinetic-dynamixel-motor-*  ros-kinetic-depthimage-to-laserscan ros-kinetic-arbotix-* ros-kinetic-turtlebot-teleop ros-kinetic-move-base ros-kinetic-map-server ros-kinetic-fake-localization ros-kinetic-amcl git subversion mercurial

sudo apt-get install -y ros-kinetic-ros-controllers

cd ${CATKIN_DIR}
catkin_make

echo "source /home/obt-sc/ros_workspace/SC0_ws/devel/setup.bash" >> ~/.bashrc 
source ~/.bashrc

