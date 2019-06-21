#!/bin/bash

sudo usermod -a -G dialout $USER

sudo apt-get install ros-kinetic-turtlebot-bringup ros-kinetic-turtlebot-create-desktop ros-kinetic-openni-* ros-kinetic-openni2-* ros-kinetic-freenect-* ros-kinetic-usb-cam ros-kinetic-laser-* ros-kinetic-hokuyo-node ros-kinetic-audio-common gstreamer0.10-pocketsphinx ros-kinetic-pocketsphinx ros-kinetic-slam-gmapping ros-kinetic-joystick-drivers python-rosinstall ros-kinetic-orocos-kdl ros-kinetic-python-orocos-kdl python-setuptools ros-kinetic-dynamixel-motor-*  ros-kinetic-depthimage-to-laserscan ros-kinetic-arbotix-* ros-kinetic-turtlebot-teleop ros-kinetic-move-base ros-kinetic-map-server ros-kinetic-fake-localization ros-kinetic-amcl git subversion mercurial

sudo apt-get install ros-kinetic-ros-controllers

#libopencv-dev python-opencv  ros-kinetic-vision-opencv
