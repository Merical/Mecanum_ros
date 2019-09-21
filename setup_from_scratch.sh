#!/bin/bash

cd ~/
mkdir Sources
echo "123456789o" | sudo -S apt-get update
sudo apt-get install -y git wget build-essential cmake tar ntpdate

cd ~/Downloads
wget "https://github.com/IntelRealSense/librealsense/archive/v2.16.1.tar.gz"
tar zxvf v2.16.1.tar.gz -C ~/Sources
cd ~/Sources/librealsense-2.16.1
mkdir build && cd build
cmake ..
make -j4
sudo make -j4 install

cd ~/Downloads
wget "https://github.com/opencv/opencv_contrib/archive/3.4.6.tar.gz"
mv 3.4.6.tar.gz contrib.3.4.6.tar.gz
wget "https://github.com/opencv/opencv/archive/3.4.6.tar.gz"
tar zxvf 3.4.6.tar.gz -C ~/Sources
tar zxvf contrib.3.4.6.tar.gz -C ~/Sources
cd ~/Sources/opencv-3.4.6
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=/home/obt-sc/Sources/opencv_contrib-3.4.6/modules -D BUILD_TIFF=ON -D OPENCV_ENABLE_NONFREE=ON -DBUILD_PNG=ON ..
make -j4
sudo make -j4 install

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

cd ~/
git clone https://github.com/Merical/Mecanum_ros.git
cd Mecanum_ros
GIT_DIR=$(pwd)

echo "export HANDSFREE_ROBOT_MODEL=stone_v2" >> ~/.bashrc 
echo "export SC_ROBOT_MODEL=mecanum" >> ~/.bashrc 
echo "export ROS_MASTER_URI=http://192.168.10.11:11311" >> ~/.bashrc 
echo "export ROS_HOSTNAME=192.168.10.11" >> ~/.bashrc 

cd ~/
mkdir ros_workspace
cd ros_workspace
mkdir SC0_ws
cd SC0_ws
CATKIN_DIR=$(pwd)
cp -r ${INIT_DIR}/src ./

sudo cp src/Documentation/sc_udev/* /etc/udev/rules.d/
sudo /etc/init.d/udev restart

sudo usermod -a -G dialout $USER
sudo apt-get install -y ros-kinetic-turtlebot-bringup ros-kinetic-turtlebot-create-desktop ros-kinetic-openni-* ros-kinetic-openni2-* ros-kinetic-freenect-* ros-kinetic-usb-cam ros-kinetic-laser-* ros-kinetic-hokuyo-node ros-kinetic-audio-common gstreamer0.10-pocketsphinx ros-kinetic-pocketsphinx ros-kinetic-slam-gmapping ros-kinetic-joystick-drivers python-rosinstall ros-kinetic-orocos-kdl ros-kinetic-python-orocos-kdl python-setuptools ros-kinetic-dynamixel-motor-*  ros-kinetic-depthimage-to-laserscan ros-kinetic-arbotix-* ros-kinetic-turtlebot-teleop ros-kinetic-move-base ros-kinetic-map-server ros-kinetic-fake-localization ros-kinetic-amcl git subversion mercurial

sudo apt-get install -y ros-kinetic-ros-controllers

cd ${CATKIN_DIR}
catkin_make

echo "source /home/obt-sc/ros_workspace/SC0_ws/devel/setup.bash" >> ~/.bashrc 
source ~/.bashrc

sudo echo "server 127.127.1.0" >> /etc/ntp.conf
sudo echo "fudge 127.127.1.0 stratum 5" >> /etc/ntp.conf
systemctl restart ntpdate.service
