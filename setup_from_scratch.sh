#!/bin/bash

cd ~/
mkdir Sources
echo "123456789o" | sudo -S apt-get update
sudo apt-get install -y git wget build-essential cmake tar ntpdate

cd ~/
git clone https://github.com/Merical/Mecanum_ros.git
cd Mecanum_ros
GIT_DIR=$(pwd)

echo "123456789o" | sudo mv /etc/apt/sources.list /etc/apt/sources.list.org
sudo cp ./sources.list /etc/apt/sources.list
echo "123456789o" | sudo -S apt-get update
sudo apt-get upgrade -y

sudo apt-get install -y libgl1-mesa-dev 
sudo apt-get install -y libglew-dev libsdl2-dev libsdl2-image-dev libglm-dev libfreetype6-dev 
sudo apt-get install -y libglfw3-dev libglfw3 libusb-1.0.-* libgtk-3-dev
sudo apt-get install -y libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install -y python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev

REALSENSE_VERSION=v2.16.1
cd ~/Downloads
curl --silent --location --location-trusted --remote-name https://github.com/IntelRealSense/librealsense/archive/$REALSENSE_VERSION.tar.gz
tar zxvf $REALSENSE_VERSION.tar.gz -C ~/Sources
cd ~/Sources/librealsense-$REALSENSE_VERSION
mkdir build && cd build
cmake ..
make -j4
sudo make -j4 install

OPENCV_VERSION=3.4.6
cd ~/Downloads
curl --silent --location --location-trusted --remote-name https://github.com/opencv/opencv_contrib/archive/$OPENCV_VERSION.tar.gz
mv $OPENCV_VERSION.tar.gz contrib.$OPENCV_VERSION.tar.gz
curl --silent --location --location-trusted --remote-name https://github.com/opencv/opencv/archive/$OPENCV_VERSION.tar.gz
tar zxvf $OPENCV_VERSION.tar.gz -C ~/Sources
tar zxvf contrib.$OPENCV_VERSION.tar.gz -C ~/Sources
cd ~/Sources/opencv-$OPENCV_VERSION
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
cp -r ${GIT_DIR}/src ./

sudo cp src/Documentation/sc_udev/* /etc/udev/rules.d/
sudo /etc/init.d/udev restart

sudo usermod -a -G dialout $USER
sudo apt-get install -y ros-kinetic-turtlebot-bringup ros-kinetic-openni-* ros-kinetic-openni2-* ros-kinetic-freenect-* ros-kinetic-usb-cam ros-kinetic-laser-* ros-kinetic-audio-common ros-kinetic-slam-gmapping ros-kinetic-joystick-drivers python-rosinstall ros-kinetic-orocos-kdl ros-kinetic-python-orocos-kdl python-setuptools ros-kinetic-dynamixel-motor-*  ros-kinetic-depthimage-to-laserscan ros-kinetic-arbotix-* ros-kinetic-turtlebot-teleop ros-kinetic-turtlebot-* ros-kinetic-move-base ros-kinetic-map-server ros-kinetic-fake-localization ros-kinetic-amcl ros-kinetic-dwa-local-planner git subversion mercurial ntpdate ntp vim

sudo apt-get install -y mate-terminal

sudo apt-get install -y ros-kinetic-frontier-exploration ros-kinetic-ros-controllers ros-kinetic-joint-state-* ros-kinetic-controller-manager ros-kinetic-move-base ros-kinetic-web-video-server

cd ${CATKIN_DIR}
catkin_make

echo "source /home/obt-sc/ros_workspace/SC0_ws/devel/setup.bash" >> ~/.bashrc 
source ~/.bashrc
