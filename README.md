# Table of Contents
- [1. Introduction](#1-introduction)
- [2. Tutorial](#2-tutorial)
  - [2.1 Preparing & Setup](#21-preparingsetup)
  - [2.2 Compile & Build](#22-compile--build)
  - [2.3 ROS Remote Connection](#23-ros-remote-connection)
  - [2.4 2D LIDAR SLAM](#24-2d-lidar-slam)
  - [2.5 Rtabmap 3D VSLAM](#25-rtabmap-3d-vslam)
  - [2.6 Audio](#26-audio)
  - [2.7 Visual Tracking](#27-visual_tracking)
  - [2.8 Simulation (Gazebo)](#28-simulationgazebo)
  - [2.9 SC_GUI](#29-sc_gui)
  - [2.10 Time Synchronization](#210-time-synchronization)
  - [2.11 Auto Driving Demo](#211-auto-driving-demo)
- [3. ROS Packages](#3-ros-packakges)
  - [3.1 sc_hw](#31-sc_hw)
  - [3.2 rplidar](#32-rplidar)
  - [3.3 realsense2_camera](#33-realsense2_camera)
  - [3.4 ocean_vision](#34-ocean_vision)
  - [3.5 sc_2dnav](#35-sc_2dnav)

# 1 Introduction
This document provides information about the ROS-related aspects of the Oceanbotech Omnidirectional Intelligent Mobile Platform. [Chinese Version](README_CHN.md)

Oceanbotech SmartCar V1.0 is an open-source hardware and software system designed for STEAM education, robotics research and development. It features a mecanum wheel-based omnidirectional mobile platform. This ROS robot platform is equipped with a Realsense D435i depth camera, rplidar a2 lidar, 9-axis IMU sensor module, motor encoders and other hardware modules. It supports ROS packages for odometry, 2D SLAM, 3D VSLAM, visual tracking, visual path following, and comes with a comprehensive user interface.

Before starting to use Oceanbotech SmartCar V1.0, it is recommended that you:
- Ensure ROS is installed on Ubuntu 16.04 and carefully read the [Beginner Tutorials](http://wiki.ros.org/ROS/Tutorials)
- Follow the Tutorial to install dependencies and compile the SmartCar ROS workspace, configure environment variables  
- Run the example to remotely control SmartCar via keyboard

```bash
roslaunch sc_hw sc_hw.launch
roslaunch sc_hw mecanum_keyboard.launch
```

## Environment

- Ubuntu 16.04
- Ros Kinetic

# 2 Tutorial

## 2.1 Preparing&Setup
```bash
# On server NUC
bash setup_from_scratch.sh # if this is a new setup
bash setup_environment_server # if opencv and ros is installed

sudo su
echo "server 127.127.1.0" >> /etc/ntp.conf
echo "fudge 127.127.1.0 stratum 5" >> /etc/ntp.conf
systemctl restart ntp.service

# add the following line to /etc/rc.local, before the "exit 0" line
bash /home/obt-sc/ros_workspace/SC0_ws/src/ocean_audio/script/server_bringup.sh

# On PC
bash setup_pc.sh # Only do this step if you didn't setup your pc envirnment at all. Manual setup is recommanded.
```

## 2.2 Compile & Build:
```bash
# Put the Mecanum_ros/src inside your workspace, for example: ~/ros_workspace/SC0_ws
cd ~/ros_workspace/SC0_ws/
catkin_make
```

## 2.3 ROS Remote Connection:

On PC (add following lines to ~/.bashrc):
```bash
export ROS_MASTER_URI=http://SERVER_IP_ADDRESS:11311
export ROS_HOSTNAME=PC_IP_ADDRESS
```

On Server (add following lines to ~/.bashrc):
```bash
export ROS_MASTER_URI=http://SERVER_IP_ADDRESS:11311
export ROS_HOSTNAME=SERVER_IP_ADDRESS
```

## 2.4 2D LIDAR SLAM

```bash
# Start the robot hardware interface and LIDAR
roslaunch sc_hw sc_hw.launch
roslaunch rplidar_ros rplidar.launch
```

### 2.4.1 Mapping:

```bash
roslaunch sc_2dnav gmapping.launch
rosrun rviz rviz -d `rospack find sc_2dnav`/rviz/HANDSFREE_Robot.rviz
roslaunch sc_hw mecanum_keyboard.launch
roscd sc_2dnav/map/
rosrun map_server map_saver -f your_map_name (on pc)
```
	
<div align=center><img src="https://github.com/Merical/Mecanum_ros/blob/master/images/mapping.png" width=640 height=480></div>
	
### 2.4.2 Navigation:

```bash
roslaunch sc_2dnav demo_move_base_amcl_server.launch map_name:=your_map_name (on nuc)
roslaunch sc_2dnav demo_move_base_amcl_client.launch (on pc)
```
<div align=center><img src="https://github.com/Merical/Mecanum_ros/blob/master/images/navigation.png" width=240 height=320></div>
	
## 2.5 Rtabmap 3D VSLAM:

```bash
roslaunch sc_hw sc_hw.launch
roslaunch rplidar_ros rplidar.launch
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

### 2.5.1 mapping:

```bash
roslaunch sc_2dnav demo_sc_rtab_mapping.launch args:="--delete_db_on_start" (nuc)
roslaunch sc_2dnav demo_sc_rtab_rviz.launch (pc)
roslaunch sc_hw mecanum_keyboard.launch
```

### 2.5.2 navigation:

```bash
roslaunch sc_2dnav demo_sc_rtab_mapping.launch localization:=true (nuc)
roslaunch sc_2dnav demo_sc_rtab_rviz.launch (pc)
```
	
<div align=center><img src="https://github.com/Merical/Mecanum_ros/blob/master/images/rtabmap.png" width=640 height=480></div>

## 2.6 Audio:

```bash
roslaunch sc_hw sc_hw.launch
roslaunch ocean_audio ocean_audio.launch  (pc/nuc)   
recognize.py  (pc)
```

## 2.7 Visual_tracking:

```bash
roslaunch sc_hw sc_hw.launch
roslaunch realsense2_camera rs_camera.launch align_depth:=true
roslaunch ocean_vision cmt_tracker_mecanum.launch
```
	
<div align=center><img src="https://github.com/Merical/Mecanum_ros/blob/master/images/visual_tracking.png" width=320 height=240></div>

## 2.8 Simulation(gazebo):

```bash
roslaunch sc_gazebo demo_gazebo_sc0.launch
roslaunch sc_gazebo demo_move_base_amcl.launch
```
	
<div align=center><img src="https://github.com/Merical/Mecanum_ros/blob/master/images/simulation.png" width=320 height=240></div>
	
## 2.9 SC_GUI
### Python3
Download weights and demo pictures from [Baidu Yun Link](https://pan.baidu.com/s/1T7QvCqoxyCtAedOI4d67PA)

Extract data/ and weights/ folder to the sc_gui_py3 dir.

```bash
# connect the robot wifi
roslaunch ocean_audio server_ros.launch (nuc)
cd sc_gui_py3 (pc)
python gui.py (pc)
```

## 2.10 Time Synchronization
```bash
sudo apt-get install -y ntpdate
sudo ntpdate -u SERVER_IP
```

## 2.11 Auto Driving Demo
Please refer to [Merical/AutoDrive](https://github.com/Merical/AutoDrive)
<div align=center><img src="https://github.com/Merical/AutoDrive/blob/master/Images/signdetection.png" width=640 height=480></div>

# 3 ROS Packakges
## 3.1 sc_hw
### 1) Overview
sc_hw is a ROS package for communication between the robot's embedded software system and the industrial computer. It includes serial communication, attitude calculation, sensor data reporting, command data transmission, odometry information publishing, robot control command reception, etc., establishing communication with the mobile platform through a polling strategy.

### 2) Sample Usage
To quickly establish the connection between the mobile platform and ROS system, use the following command to start the driver node and obtain odometry information:

```bash
roslaunch sc_hw sc_hw.launch
```

If you need to control the omnidirectional mobile platform using keyboard, use the following command:

```bash
roslaunch sc_hw mecanum_keyboard.launch
```

### 3) Nodes
#### sc_hw_node
ROS node for driving the omnidirectional intelligent mobile platform
##### Subscribed Topics
/mobile_base/mobile_base_controller/cmd_vel([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))\
&emsp;Mobile platform motion velocity control topic, receives robot movement velocity commands
##### Published Topics
/mobile_base/mobile_base_controller/odom([nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html))\
&emsp;Odometry information calculated by the mobile platform using encoders

/handsfree/imu_data([sensor_msgs/Imu](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html))\
&emsp;IMU attitude information obtained from the mobile platform's nine-axis sensor

/handsfree/robot_state(sc_msgs)\
&emsp;Low-level status information reported by the mobile platform, including system time, battery level, etc.

##### Parameters
~odom_linear_scale_correction(double, default: 1.0)\
&emsp;Odometry linear movement error correction coefficient\
~odom_angle_scale_correction(double, default: 1.0)\
&emsp;Odometry rotation error correction coefficient\
~serial_port(string, default: "/dev/SCRobot")\
&emsp;Robot mobile platform USB binding port\
~base_mode(string, default: "4omni-wheel")\
&emsp;Mobile platform mechanical structure type\
~with_arm(bool, default: False)\
&emsp;Whether equipped with robotic arm\
~controller_freq(double, default: 100)\
&emsp;Mobile platform refresh rate

#### mecanum_teleop_key
ROS node for controlling the omnidirectional intelligent mobile platform using keyboard
##### Subscribed Topics
None
##### Published Topics
/mobile_base/mobile_base_controller/cmd_vel([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))\
&emsp;Mobile platform motion velocity control topic, receives robot movement velocity commands

### 3) C\C++ Implimentation
```
graph TD
id[main]--Read config files and initialize ros nodehandle, entering mainloop-->id1[HF_HW_ros::mainloop]
id1[HF_HW_ros::mainloop] --Setup communication with MCU, read data and set robot action --> id2[HF_HW::checkHandsShake / HF_HW::UpdateCommand]
id2[HF_HW::checkHandsShake / HF_HW::UpdateCommand] --Update robot state and sensor output, Send command to MCU--> id3[HF_HW_ros::readBufferUpdate/HF_HW_ros::writeBufferUpdate]
id3[HF_HW_ros::readBufferUpdate/HF_HW_ros::writeBufferUpdate] --Update robot data in memory and send data ros controller_manager-->　id4[controller_manager]
```

### 4)  MCU Communication Command Type

```
    SHAKING_HANDS          # Check communication handshake with MCU to ensure connection
    READ_SYSTEM_INFO       # Read MCU system info including runtime, battery level etc.
    SET_ROBOT_SPEED        # Set robot movement speed
    READ_ROBOT_SPEED       # Read robot movement speed
    READ_GLOBAL_COORDINATE # Read robot global coordinate information
    READ_IMU_FUSION_DATA   # Read IMU sensor data
    READ_INTF_MODE         # Read robot control authority
    SET_INTF_MODE          # Set robot control authority
    READ_MODULE_CONFIG     # Read robot module configuration
    READ_SONAR_DATA        # Read robot ultrasonic sensor data
    SET_SONAR_STATE        # Enable/disable robot ultrasonic sensors
    CLEAR_ODOMETER_DATA    # Clear robot coordinate information
```

## 3.2 rplidar
### 1) Overview
The LiDAR is mainly used for mapping, navigation, target tracking and other applications. It connects to the mobile platform via serial port. Dependencies: Communication between two computers is achieved through ROS, and LiDAR-related data is also transmitted to the host through ROS.
### 2) Usage

```bash
roslaunch rplidar_ros rplidar.launch
```

### 3) Nodes
#### rplidar Node
Drive rplidar_a2 and publish scan data
##### Subscribed Topics
None
##### Published Topics
/scan([sensor_msgs/LaserScan](http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/LaserScan.html))
##### Parameters
serial_port(string，default: "/dev/rplidar"）\
&emsp;Serial port name used in the system\
serial_baudrate(int，default: 115200)\
&emsp;Serial port baud rate\
frame_id(String，default=laser_frame）\
&emsp;Coordinate system name of the device\
inverted (bool, default: false)\
&emsp;Indicates whether the lidar is mounted upside down\
angle_compensate (bool, default: false)\
&emsp;Whether angle compensation is needed\
scan_mode (string, default: std::string())\
&emsp;Scanning mode of the lidar.

## 3.3 realsense2_camera
### 1) Overview
The Realsense camera is mainly used for 3D mapping, navigation and target tracking functions. The related information in Realsense is also published through topics.
Topics include depth images, RGB images, etc.
### 2) Usage

```bash
roslaunch realsense2_camera rs_camera.launch aligned_depth:=true
```

### 3) Nodes
#### realsense2_camera_nodelet
Drive realsense D435 and publish image data
##### Subscribed Topics
None
##### Published Topics
###### Color camera
/camera/color/camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/CameraInfo.html))\
&emsp;Camera calibration and metadata\
/camera/color/image_raw ([sensor_msgs/Image](http://docs.ros.org/jade/api/sensor_msgs/html/msg/Image.html))\
&emsp;Color image captured by the camera in RGB format.
###### Depth camera
/camera/depth/camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/CameraInfo.html))\
&emsp;Camera calibration and metadata\
/camera/depth/image_raw ([sensor_msgs/Image](http://docs.ros.org/jade/api/sensor_msgs/html/msg/Image.html))\
&emsp;Depth image captured by the camera, pixel values are uint16 depth values.\
/camera/aligned_depth_to_color/image_raw ([sensor_msgs/Image](http://docs.ros.org/jade/api/sensor_msgs/html/msg/Image.html))\
&emsp;Depth image aligned to RGB image perspective, pixel values are uint16 depth values.\
##### Parameters
align_depth (bool, default: false)\
&emsp;Indicates whether to use aligned depth image\

For more parameters and features, please see [realsense_ros](https://github.com/IntelRealSense/realsense-ros)

## 3.4 ocean_vision
### 1) Overview
Oceanbotech vision tracking ROS package, using CMT algorithm and PID control algorithm to implement intelligent mobile platform tracking functionality

### 2) Usage
```bash
roslaunch sc_hw sc_hw.launch
roslaunch realsense2_camera rs_camera.launch align_depth:=true
roslaunch ocean_vision cmt_tracker_mecanum_remote.launch
```

## 3.5 sc_2dnav
### 1) Overview
Oceanbotech 2D navigation package\
Using [gmapping](http://wiki.ros.org/gmapping/) algorithm for map building\
Using [move_base](http://wiki.ros.org/move_base/) and [amcl](http://wiki.ros.org/amcl) for real-time navigation\
Using [rtabmap](http://wiki.ros.org/rtabmap_ros) for 2D and 3D mapping and navigation

### 2) Usage


```bash
# Mapping:
roslaunch sc_2dnav gmapping.launch
rosrun rviz rviz -d `rospack find sc_2dnav`/rviz/HANDSFREE_Robot.rviz
roslaunch sc_hw mecanum_keyboard.launch
roscd sc_2dnav/map/
rosrun map_server map_saver -f your_map_name (on pc)

# Navigation:
roslaunch sc_2dnav demo_move_base_amcl.launch map_name:=your_map_name
```

