# 1 Introduction
本文档对Oceanbotech全向智能移动平台ROS相关部分进行说明。

Oceanbotech SmartCar V1.0 是一款面向STEAM教育，机器人研究、开发的开源软硬件系统，搭载了麦克纳姆轮的全向智能移动平台，该ROS机器人平台搭载Realsense D435i 深度摄像头，rplidar a2激光雷达，IMU九轴传感器模块，电机编码器等硬件模块，支持里程计、2D SLAM、3D VSLAM、视觉跟踪、视觉循迹等ros功能包，并拥有完备的用户界面。

在开始使用Oceanbotech SmartCar V1.0之前，建议您:
- 确保已经在Ubuntu 16.04上安装ROS操作系统并仔细阅读[初学者文档](http://wiki.ros.org/ROS/Tutorials)
- 遵循Tutorial安装依赖安装包并编译SmartCar ROS工作空间，配置环境变量
- 运行示例  对SmartCar进行远程键盘操控
```
    roslaunch handsfree_hw sc_hw.launch
    roslaunch handsfree_hw mecanum_keyboard.launch
```

# 2 Tutorial

## 2.1 Preparing&Setup

    # copy the udev rules in src/Documentation/sc_udev to /etc/udev/rules.d
    sh setup_environment.sh

## 2.2 Complie&Build:
    # put the Mecanum_ros/src inside your workspace, for example: ~/ros_workspace/SC0_ws
	cd ~/ros_workspace/SC0_ws/
	catkin_make

## 2.3 Ros Remote Connection:

on PC (add following lines to ~/.bashrc)

	export ROS_MASTER_URI=http://SERVER_IP_ADDRESS:11311
	export ROS_HOSTNAME=PC_IP_ADDRESS
	
on Server (add following lines to ~/.bashrc)

	export ROS_MASTER_URI=http://SERVER_IP_ADDRESS:11311
	export ROS_HOSTNAME=SERVER_IP_ADDRESS

## 2.4 2D LIDAR SLAM

	roslaunch handsfree_hw sc_hw.launch
	roslaunch rplidar_ros rplidar.launch

### 2.4.1 Mapping:

	roslaunch sc_2dnav gmapping.launch
	rosrun rviz rviz -d `rospack find sc_2dnav`/rviz/HANDSFREE_Robot.rviz
	roslaunch handsfree_hw mecanum_keyboard.launch
	roscd sc_2dnav/map/
	rosrun map_server map_saver -f your_map_name (on pc)
	
<div align=center><img src="https://github.com/Merical/Mecanum_ros/blob/master/images/mapping.png" width="50%"></div>
	
### 2.4.2 Navigation:

	roslaunch sc_2dnav demo_move_base_amcl.launch map_name:=your_map_name
<div align=center><img src="https://github.com/Merical/Mecanum_ros/blob/master/images/navigation.png" width=320 height=240></div>
	
## 2.5 Rtabmap 3D VSLAM:

	roslaunch handsfree_hw sc_hw.launch
	roslaunch rplidar_ros rplidar.launch
	roslaunch realsense2_camera rs_camera.launch align_depth:=true

### 2.5.1 mapping:

	roslaunch sc_2dnav demo_sc_rtab_mapping.launch args:="--delete_db_on_start" (nuc)
	roslaunch sc_2dnav demo_sc_rtab_rviz.launch (pc)

### 2.5.2 navigation:

	roslaunch sc_2dnav demo_sc_rtab_mapping.launch localization:=true (nuc)
	roslaunch sc_2dnav demo_sc_rtab_rviz.launch (pc)
	
<div align=center><img src="https://github.com/Merical/Mecanum_ros/blob/master/images/rtabmap.png" width=240 height=320></div>

## 2.6 Audio:

	roslaunch handsfree_hw sc_hw.launch
	roslaunch ocean_audio ocean_audio.launch  (pc/nuc)   
	recognize.py  (pc)

## 2.7 Visual_tracking:

	roslaunch handsfree_hw sc_hw.launch
	roslaunch realsense2_camera rs_camera.launch align_depth:=true
	roslaunch ocean_vision cmt_tracker_mecanum_remote.launch
	
<div align=center><img src="https://github.com/Merical/Mecanum_ros/blob/master/images/visual_tracking.png" width=320 height=240></div>

## 2.8 Simulation(gazebo):

	roslaunch sc_gazebo demo_gazebo_sc0.launch
	roslaunch sc_gazebo demo_move_base_amcl.launch
	
<div align=center><img src="https://github.com/Merical/Mecanum_ros/blob/master/images/simulation.png" width=320 height=240></div>
	
## 2.9 SC_GUI
### Python3
Download weights and demo pictures from [Baidu Yun Link](https://pan.baidu.com/s/1T7QvCqoxyCtAedOI4d67PA)

Extract data/ and weights/ folder to the sc_gui_py3 dir.
	
	# connect the robot wifi
	roslaunch ocean_audio server_ros.launch (nuc)
	cd sc_gui_py3 (pc)
	python OB_SC_GUI1.0.py (pc)


# 3 ROS Packakges
## 3.1 handsfree_hw
### 1) Overview
handsfree_hw 是机器人嵌入式软件系统与工控机进行通讯的ros package，其中包含了串口通讯，姿态解算，传感器数据上报，命令数据下发，里程计信息发布，机器人控制命令接受等等，以询问的策略与移动平台建立通讯．
```
sequenceDiagram
ROS->>MCU: Set Robot Command
ROS->>MCU: Read Robot Status
```

### 2) Sample Usage
为了能够快速启动移动平台与ROS系统的连接，可以使用下面的命令启动驱动结点并获取里程计信息

    roslaunch handsfree_hw sc_hw.launch
如果需要使用键盘控制全向移动平台，则可以使用下列命令：
    
    roslaunch handsfree_hw mecanum_keyboard.launch

### 3) Nodes
#### handsfree_hw_node
驱动全向智能移动平台ros节点
##### Subscribed Topics
/mobile_base/mobile_base_controller/cmd_vel([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))\
&emsp;移动平台运动速度控制话题，接受机器人移动速度指令
##### Published Topics
/mobile_base/mobile_base_controller/odom([nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html))\
&emsp;移动平台通过编码器计算得到的里程计信息

/handsfree/imu_data([sensor_msgs/Imu](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html))\
&emsp;移动平台通过九轴传感器得到的IMU姿态信息

/handsfree/robot_state(sc_msgs)\
&emsp;移动平台上报的底层状态信息，包括系统时间，电池电量等等

##### Parameters
~odom_linear_scale_correction(double, default: 1.0)\
&emsp;里程计线性移动误差矫正系数\
~odom_angle_scale_correction(double, default: 1.0)\
&emsp;里程计转动误差矫正系数\
~serial_port(string, default: "/dev/SCRobot")\
&emsp;机器人移动平台USB绑定端口\
~base_mode(string, default: "4omni-wheel")\
&emsp;移动平台机械结构类型\
~with_arm(bool, default: False)\
&emsp;是否搭载机械臂\
~controller_freq(double, default: 100)\
&emsp;移动平台刷新频率

#### mecanum_teleop_key
使用键盘控制全向智能移动平台ros节点
##### Subscribed Topics
None
##### Published Topics
/mobile_base/mobile_base_controller/cmd_vel([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))\
&emsp;移动平台运动速度控制话题，接受机器人移动速度指令

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
    SHAKING_HANDS 　        //检查与MCU的通讯握手，确保建立连接
    READ_SYSTEM_INFO　      //读取MCU的系统信息，包括运行时间，电量等
    SET_ROBOT_SPEED 　      //设置机器人的移动速度
    READ_ROBOT_SPEED 　     //读取机器人的移动速度
    READ_GLOBAL_COORDINATE　//读取机器人的全局坐标信息
    READ_IMU_FUSION_DATA    //读取IMU传感器数据
    READ_INTF_MODE          //读取机器人的控制权
    SET_INTF_MODE           //设置机器人的控制权
    READ_MODULE_CONFIG      //读取机器人的模块配置
    READ_SONAR_DATA         //读取机器人超声波传感器数局
    SET_SONAR_STATE         //设置机器人超声波传感器使能
    CLEAR_ODOMETER_DATA     //清除机器人坐标信息
```

## 3.2 rplidar
### 1) Overview
激光雷达主要应用在建图、导航、目标跟踪等方面，是以串口形式连接到移动平台上依赖包：两台电脑间通过ROS实现通信，激光雷达相关数据也通过ROS传输到主机中
### 2) Usage

```
    roslaunch rplidar_ros rplidar.launch
```

### 3) Nodes
#### rplidarNode
驱动rplidar_a2并发布扫描数局
##### Subscribed Topics
None
##### Published Topics
/scan([sensor_msgs/LaserScan](http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/LaserScan.html))
##### Parameters
serial_port(string，default: "/dev/rplidar"）\
&emsp;系统中使用的串行端口名称\
serial_baudrate(int，default: 115200)\
&emsp;串口波特率\
frame_id(String，default=laser_frame）\
&emsp;设备的坐标系名称\
inverted (bool, default: false)\
&emsp;指示激光雷达是否倒置安装\
angle_compensate (bool, default: false)\
&emsp;是否需要进行角度补偿\
scan_mode (string, default: std::string())\
&emsp;激光雷达的扫描模式.

## 3.3 realsense2_camera
### 1) Overview
Realsense相机主要应用在三维建图、导航及目标跟踪等功能中，realsense中相关信息也通过主题发布。
深度图像、RGB图像等相关主题
### 2) Usage

```
    roslaunch realsense2_camera rs_camera.launch aligned_depth:=true
```

### 3) Nodes
#### realsense2_camera_nodelet
驱动realsense D435 并发布图像数据
##### Subscribed Topics
None
##### Published Topics
###### Color camera
/camera/color/camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/CameraInfo.html))\
&emsp;相机矫正和元数据\
/camera/color/image_raw ([sensor_msgs/Image](http://docs.ros.org/jade/api/sensor_msgs/html/msg/Image.html))\
&emsp;相机拍摄的彩色图像，格式为RGB.
###### Depth camera
/camera/depth/camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/CameraInfo.html))\
&emsp;相机矫正和元数据\
/camera/depth/image_raw ([sensor_msgs/Image](http://docs.ros.org/jade/api/sensor_msgs/html/msg/Image.html))\
&emsp;相机拍摄的深度图像，像素值为uint16的深度值.\
/camera/aligned_depth_to_color/image_raw ([sensor_msgs/Image](http://docs.ros.org/jade/api/sensor_msgs/html/msg/Image.html))\
&emsp;经过适应RGB图像视角矫正的深度图像，像素值为uint16的深度值.\
##### Parameters
align_depth (bool, default: false)\
&emsp;指示是否使用矫正后的深度图像\

更多参数及更多功能请见[realsense_ros](https://github.com/IntelRealSense/realsense-ros)

## 3.4 ocean_vision
### 1) Overview
Oceanbotech视觉跟踪ros包，使用CMT算法与pid控制算法实现智能移动平台跟踪功能

### 2) Usage
```
	roslaunch handsfree_hw sc_hw.launch
	roslaunch realsense2_camera rs_camera.launch align_depth:=true
	roslaunch ocean_vision cmt_tracker_mecanum_remote.launch
```

## 3.5 sc_2dnav
### 1) Overview
Oceanbotech二维导航功能包\
使用[gmapping](http://wiki.ros.org/gmapping/)算法进行地图简历\
使用[move_base](http://wiki.ros.org/move_base/)和[amcl](http://wiki.ros.org/amcl)进行实时导航\
使用[rtabmap](http://wiki.ros.org/rtabmap_ros)进行2维和3维的建图和导航

### 2) Usage


    //Mapping:
	roslaunch sc_2dnav gmapping.launch
	rosrun rviz rviz -d `rospack find sc_2dnav`/rviz/HANDSFREE_Robot.rviz
	roslaunch handsfree_hw mecanum_keyboard.launch
	roscd sc_2dnav/map/
	rosrun map_server map_saver -f your_map_name (on pc)

    //Navigation:
    roslaunch sc_2dnav demo_move_base_amcl.launch map_name:=your_map_name

