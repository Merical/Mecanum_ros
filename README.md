# Mecanum_ros
Ros package for a mecanum wheel mobile platform with gazebo simulation, navigation, slam, object tracking, etc

# Usage
ros_workspace: SC0_ws

# Complie&Build:

	cd ~/ros_workspace/SC0_ws/
	catkin_make
	# copy the rules in src/Documentation/sc_udev to /etc/udev/rules.d
	
# Ros Remote Connection:

on PC (add following lines to ~/.bashrc)

	export ROS_MASTER_URI=http://PC_IP_ADDRESS:11311
	export ROS_HOSTNAME=PC_IP_ADDRESS
	
on Server (add following lines to ~/.bashrc)

	export ROS_MASTER_URI=http://PC_IP_ADDRESS:11311
	export ROS_HOSTNAME=SERVER_IP_ADDRESS

# Mapping:

	roslaunch handsfree_hw sc_hw.launch
	roslaunch rplidar_ros rplidar.launch
	roslaunch sc_2dnav gmapping.launch
	rosrun rviz rviz -d `rospack find sc_2dnav`/rviz/HANDSFREE_Robot.rviz
	roslaunch handsfree_hw mecanum_keyboard.launch
	roscd sc_2dnav/map/
	rosrun map_server map_saver -f my_map (on pc)

# Navigation:

	roslaunch handsfree_hw sc_hw.launch
	roslaunch rplidar_ros rplidar.launch
	roslaunch sc_2dnav demo_move_base_amcl.launch map_name:=my_map

# Rtabmap:

	roslaunch handsfree_hw sc_hw.launch
	roslaunch rplidar_ros rplidar.launch
	roslaunch realsense2_camera rs_camera.launch align_depth:=true

## mapping:

	roslaunch sc_2dnav demo_sc_rtab_mapping.launch args:="--delete_db_on_start" (nuc)
	roslaunch sc_2dnav demo_sc_rtab_rviz.launch (pc)

## navigation:

	roslaunch sc_2dnav demo_sc_rtab_mapping.launch localization:=true (nuc)
	roslaunch sc_2dnav demo_sc_rtab_rviz.launch (pc)

# Audio:

	roslaunch handsfree_hw sc_hw.launch
	roslaunch ocean_audio ocean_audio.launch  (pc/nuc)   
	recognize.py  (pc)

# Visual_tracking:

	roslaunch handsfree_hw sc_hw.launch
	roslaunch rplidar_ros rplidar.launch
	roslaunch realsense2_camera rs_camera.launch align_depth:=true
	roslaunch ocean_vision cmt_tracker_ros_nuc.launch

# Simulation(gazebo):

	roslaunch sc_gazebo demo_gazebo_sc0.launch
	roslaunch sc_gazebo demo_move_base_amcl.launch
	
# SC_GUI
## Python3
Download weights and demo pictures from [Baidu Yun Link](https://pan.baidu.com/s/1T7QvCqoxyCtAedOI4d67PA) code h5ev

Extract data/ and weights/ folder to the sc_gui_py3 dir.
	
	# connect the robot wifi
	roslaunch ocean_audio server_ros.launch (nuc)
	cd sc_gui_py3 (pc)
	python OB_SC_GUI1.0.py (pc)

# ps:
	如果发生realsense节点报错"Frame didn't arrive for 5 seconds",停止realsense节点重新拔插摄像头并重启节点
