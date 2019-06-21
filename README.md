# Mecanum_ros
Ros package for a mecanum wheel mobile platform with gazebo simulation, navigation, slam, object tracking, etc

# Usage
ros_workspace: SC0_ws

complie&build:
	cd ~/ros_workspace/SC0_ws/
	catkin_make

mapping:
	roslaunch handsfree_hw sc_hw.launch
	roslaunch rplidar_ros rplidar.launch
	roslaunch sc_2dnav gmapping.launch
	rosrun rviz rviz -d `rospack find sc_2dnav`/rviz/HANDSFREE_Robot.rviz
	roslaunch handsfree_hw mecanum_keyboard.launch

	roscd sc_2dnav/map/
	rosrun map_server map_saver -f my_map (on pc)

navigation:
	roslaunch handsfree_hw sc_hw.launch
	roslaunch rplidar_ros rplidar.launch
	roslaunch sc_2dnav demo_move_base_amcl.launch map_name:=my_map

rtabmap:
	roslaunch handsfree_hw sc_hw.launch
	roslaunch rplidar_ros rplidar.launch
	roslaunch realsense2_camera rs_camera.launch align_depth:=true

	mapping:
		roslaunch sc_2dnav demo_sc_rtab_mapping.launch args:="--delete_db_on_start"
		roslaunch sc_2dnav demo_sc_rtab_rviz.launch

	navigation:
		roslaunch sc_2dnav demo_sc_rtab_mapping.launch localization:=true
		roslaunch sc_2dnav demo_sc_rtab_rviz.launch

audio:
	roslaunch handsfree_hw sc_hw.launch
	roslaunch ocean_audio ocean_audio.launch  (pc)   
	recognize.py  (pc)

visual_tracking:
	roslaunch handsfree_hw sc_hw.launch
	roslaunch rplidar_ros rplidar.launch
	roslaunch realsense2_camera rs_camera.launch align_depth:=true
	roslaunch ocean_vision cmt_tracker_ros_nuc.launch

simulation(gazebo):
	roslaunch sc_gazebo demo_gazebo_sc0.launch
	roslaunch sc_gazebo demo_move_base_amcl.launch

ps:
	如果发生realsense节点报错"Frame didn't arrive for 5 seconds",停止realsense节点重新拔插摄像头并重启节点
