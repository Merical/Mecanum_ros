import roslaunch
import time
import rospy
import rospkg

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
tracking_launch = roslaunch.parent.ROSLaunchParent(
    uuid, ['/home/lishenghao/ros_workspace/SC0_ws/src/ocean_audio/launch/ocean_audio.launch'])
tracking_launch.start()

time.sleep(5)
print('shutdown')
tracking_launch.shutdown()
time.sleep(1)
print('shutdown')
tracking_launch.shutdown()
