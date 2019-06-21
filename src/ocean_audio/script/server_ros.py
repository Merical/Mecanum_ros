#!/usr/bin/python
# -*- coding: utf-8 -*-

import socket
import re
import rospy
import time
import Queue
import multiprocessing
import roslaunch
import threading
import json
from sc_msgs.msg import vision_rect as Rect
from geometry_msgs.msg import Twist


def communication_job():
    global cmd_queue
    global init_queue
    # global clientsocket

    ip_port = ('192.168.1.145', 8899)
    sk = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    sk.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sk.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    sk.bind(ip_port)
    sk.listen(5)
    clientsocket, addr = sk.accept()
    print('LCH: Socket initialized, processes begin')

    while not rospy.is_shutdown():

        data = clientsocket.recv(1024)
        message = data.decode()
        if len(message) > 0:
            print 'LCH: recv done, message is ', message

            pattern = re.compile(r'\{"COM".*\}\}\}')
            slices = re.findall(pattern, message)

            if len(slices) > 0:
                message_dict = json.loads(slices[-1])
                cmd_queue.append(message_dict['COM']['CMD'])
                node_queue.append(message_dict['COM']['ROS'])
                init_queue.append(message_dict['COM']['INIT'])
        time.sleep(0.005)
    clientsocket.close()

def initialize_job():
    global init_queue
    global init_pub

    def get_rect(rect, cmd):
        rect.topleft_x = cmd['tl'][0]
        rect.topleft_y = cmd['tl'][1]
        rect.bottomright_x = cmd['br'][0]
        rect.bottomright_y = cmd['br'][1]

        return rect

    rect = Rect()
    last_cmd = None
    cmd = None
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if len(init_queue) > 0:
            while len(init_queue) > 0:
                cmd = init_queue.popleft()

            if cmd != last_cmd and cmd['tl'] != [0, 0] and cmd['br'] != [0, 0]:
                rect = get_rect(rect, cmd)
                init_pub.publish(rect)

        rate.sleep()


def motion_job():
    global cmd_queue
    global pub

    def get_twist(twist, cmd):

        twist.linear.x = cmd['linear_x']
        twist.linear.y = cmd['linear_y']
        twist.linear.z = cmd['angular_z']

        return twist

    twist = Twist()
    count = 0
    last_cmd = None
    cmd = None
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if len(cmd_queue) > 0:
            while len(cmd_queue) > 0:
                cmd = cmd_queue.popleft()
            # print('LCH: the cmd is ', cmd)
            last_cmd = cmd
            twist = get_twist(twist, cmd)
            count = 0

        else:
            if last_cmd:
                twist = get_twist(twist, last_cmd)
            else:
                twist = Twist()
            count += 1

        if count > 10:
            twist = Twist()

        pub.publish(twist)
        rate.sleep()
    pub.publish(Twist())


def node_job():
    global node_queue

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    platform_launch = roslaunch.parent.ROSLaunchParent(
        uuid, ["/home/lishenghao/ros_workspace/SC0_ws/src/handsfree_hw/launch/sc_hw.launch"]
    )
    realsense_launch = roslaunch.parent.ROSLaunchParent(
        uuid, ["/home/lishenghao/ros_workspace/SC0_ws/src/realsense-2.1.0/realsense2_camera/launch/rs_camera.launch"]
    )
    lidar_launch = roslaunch.parent.ROSLaunchParent(
        uuid, ["/home/lishenghao/ros_workspace/SC0_ws/src/rplidar_ros/launch/rplidar.launch"]
    )
    cmt_launch = roslaunch.parent.ROSLaunchParent(
        uuid, ["/home/lishenghao/ros_workspace/SC0_ws/src/ocean_vision/launch/cmt_tracker_mecanum_remote.launch"]
    )
    video_launch = roslaunch.parent.ROSLaunchParent(
        uuid, ["/home/lishenghao/ros_workspace/SC0_ws/src/ocean_vision/launch/video_transfor.launch"]
    )
    print('LCH: the launch initialized')

    last_state = {'realsense': 0, 'platform': 0, 'lidar': 0, 'cmt': 0}
    state = None

    while not rospy.is_shutdown():
        if len(node_queue) > 0:
            while len(node_queue) > 0:
                state = node_queue.popleft()

            if state != last_state:
                if state['realsense'] == 1 and last_state['realsense'] == 0:
                    realsense_launch.start()
                elif state['realsense'] == 0 and last_state['realsense'] == 1:
                    realsense_launch.shutdown()
                elif state['platform'] == 1 and last_state['platform'] == 0:
                    platform_launch.start()
                elif state['platform'] == 0 and last_state['platform'] == 1:
                    platform_launch.shutdown()
                elif state['lidar'] == 1 and last_state['lidar'] == 0:
                    lidar_launch.start()
                elif state['lidar'] == 0 and last_state['lidar'] == 1:
                    lidar_launch.shutdown()
                elif state['cmt'] == 1 and last_state['cmt'] == 0:
                    cmt_launch.start()
                elif state['cmt'] == 0 and last_state['cmt'] == 1:
                    cmt_launch.shutdown()
                elif state['video'] == 1 and last_state['video'] == 0:
                    video_launch.start()
                elif state['video'] == 0 and last_state['video'] == 1:
                    video_launch.shutdown()

                last_state = state

    realsense_launch.shutdown()
    cmt_launch.shutdown()
    lidar_launch.shutdown()
    platform_launch.shutdown()


if __name__ == '__main__':
    print('ocean_gui_communication begin')
    cmd_queue = Queue.deque()
    node_queue = Queue.deque()
    init_queue = Queue.deque()
    # speed = 0.2

    rospy.init_node('ocean_gui_communicaton')
    pub = rospy.Publisher('/server_ros/cmd_vel', Twist, queue_size=5)
    init_pub = rospy.Publisher('/server_ros/initial_rect', Rect, queue_size=5)

    motion_process = threading.Thread(target=motion_job)
    # node_process = threading.Thread(target=node_job)
    communication_process = threading.Thread(target=communication_job)
    initialize_process = threading.Thread(target=initialize_job)

    print('thread initialized')

    communication_process.start()
    motion_process.start()
    initialize_process.start()
    node_job()

    rospy.spin()

