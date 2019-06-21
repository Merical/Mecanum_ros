#!/usr/bin/python
# -*- coding: utf-8 -*-

import socket
import re
import rospy
import time
import Queue
import multiprocessing
import threading
import json
from geometry_msgs.msg import Twist

speed = 0.2

def get_twist(twist, cmd):
    global speed
    local_speed = speed

    if cmd["CMD"] == "forward":
        twist = Twist()
        twist.linear.x = speed
    elif cmd["CMD"] == "backward":
        twist = Twist()
        twist.linear.x = -speed
    elif cmd["CMD"] == "turn left":
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = speed
    elif cmd["CMD"] == "turn right":
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = -speed*2
    elif cmd['CMD'] == "rotate left":
        twist = Twist()
        twist.angular.z = -speed*2
    elif cmd['CMD'] == "rotate right":
        twist = Twist()
        twist.angular.z = speed
    elif cmd['CMD'] == "speed up":
        speed = speed * 2
        speed = min(speed, 0.6)
    elif cmd['CMD'] == "speed down":
        speed = speed / 2
        speed = max(speed, 0.1)
    elif cmd['CMD'] == "stop":
        twist = Twist()
    else:
        twist = Twist()
    # print('LCH: the speed is ', speed)

    return twist


def communication_job():
    global cmd_queue
    # global clientsocket

    ip_port = ('127.0.0.1',8080)
    sk = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    sk.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sk.bind(ip_port)
    sk.listen(5)
    clientsocket, addr = sk.accept()
    print('LCH: Socket initialized, processes begin')

    while not rospy.is_shutdown():

        data = clientsocket.recv(1024)
        message = data.decode()
        if len(message) > 0:
            print('LCH: recv done, message is ', message)

            pattern = re.compile(r'\{"CMD".*\}')
            slices = re.findall(pattern, message)

            if len(slices) > 0:
                cmd_queue.append(slices[-1])
        time.sleep(0.01)
    clientsocket.close()

def motion_job():
    global cmd_queue
    global pub

    twist = Twist()
    count = 0
    last_cmd = None
    cmd = None
    rate = rospy.Rate(10)


    while not rospy.is_shutdown():
        if len(cmd_queue) > 0:
            while len(cmd_queue) > 0:
                cmd = json.loads(cmd_queue.popleft())
            last_cmd = cmd
            twist = get_twist(twist, cmd)
            count = 0

        else:
            if last_cmd:
                if last_cmd['CMD'] == "speed up" or last_cmd['CMD'] == "speed down":
                    twist = Twist()
                else:
                    twist = get_twist(twist, last_cmd)
            else:
                twist = Twist()
            count += 1

        if count > 20:
            twist = Twist()

        pub.publish(twist)
        rate.sleep()

    pub.publish(Twist())

if __name__ == '__main__':
    print('Server debug thread begin')
    cmd_queue = Queue.deque()
    # speed = 0.2

    rospy.init_node('server_debug_thread')
    pub = rospy.Publisher('~cmd_vel', Twist, queue_size=5)

    # motion_process = multiprocessing.Process(target=motion_job)
    # communication_process = multiprocessing.Process(target=communication_job)

    motion_process = threading.Thread(target=motion_job)
    communication_process = threading.Thread(target=communication_job)

    communication_process.start()
    motion_process.start()

    rospy.spin()

