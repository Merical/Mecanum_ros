#!/usr/bin/python
# -*- coding: utf-8 -*-

import socket
import re
import rospy
import time
from std_msgs.msg import String
import json
from geometry_msgs.msg import Twist

speed = 0.4

def get_twist(twist, message):
    global speed

    pattern = re.compile(r'\{"CMD".*\}')
    slices = re.findall(pattern, message)

    if len(slices) > 0:
        cmd = json.loads(slices[-1])
        print('LCH: the cmd is', cmd)

        if cmd["CMD"] == "forward":
            twist = Twist()
            twist.linear.x = speed
        elif cmd["CMD"] == "backward":
            twist = Twist()
            twist.linear.x = -speed
        elif cmd["CMD"] == "turn left":
            twist = Twist()
            twist.linear.x = speed
            twist.angular.z = -speed
        elif cmd["CMD"] == "turn right":
            twist = Twist()
            twist.linear.x = speed
            twist.angular.z = speed
        elif cmd['CMD'] == "rotate left":
            twist = Twist()
            twist.angular.z = -speed
        elif cmd['CMD'] == "rotate right":
            twist = Twist()
            twist.angular.z = speed
        elif cmd['CMD'] == "speed up":
            speed *= 2
            twist.linear.x *= 2
            twist.angular.z *= 2
        elif cmd['CMD'] == "speed up":
            speed /= 2
            twist.linear.x *= 2
            twist.angular.z *= 2
        else:
            twist = Twist()

    return twist


rospy.init_node('server_debug')
# pub = rospy.Publisher('/ocean_audio/command_message', String, queue_size=5)
pub = rospy.Publisher('~cmd_vel', Twist, queue_size=5)
# rate = rospy.Rate(10)

ip_port = ('127.0.0.1',8080)
sk = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
sk.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sk.bind(ip_port)
sk.listen(5)
clientsocket, addr = sk.accept()
print('LCH: Socket initialized')
count = 0
twist = Twist()
last_message = None

while not rospy.is_shutdown():

    data = clientsocket.recv(1024)
    print('LCH: recv done!')
    message = data.decode()
    if len(message) != 0:
        print(message)
        twist = get_twist(twist, message)
        last_message = message
        count = 0
    else:
        twist = get_twist(twist, last_message)
        count += 1

    if count > 20:
        twist = Twist()

    pub.publish(twist)
    time.sleep(0.1)
    # rate.sleep()

clientsocket.close()
