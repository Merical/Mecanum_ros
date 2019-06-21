#!/usr/bin/python
# -*- coding: utf-8 -*-

import time
import socket
import rospy
from std_msgs.msg import String

######################tcp begining
HOST = '127.0.0.1'
PORT = 8080
BUFFER = 4096
sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((HOST,PORT))
sock.listen(5)

################ros begining
rospy.init_node('tcptalker', anonymous=0)
pub = rospy.Publisher('tcptopic', String, queue_size=10)

clientsocket, addr = sock.accept()
print('Waiting for message ...')
while True:
    data = clientsocket.recv(1024)
    message = data.decode()

clientsocket.close()