#!/usr/bin/python
# -*- coding: utf-8 -*-

import socket
import re
import sys
import os
import time
import Queue
import multiprocessing
import roslaunch
import threading
import json
from sc_msgs.msg import vision_rect as Rect
from geometry_msgs.msg import Twist

def CutIntactMessage(message, depth=1):
    if len(message) == 0:
        return ''
    begin = None
    end = None
    count = 0
    max = 0
    index = 0
    output = ''
    while index < len(message):
        if message[index] == "{":
            if count == 0:
                begin = index
            count += 1
        elif message[index] == "}" and begin is not None:
            count -= 1
            if count == 0:
                end = index
        index += 1
        max = count if count > max else max
        if begin is not None and end is not None and count == 0:
            if max == depth:
                output = message[begin:end + 1]
            else:
                begin = None
                end = None

    return output

def communication_job():
    global shutdown_flag
    # global clientsocket

    ip_port = ('192.168.10.11', 8900)
    sk = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    sk.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sk.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    sk.bind(ip_port)
    sk.listen(5)
    serverSocket, addr = sk.accept()
    print('LCH: Socket initialized, processes begin')

    while True:
        data = serverSocket.recv(1024)
        message = data.decode()
        if len(message) > 0:
#print 'LCH: recv done, message is ', message

            slice = CutIntactMessage(message)

            if len(slice) > 0:
                message_dict = json.loads(slice)
                for _ in range(3):
                    node_queue.append(message_dict)

        response = json.dumps({'SHUTDOWN': shutdown_flag})
#        print('Reponse is ', response)
        serverSocket.send(response.encode('utf-8'))
        time.sleep(0.005)
    print 'communication_job drop out the loop'
    time.sleep(0.5)
    serverSocket.close()
    del serverSocket
    exit(0)

def node_job():
    global node_queue
    global shutdown_flag

    while True:
        if len(node_queue) > 0:
            while len(node_queue) > 0:
                state = node_queue.popleft()
            
            if state['SHUTDOWN'] == 1:
                shutdown_flag = True
                time.sleep(0.5)
                break

        time.sleep(0.005)

    print 'node_job drop out the loop'
    if shutdown_flag:
        os.system("echo %s | sudo -S shutdown -h now" % sudopw)
    exit(0)


if __name__ == '__main__':
    print('ocean_gui_communication begin')
    sudopw = "123456789o"
    shutdown_flag = 1

    node_queue = Queue.deque()
    communication_process = threading.Thread(target=communication_job)
    print('thread initialized')

    communication_process.start()
    node_job()

    time.sleep(1)
    exit(0)

