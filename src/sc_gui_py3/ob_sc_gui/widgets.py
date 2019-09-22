import sys
from PyQt5.QtWidgets import QFileDialog, QApplication, QRadioButton, QWidget, QTableWidgetItem, QMainWindow
from PyQt5 import QtCore
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import cv2
import queue
import socket
import re
import json
import time
import numpy as np
from ob_sc_gui.models import *

Debug_flag = True
Debug_Send_flag = True
Debug_Key_flag = False
Debug_video = False
Debug_track = False
Debug_cmt = False
Debug_ros = False

ONLINE_MODE = True
command_queue = queue.Queue()
ROBOT_CONDITION = {}
# ROS_CAMERA_IP_ADDR = "http://192.168.10.11:8080/stream?topic=/camera/color/image_raw"
ROS_CAMERA_IP_ADDR = "http://192.168.10.11:8080/stream?topic=/ob_vision/follower/color"
ROV_CAMERA_IP_ADDR = "rtsp://192.168.10.10:554/user=admin&password=&channel=1&stream=0.sdp?"
OFFLINE_DATA = b'{"RPT":{"YAW":239.7,"SONA1":106,"SONA2":103,"SONA3":204,"SONA4":282,"SONA5": 69,"LINESPD":  0.0,"LYAW":  0.0,"GER":1,"PID":0,"TRC":0,"TRCSEN":0XE0,"ULT":0,"Volt":0,"WZ": -0.06,"TEMP":51.1,"CTRL":"NONE","X":0.24,"Y":0.02,"W":0.7}}'
OFFLINE_DATA_ros = b'{"ROS":{"sc_hw": 0, "lidar": 0, "realsense": 0, "cmt": 0}, "CMD":{"linear_x":0.6, "linear_y":0.8, "angular_z":0.2}}'

class ThreadTemplate(QThread):
    def __init__(self):
        super().__init__()
        self.sock = None
        self.working = True

    def setSock(self, sock):
        self.sock = sock

    def cancelSock(self):
        self.sock = None

    def disable(self):
        self.working = False

    def enable(self):
        self.working = True

def to_json(str):
    return json.dumps(str)

# --------------------------------------------------------Pad Start Here------------------------------------------------- #


class Window_pad(QWidget):
    def __init__(self, mainWindow):
        super().__init__()

        if not ONLINE_MODE:
            self.sock = 0

        self.mainWindow = mainWindow
        self.cmd_thread = AnalyzeCommand()
        self.update_thread = UpdateData()
        # self.video_thread = VideoData()
        self.video_thread = VideoThread(ROV_CAMERA_IP_ADDR)
        self.track_thread = TrackThread()
        self.cam_show_flag = True
        self.track_mode = False
        self.connected = False
        self.initUI()

    def initUI(self):

        main_layout = QtWidgets.QHBoxLayout()
        fun_button = QtWidgets.QVBoxLayout()
        socket_layout = QtWidgets.QHBoxLayout()
        self.data_table = self.get_data_table()

        # camera_layout = QtWidgets.QVBoxLayout()

        self.label_show_camera = QtWidgets.QLabel()
        self.label_show_camera.setMinimumSize(641, 481)
        self.label_show_camera.setBaseSize(641, 481)
        self.label_show_camera.setAutoFillBackground(False)

        self.button_connect_robot = QtWidgets.QPushButton(u'Connect Robot')
        self.button_connect_robot.setBaseSize(120, 30)
        socket_layout.addWidget(self.button_connect_robot)
        self.button_connect_robot.clicked.connect(self.connect_socket_on_click)

        self.text_ip_address = QtWidgets.QLineEdit()
        # self.text_ip_address.setText('127.0.0.1')
        self.text_ip_address.setText('192.168.10.200')
        self.text_ip_address.setBaseSize(200, 30)
        socket_layout.addWidget(self.text_ip_address)

        fun_button.addLayout(socket_layout)

        self.button_open_camera = QtWidgets.QPushButton(u'Open Camera')
        self.button_open_camera.setBaseSize(120, 50)
        fun_button.addWidget(self.button_open_camera)
        self.button_open_camera.clicked.connect(self.open_camera_on_click)

        self.button_open_track = QtWidgets.QPushButton(u'Open Track')
        self.button_open_track.setBaseSize(120, 50)
        fun_button.addWidget(self.button_open_track)
        self.button_open_track.clicked.connect(self.open_track_on_click)

        button_close = QtWidgets.QPushButton(u'Quit')
        button_close.setBaseSize(120, 50)
        fun_button.addWidget(button_close)
        button_close.clicked.connect(self.quit_to_mainWindow_on_click)

        fun_button.addWidget(self.data_table)

        main_layout.addLayout(fun_button)
        main_layout.addWidget(self.label_show_camera)
        self.setLayout(main_layout)
        self.update_thread.update_date.connect(self.update_item_data)
        self.video_thread.frame_data.connect(self.show_img)

        self.show()
        print('GUI initiated')

    def quit_to_mainWindow_on_click(self):
        self.close()
        self.mainWindow.restart()


    def get_data_table(self):
        data_table = QtWidgets.QTableWidget()
        data_table.setMaximumSize(300, 400)
        data_table.setColumnCount(1)
        data_table.setRowCount(8)
        col_name = [
            'Data',

        ]
        data_table.setHorizontalHeaderLabels(col_name)
        row_name = [
            'GER',
            'PID',
            'SONA1',
            'SONA3',
            'SONA5',
            'SONA7',
            'SONA8',
            'TRACK_SENSOR',
        ]
        data_table.setVerticalHeaderLabels(row_name)
        return data_table

    def update_item_data(self, x,y,z,a,b,c,d,e):
        self.data_table.setItem(0, 0, QTableWidgetItem(x))
        self.data_table.setItem(1, 0, QTableWidgetItem(y))
        self.data_table.setItem(2, 0, QTableWidgetItem(z))
        self.data_table.setItem(3, 0, QTableWidgetItem(a))
        self.data_table.setItem(4, 0, QTableWidgetItem(b))
        self.data_table.setItem(5, 0, QTableWidgetItem(c))
        self.data_table.setItem(6, 0, QTableWidgetItem(d))
        self.data_table.setItem(7, 0, QTableWidgetItem(e))

    def keyPressEvent(self, event):
        if not self.track_mode:
            key = event.key()
            if Debug_Key_flag: print("pressed:" + str(key))
            if key == Qt.Key_Escape:
                self.close()
            elif key == Qt.Key_S:
                command_queue.put('command_type:0')  # stop
            elif key == Qt.Key_W:
                command_queue.put('command_type:1')  # forward
            elif key == Qt.Key_X:
                command_queue.put('command_type:2')  # backward
            elif key == Qt.Key_D:
                command_queue.put('command_type:3')  # right
            elif key == Qt.Key_A:
                command_queue.put('command_type:4')  # left
            elif key == Qt.Key_E:
                command_queue.put('command_type:5')  # forward right
            elif key == Qt.Key_Q:
                command_queue.put('command_type:6')  # forward left
            elif key == Qt.Key_C:
                command_queue.put('command_type:7')  # back right
            elif key == Qt.Key_Z:
                command_queue.put('command_type:8')  # back left
            elif key == Qt.Key_L:
                command_queue.put('command_type:9')  # turn right
            elif key == Qt.Key_J:
                command_queue.put('command_type:10')  # turn left
            elif key == Qt.Key_U:
                command_queue.put('command_type:11')  # turn right & forward
            elif key == Qt.Key_O:
                command_queue.put('command_type:12')  # turn left & forward
        else:
            pass


    def keyReleaseEvent(self, a0: QtGui.QKeyEvent):
        if not self.track_mode:
            command_queue.put('command_type:0')
        else:
            pass

    def open_camera_on_click(self):
        if Debug_video: print('LCH: open_camera initiated.')
        if self.connected:
            if self.cam_show_flag:
                self.video_thread.start()
                self.video_thread.cam_timer.start(30)
                self.button_open_camera.setText(u'Close Camera')
                self.cam_show_flag = False
            else:
                if Debug_video: print('LCH: open_camera_on_click: 5')
                self.video_thread.cam_timer.stop()
                self.cam_show_flag = True
                self.video_thread.quit()
                # self.video_thread.wait()
                self.label_show_camera.clear()
                self.button_open_camera.setText(u'Open Camera')
        else:
            pass

    def open_track_on_click(self):
        if Debug_track: print('LCH: open_track initiated.')
        if self.connected:
            if self.track_mode:
                self.track_mode = False
                self.track_thread.disableTrack()
                self.track_thread.quit()
                self.cmd_thread.setSpeed(100)
                self.button_open_track.setText(u'Open Track')
                # [command_queue.put('command_type:21') for _ in range(3)]

            else:
                # [command_queue.put('command_type:20') for _ in range(3)]
                self.track_mode = True
                self.track_thread.enableTrack()
                self.track_thread.start()
                self.cmd_thread.setSpeed(25)  # set the track mode basic speed to 30
                self.button_open_track.setText(u'Close Track')


    def show_img(self, frame):
        if Debug_video: print('LCH: Main thread show img.')
        showImage = QtGui.QImage(frame.data, frame.shape[1], frame.shape[0], QtGui.QImage.Format_RGB888)
        self.label_show_camera.setPixmap(QtGui.QPixmap.fromImage(showImage))

    def connect_socket_on_click(self):
        global ONLINE_MODE
        if not self.connected:
            ip_address = self.text_ip_address.text()
            if ip_address == '127.0.0.1':
                self.sock = 0
                ONLINE_MODE = False
            else:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 创建一个socket
                self.sock.connect((ip_address, 8899))  # 建立连接
                ONLINE_MODE = True

            print('LCH: the sock is set')

            self.cmd_thread.setSock(self.sock)
            self.update_thread.setSock(self.sock)
            self.video_thread.setSock(self.sock)

            self.cmd_thread.start()
            self.update_thread.start()
            self.button_connect_robot.setText(u'Disconnect Robot')

            self.connected = True

        else:
            self.cmd_thread.disable()
            self.cmd_thread.quit()
            self.cmd_thread.wait()

            self.update_thread.disable()
            self.update_thread.quit()
            self.update_thread.wait()

            self.video_thread.disable()
            self.video_thread.quit()
            self.video_thread.wait()

            del self.cmd_thread
            del self.update_thread
            del self.video_thread

            print('LCH: delete done')

            self.cmd_thread = AnalyzeCommand()
            self.update_thread = UpdateData()
            self.video_thread = VideoThread(ROV_CAMERA_IP_ADDR)
            # self.video_thread = VideoData()

            self.update_thread.update_date.connect(self.update_item_data)
            self.video_thread.frame_data.connect(self.show_img)

            self.button_connect_robot.setText(u'Connect Robot')
            self.connected = False


class TrackThread(ThreadTemplate):
    def __init__(self):
        super(TrackThread, self).__init__()
        self.track = False

    def run(self):
        if Debug_track: print('LCH: Track thread begin ...')
        while self.working:
            if self.track:
                sensor = ROBOT_CONDITION['TRCSEN_Data']
                assert len(sensor) == 5, 'Track sensor data error'
                print('LCH: the track sensor data is ', sensor)
                if sensor == [1, 1, 1, 1, 1]:
                    command_queue.put('command_type:0')
                    continue
                l, r = 0, len(sensor) - 1
                count = 0
                while l <= r:
                    if sensor[2] == 0:
                        command_queue.put('command_type:1')
                        break
                    elif l == r and sensor[l] == sensor[r] == 0:
                        command_queue.put('command_type:1')
                        if Debug_track: print('LCH: foward')
                        break
                    elif sensor[l] == 0:
                        if count > 0:
                            command_queue.put('command_type:12')
                            if Debug_track: print('LCH: foward & turn left')
                        else:
                            command_queue.put('command_type:14')
                            if Debug_track: print('LCH: foward & turn left hard')
                        break
                    elif sensor[r] == 0:
                        if count > 0:
                            command_queue.put('command_type:13')
                            if Debug_track: print('LCH: foward & turn right')
                        else:
                            command_queue.put('command_type:11')
                            if Debug_track: print('LCH: foward & turn right hard')
                        break

                    l += 1
                    r -= 1
                    count += 1

            time.sleep(0.05)

    def disableTrack(self):
        self.track = False

    def enableTrack(self):
        self.track = True


class AnalyzeCommand(ThreadTemplate):
    def __init__(self):
        super().__init__()
        self.led_count = 1
        self.loop_count = 1
        self.speed = 100

    def run(self):
        if Debug_flag: print('LCH: the AnalyzeCommand Initialized!')
        # stop_ret = True
        while self.working:
            cmd_message = None
            while not command_queue.empty():
                cmd_message = command_queue.get()
                # stop_ret = True
            # if not stop_ret: continue
            if cmd_message:
                # print("LCH: the cmd_message is", cmd_message)
                send_message, stop_ret = self.analyze_json(message=cmd_message)
            # else:
            #     send_message, stop_ret = self.analyze_json(message='command_type:0')
                if Debug_Send_flag: print("LCH: the send_message is: ", send_message)
                if ONLINE_MODE: self.sock.send(send_message.encode('utf-8'))
            time.sleep(0.005)

    def analyze_json(self, message):
        '''


        :param message:
        :return: send_message, ret
        '''
        speed = self.speed
        k_angle = 2.8
        k_linear = 1
        ret = True
        if message:
            pattern = re.compile(r'\d+')
            command_type = int(re.findall(pattern, message)[0])
            if command_type == 0:
                cmd_string = {"CTRL": {"FWD": 0, "RGT": 0, "TRN": 0}}
                ret = False

            if command_type == 1:  # forward
                cmd_string = {"CTRL": {"FWD": speed, "LFT": 0, "RGT": 0, "TRN": 0}}

            elif command_type == 2:  # backward
                cmd_string = {"CTRL": {"FWD": -speed, "RGT": 0, "TRN": 0}}

            elif command_type == 3:  # right
                cmd_string = {"CTRL": {"FWD": 0, "RGT": speed, "TRN": 0}}

            elif command_type == 4:  # left
                cmd_string = {"CTRL": {"FWD": 0, "RGT": -speed, "TRN": 0}}

            elif command_type == 5:  # forward right
                cmd_string = {"CTRL": {"FWD": speed//2, "RGT": speed//2, "TRN": 0}}

            elif command_type == 6:  # forward left
                cmd_string = {"CTRL": {"FWD": speed//2, "RGT": -speed//2, "TRN": 0}}

            elif command_type == 7:  # back right
                cmd_string = {"CTRL": {"FWD": -speed//2, "RGT": speed//2, "TRN": 0}}

            elif command_type == 8:  # back left
                cmd_string = {"CTRL": {"FWD": -speed//2, "RGT": -speed//2, "TRN": 0}}

            elif command_type == 9:  # turn right
                cmd_string = {"CTRL": {"FWD": 0, "RGT": 0, "TRN": speed}}

            elif command_type == 10:  # turn left
                cmd_string = {"CTRL": {"FWD": 0, "RGT": 0, "TRN": -speed}}

            elif command_type == 11:  # turn right & forward
                cmd_string = {"CTRL": {"FWD": int(speed * k_linear), "RGT": 0, "TRN": int(speed * k_angle // 1.5)}}

            elif command_type == 12:  # turn left & forward
                cmd_string = {"CTRL": {"FWD": int(speed * k_linear), "RGT": 0, "TRN": int(-speed * k_angle // 1.5)}}

            elif command_type == 13:  # turn right hard & forward
                cmd_string = {"CTRL": {"FWD": int(speed * k_linear), "RGT": 0, "TRN": int(speed * k_angle)}}

            elif command_type == 14:  # turn left hard & forward
                cmd_string = {"CTRL": {"FWD": int(speed * k_linear), "RGT": 0, "TRN": int(-speed * k_angle)}}

            elif command_type == 20: # enable track sensor
                # cmd_string = {"CRTL": {"TRC": 1}}
                cmd_string = {"SET": {"TRC": 1}}

            elif command_type == 21: # disable track sensor
                # cmd_string = {"CRTL": {"TRC": 0}}
                cmd_string = {"SET": {"TRC": 0}}

        send_message = to_json(cmd_string)

        return send_message, ret

    def setSpeed(self, speed):
        self.speed = speed


class UpdateData(ThreadTemplate):
    update_date = pyqtSignal(str,str,str,str,str,str,str,str)

    def __init__(self):
        super().__init__()
        # self.sock = sock

    def CutIntactMessage(self, message, depth=2):
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

    def run(self):
        global ROBOT_CONDITION

        if Debug_flag: print('LCH: UpdateData QThread start!')
        while self.working:
            if ONLINE_MODE:
                recv_data = self.sock.recv(1024)
            else:
                recv_data = OFFLINE_DATA
            if recv_data:
                if Debug_flag: print('LCH: Recv Data:', recv_data)
                recv_message = recv_data.decode('utf-8')

                if Debug_flag: print('LCH: Recv Message:', recv_message)
                recv_message_slice = self.CutIntactMessage(recv_message)
                if len(recv_message_slice) > 0:
                    stringData = json.loads(recv_message_slice)
                    stringData['RPT']['TRCSEN'] = int(stringData['RPT']['TRCSEN'], base=16)
                    stringData['RPT']['TRCSEN_Data'] = [int(i) for i in '{0:08b}'.format(stringData['RPT']['TRCSEN'])][3:]

                    # if Debug_track: print('LCH: recved track sensor data is ', stringData['RPT']['TRCSEN_Data'], ' the trcsen is ', stringData['RPT']['TRCSEN'])
                    if Debug_track: print('LCH: recved track sensor data is ', stringData['RPT']['TRCSEN'])
                    ROBOT_CONDITION = stringData['RPT'].copy()
                    if Debug_flag: print('LCH: StringData:', stringData)
                    cnt0 = stringData['RPT']['GER']
                    cnt1 = stringData['RPT']['PID']
                    cnt2 = stringData['RPT']['SONA1']
                    cnt3 = stringData['RPT']['SONA3']
                    cnt4 = stringData['RPT']['SONA5']
                    cnt5 = stringData['RPT']['SONA7']
                    cnt6 = stringData['RPT']['SONA8']
                    cnt7 = stringData['RPT']['TRCSEN_Data']

                    if Debug_flag: print('LCH: Updating...')
                    self.update_date.emit(str(cnt0), str(cnt1), str(cnt2), str(cnt3), str(cnt4), str(cnt5), str(cnt6), str(cnt7))
                    if Debug_flag: print('LCH: Update done')
                    time.sleep(0.005)
                else:
                    time.sleep(0.005)
                    continue
# --------------------------------------------------------Pad End Here------------------------------------------------- #


# --------------------------------------------------------ROS Start Here------------------------------------------------- #

class Window_ros(QWidget):
    def __init__(self, mainWindow):
        super().__init__()

        if not ONLINE_MODE:
            self.sock = 0
        else:
            self.sock = None

        self.mainWindow = mainWindow
        self.cmd_thread = AnalyzeCommand_ros()
        self.video_thread = VideoThread(ROS_CAMERA_IP_ADDR)
        # self.cmd_vel = {'CMD': {'angular_z': 0, 'linear_x': 0, 'linear_y': 0}}
        # self.state = {'ROS': {'cmt': 0, 'lidar': 0, 'realsense': 0, 'sc_hw': 0}}
        self.communication = {'COM':
            {"ROS": {"platform": 0, "lidar": 0, "realsense": 0, "cmt": 0, "video": 0},
             "CMD": {"linear_x": 0, "linear_y": 0, "angular_z": 0},
             "INI": {"br": (0, 0), "tl": (0, 0)},
             "SHUTDOWN": 0,
             "EXIT": 0}
        }
        self.cam_show_flag = True
        self.track_mode = False
        self.connected = False
        self.cmt_enabled = False
        self.initUI()

    def initUI(self):
        # self.setWindowTitle('OceanBotech ROV')
        # self.setWindowIcon(QIcon("oceanbotech.png"))

        main_layout = QtWidgets.QHBoxLayout()
        fun_button = QtWidgets.QVBoxLayout()
        socket_layout = QtWidgets.QHBoxLayout()

        # camera_layout = QtWidgets.QVBoxLayout()

        self.label_show_camera = QtWidgets.QLabel()
        self.label_show_camera.setBaseSize(641, 481)
        self.label_show_camera.setMinimumSize(641, 481)
        self.label_show_camera.setAutoFillBackground(False)

        self.button_connect_robot = QtWidgets.QPushButton(u'Connect Robot')
        self.button_connect_robot.setBaseSize(120, 30)
        socket_layout.addWidget(self.button_connect_robot)
        self.button_connect_robot.clicked.connect(self.connect_socket_on_click)

        self.text_ip_address = QtWidgets.QLineEdit()
        self.text_ip_address.setText('192.168.10.11')
        self.text_ip_address.setBaseSize(120, 30)
        socket_layout.addWidget(self.text_ip_address)

        fun_button.addLayout(socket_layout)

        self.button_open_realsense = QtWidgets.QPushButton(u'Open Realsense Node')
        self.button_open_realsense.setBaseSize(200, 50)
        fun_button.addWidget(self.button_open_realsense)
        self.button_open_realsense.clicked.connect(self.open_realsense_on_click)

        self.button_open_video = QtWidgets.QPushButton(u'Open WebVideo Node')
        self.button_open_video.setBaseSize(200, 50)
        fun_button.addWidget(self.button_open_video)
        self.button_open_video.clicked.connect(self.open_video_on_click)

        self.button_open_lidar = QtWidgets.QPushButton(u'Open Lidar Node')
        self.button_open_lidar.setBaseSize(200, 50)
        fun_button.addWidget(self.button_open_lidar)
        self.button_open_lidar.clicked.connect(self.open_lidar_on_click)

        self.button_open_platform = QtWidgets.QPushButton(u'Open Plateform Node')
        self.button_open_platform.setBaseSize(200, 50)
        fun_button.addWidget(self.button_open_platform)
        self.button_open_platform.clicked.connect(self.open_platform_on_click)

        self.button_open_cmt = QtWidgets.QPushButton(u'Open Visual Track Node')
        self.button_open_cmt.setBaseSize(200, 50)
        fun_button.addWidget(self.button_open_cmt)
        self.button_open_cmt.clicked.connect(self.open_cmt_on_click)

        self.button_open_camera = QtWidgets.QPushButton(u'Open Camera')
        self.button_open_camera.setBaseSize(200, 50)
        fun_button.addWidget(self.button_open_camera)
        self.button_open_camera.clicked.connect(self.open_camera_on_click)

        self.button_initalize_cmt = QtWidgets.QPushButton(u'Init Object')
        self.button_initalize_cmt.setBaseSize(200, 50)
        fun_button.addWidget(self.button_initalize_cmt)
        self.button_initalize_cmt.clicked.connect(self.open_init_on_click)

        self.button_shutdown = QtWidgets.QPushButton(u'Shutdown')
        self.button_shutdown.setBaseSize(200, 50)
        fun_button.addWidget(self.button_shutdown)
        self.button_shutdown.clicked.connect(self.shutdown_server_on_click)

        button_close = QtWidgets.QPushButton(u'Quit')
        button_close.setBaseSize(200, 50)
        fun_button.addWidget(button_close)
        button_close.clicked.connect(self.close_ros_on_click)

        main_layout.addLayout(fun_button)
        main_layout.addWidget(self.label_show_camera)
        self.setLayout(main_layout)
        self.video_thread.frame_data.connect(self.show_img)

        self.show()
        print('GUI initiated')

    def close_ros_on_click(self):
        if self.connected:
            self.connect_socket_on_click()
        time.sleep(0.5)
        self.close()
        self.mainWindow.restart()

    def open_realsense_on_click(self):
        if Debug_ros: print('LCH: open_realsense initiated.')
        if self.communication['COM']['ROS']['realsense']:
            self.communication['COM']['ROS']['realsense'] = 0
            self.button_open_realsense.setText(u'Open Realsense Node')
            command_queue.put(self.communication)
        else:
            self.communication['COM']['ROS']['realsense'] = 1
            self.button_open_realsense.setText(u'Close Realsense Node')
            command_queue.put(self.communication)

    def open_video_on_click(self):
        if Debug_ros: print('LCH: open_web_video initiated.')
        if self.communication['COM']['ROS']['video']:
            self.communication['COM']['ROS']['video'] = 0
            self.button_open_video.setText(u'Open WebVideo Node')
            command_queue.put(self.communication)
        else:
            self.communication['COM']['ROS']['video'] = 1
            self.button_open_video.setText(u'Close WebVideo Node')
            command_queue.put(self.communication)

    def open_lidar_on_click(self):
        if Debug_ros: print('LCH: open_lidar initiated.')
        if self.communication['COM']['ROS']['lidar']:
            self.communication['COM']['ROS']['lidar'] = 0
            self.button_open_lidar.setText(u'Open Lidar Node')
            command_queue.put(self.communication)
        else:
            self.communication['COM']['ROS']['lidar'] = 1
            self.button_open_lidar.setText(u'Close Lidar Node')
            command_queue.put(self.communication)


    def open_platform_on_click(self):
        if Debug_ros: print('LCH: open_platform initiated.')
        if self.communication['COM']['ROS']['platform']:
            self.communication['COM']['ROS']['platform'] = 0
            self.button_open_platform.setText(u'Open Platform Node')
            command_queue.put(self.communication)
        else:
            self.communication['COM']['ROS']['platform'] = 1
            self.button_open_platform.setText(u'Close Platform Node')
            command_queue.put(self.communication)


    def open_cmt_on_click(self):
        if Debug_ros: print('LCH: open_cmt initiated.')
        if self.communication['COM']['ROS']['cmt']:
            self.communication['COM']['ROS']['cmt'] = 0
            self.button_open_cmt.setText(u'Open Visual Track Node')
            self.cmt_enabled = False
            command_queue.put(self.communication)
        else:
            self.communication['COM']['ROS']['cmt'] = 1
            self.button_open_cmt.setText(u'Close Visual Track Node')
            self.cmt_enabled = True
            command_queue.put(self.communication)

    def open_camera_on_click(self):
        if Debug_video: print('LCH: open_camera initiated.')
        if self.connected:
            if self.cam_show_flag:
                self.video_thread.start()
                self.video_thread.cam_timer.start(30)
                self.button_open_camera.setText(u'Close Camera')
                self.cam_show_flag = False
            else:
                if Debug_video: print('LCH: open_camera_on_click: 5')
                self.video_thread.cam_timer.stop()
                self.cam_show_flag = True
                self.video_thread.quit()
                # self.video_thread.wait()
                self.label_show_camera.clear()
                self.button_open_camera.setText(u'Open Camera')
        else:
            pass

    def open_track_on_click(self):
        if Debug_track: print('LCH: open_track initiated.')
        if self.connected:
            if self.track_mode:
                self.track_mode = False
                self.track_thread.disableTrack()
                self.track_thread.quit()
                self.cmd_thread.setSpeed(100)
                self.button_open_track.setText(u'Open Track')
                # [command_queue.put('command_type:21') for _ in range(3)]

            else:
                # [command_queue.put('command_type:20') for _ in range(3)]
                self.track_mode = True
                self.track_thread.enableTrack()
                self.track_thread.start()
                self.cmd_thread.setSpeed(25)  # set the track mode basic speed to 30
                self.button_open_track.setText(u'Close Track')

    def open_init_on_click(self):
        if Debug_cmt: print('LCH: open_init initiated')
        if self.connected and self.cmt_enabled:
            tl, br = self.video_thread.init_cmt_tracking()
            self.communication['COM']['INI']['br'] = br
            self.communication['COM']['INI']['tl'] = tl
            command_queue.put(self.communication)
        else:
            pass

    def shutdown_server_on_click(self):
        if Debug_cmt: print('LCH: Server shutdown called')
        if self.connected:
            self.communication['COM']['SHUTDOWN'] = 1
            command_queue.put(self.communication)
        else:
            pass
        time.sleep(0.5)
        self.close_ros_on_click()

    def show_img(self, frame):
        if Debug_video: print('LCH: Main thread show img.')
        showImage = QtGui.QImage(frame.data, frame.shape[1], frame.shape[0], QtGui.QImage.Format_RGB888)
        self.label_show_camera.setPixmap(QtGui.QPixmap.fromImage(showImage))

    def connect_socket_on_click(self):
        global ONLINE_MODE
        if self.sock:
            self.sock = None

        if not self.connected:
            ip_address = self.text_ip_address.text()
            if ip_address == '127.0.0.1':
                self.sock = 0
                ONLINE_MODE = False
            else:
                try:
                    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 创建一个socket
                    self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    self.sock.connect((ip_address, 8899))  # 建立连接
                except:
                    return
                ONLINE_MODE = True

            print('LCH: the sock is set')

            self.cmd_thread.setSock(self.sock)
            self.video_thread.setSock(self.sock)

            self.cmd_thread.start()
            self.button_connect_robot.setText(u'Disconnect Robot')

            self.connected = True

        else:

            for key in self.communication['COM']['ROS'].keys():
                self.communication['COM']['ROS'][key] = 0
            self.communication['COM']['EXIT'] = 1
            [command_queue.put(self.communication) for _ in range(2)]
            time.sleep(0.5)
            self.communication['COM']['EXIT'] = 0

            self.cmd_thread.disable()
            self.cmd_thread.quit()
            self.cmd_thread.wait()

            self.video_thread.disable()
            self.video_thread.quit()
            self.video_thread.wait()

            del self.cmd_thread
            del self.video_thread

            print('LCH: delete done')

            self.cmd_thread = AnalyzeCommand_ros()
            self.video_thread = VideoThread(ROS_CAMERA_IP_ADDR)

            self.video_thread.frame_data.connect(self.show_img)

            self.button_connect_robot.setText(u'Connect Robot')
            self.connected = False


class AnalyzeCommand_ros(ThreadTemplate):
    def __init__(self):
        super().__init__()
        self.led_count = 1
        self.loop_count = 1
        self.speed = 100

    def run(self):
        if Debug_flag: print('LCH: the AnalyzeCommand Initialized!')
        # stop_ret = True
        while self.working:
            cmd_dict = None
            while not command_queue.empty():
                cmd_dict = command_queue.get()
            if cmd_dict:
                # print("LCH: the cmd_message is", cmd_dict)
                send_message = json.dumps(cmd_dict)
                if Debug_Send_flag: print("LCH: the send_message is: ", send_message)
                if ONLINE_MODE: self.sock.send(send_message.encode('utf-8'))
            time.sleep(0.005)

    def analyze_json(self, message):
        send_message = to_json(message)
        return send_message

    def setSpeed(self, speed):
        self.speed = speed

# --------------------------------------------------------ROS End Here------------------------------------------------- #


# --------------------------------------------------------COMMON Start Here------------------------------------------------- #

class VideoThread(ThreadTemplate):
    frame_data = pyqtSignal(np.ndarray)

    def __init__(self, camera_address):
        super().__init__()
        self.frame = np.zeros([480, 640])
        self.cam_timer = QtCore.QTimer()
        self.cam_timer.timeout.connect(self.push_img)
        self.camera_address = camera_address

    def run(self):
        if self.sock:
            self.cam_num = self.camera_address
        else:
            self.cam_num = 0
        self.cap = cv2.VideoCapture(self.cam_num)
        if Debug_video: print('LCH: Video thread begin...')
        while self.working:
            ret, img = self.cap.read()
            if ret:
                img = cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), (640, 480))
                if Debug_video: print('LCH: img processed...')
                # self.frame = cv2.resize(cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB), (640, 480))
                self.frame = img.copy()
            time.sleep(0.01)

    def push_img(self):
        if Debug_video: print('LCH: push_img begin...')
        self.frame_data.emit(self.frame)
        if Debug_video: print('LCH: push_img done.')

    def init_cmt_tracking(self):
        (tl, br) = self.get_rect()
        return tl, br

    def get_rect(self, title='get_rect'):
        im = cv2.cvtColor(self.frame, cv2.COLOR_RGB2BGR)
        mouse_params = {'tl': None, 'br': None, 'current_pos': None,
                'released_once': False}

        cv2.namedWindow(title)
        cv2.moveWindow(title, 100, 100)

        def onMouse(event, x, y, flags, param):

                param['current_pos'] = (x, y)

                if param['tl'] is not None and not (flags & cv2.EVENT_FLAG_LBUTTON):
                        param['released_once'] = True

                if flags & cv2.EVENT_FLAG_LBUTTON:
                        if param['tl'] is None:
                                param['tl'] = param['current_pos']
                        elif param['released_once']:
                                param['br'] = param['current_pos']

        cv2.setMouseCallback(title, onMouse, mouse_params)
        cv2.imshow(title, im)

        while mouse_params['br'] is None:
                im_draw = np.copy(im)

                if mouse_params['tl'] is not None:
                        cv2.rectangle(im_draw, mouse_params['tl'],
                                mouse_params['current_pos'], (255, 0, 0))

                cv2.imshow(title, im_draw)
                _ = cv2.waitKey(10)

        cv2.destroyWindow(title)

        tl = (min(mouse_params['tl'][0], mouse_params['br'][0]),
                min(mouse_params['tl'][1], mouse_params['br'][1]))
        br = (max(mouse_params['tl'][0], mouse_params['br'][0]),
                max(mouse_params['tl'][1], mouse_params['br'][1]))

        return (tl, br)

# --------------------------------------------------------COMMON End Here------------------------------------------------- #

# -------------------------------------------------------- DL Start Here ------------------------------------------------
class Window_dl(QWidget):
    def __init__(self, mainWindow):
        super().__init__()

        self.mainWindow = mainWindow

        # self.img_cols = 320
        # self.img_rows = 240
        self.img_cols = 640
        self.img_rows = 480

        self.image = None
        self.detector = None
        self.model = 'handy'
        self.weights_initial = {"printed": "./weights/printed_digit_best_wiehgts.pth",
                                "handy": "./weights/handy_digit_best_weights.pth",
                                "cvd": "./weights/cvd_best_weights.pth"}
        self.demo_pic = {"printed": "./data/printed_demo.png",
                         "handy": "./data/handy_demo.png",
                         "cvd": "./data/cvd_demo.png"}
        self.initUI()

    def initUI(self):
        # self.setWindowTitle('OceanBotech SC')
        # self.setWindowIcon(QIcon("oceanbotech.png"))

        main_layout = QtWidgets.QHBoxLayout()
        fun_button = QtWidgets.QVBoxLayout()
        model_layout = QtWidgets.QHBoxLayout()
        image_layout = QtWidgets.QHBoxLayout()
        weights_layout = QtWidgets.QHBoxLayout()
        choose_layout = QtWidgets.QHBoxLayout()
        result_layout = QtWidgets.QHBoxLayout()

        self.rb_cvd = QRadioButton('cat vs dog', self)
        self.rb_printed = QRadioButton('printed digit', self)
        self.rb_handy = QRadioButton('handy digit', self)
        self.rb_select = QtWidgets.QPushButton('Select', self)

        self.rb_group = QtWidgets.QButtonGroup(self)
        self.rb_group.addButton(self.rb_cvd, 11)
        self.rb_group.addButton(self.rb_printed, 12)
        self.rb_group.addButton(self.rb_handy, 13)
        self.rb_group.buttonClicked.connect(self.rbclicked)
        self.rb_select.clicked.connect(self.select_model_on_click)
        model_layout.addWidget(self.rb_cvd)
        model_layout.addWidget(self.rb_printed)
        model_layout.addWidget(self.rb_handy)
        model_layout.addWidget(self.rb_select)

        self.button_load_weights = QtWidgets.QPushButton(u'Load Model')
        self.button_load_weights.setFixedSize(120, 30)
        self.button_load_weights.setMinimumSize(120, 30)
        weights_layout.addWidget(self.button_load_weights)
        self.button_load_weights.clicked.connect(self.load_weights_on_click)

        self.text_weights_address = QtWidgets.QLineEdit()
        # self.text_weights_address.setText('printed_digit_best_weights.pth')
        self.text_weights_address.setFixedSize(200, 30)
        self.text_weights_address.setMinimumSize(200, 30)
        weights_layout.addWidget(self.text_weights_address)

        self.label_show_image = QtWidgets.QLabel()
        self.label_show_image.setFixedSize(self.img_cols+1, self.img_rows+1)
        self.label_show_image.setAutoFillBackground(True)

        self.button_read_image = QtWidgets.QPushButton(u'Read Image')
        self.button_read_image.setFixedSize(120, 30)
        self.button_read_image.setMinimumSize(120, 30)
        image_layout.addWidget(self.button_read_image)
        self.button_read_image.clicked.connect(self.read_image_on_click)

        self.text_image_address = QtWidgets.QLineEdit()
        # self.text_image_address.setText('./demo.png')
        self.text_image_address.setFixedSize(200, 30)
        self.text_image_address.setMinimumSize(200, 30)
        image_layout.addWidget(self.text_image_address)

        button_choose_weights = QtWidgets.QPushButton(u'Choose Weights')
        button_choose_weights.setFixedSize(160, 40)
        button_choose_weights.setMinimumSize(160, 40)
        choose_layout.addWidget(button_choose_weights)
        button_choose_weights.clicked.connect(self.choose_weights_on_click)

        button_choose_image = QtWidgets.QPushButton(u'Choose Image')
        button_choose_image.setFixedSize(160, 40)
        button_choose_image.setMinimumSize(160, 40)
        choose_layout.addWidget(button_choose_image)
        button_choose_image.clicked.connect(self.choose_image_on_click)

        button_recognize = QtWidgets.QPushButton(u'Recognize')
        button_recognize.setFixedSize(120, 60)
        button_recognize.setMinimumSize(120, 60)
        result_layout.addWidget(button_recognize)
        button_recognize.clicked.connect(self.recognize_image_on_click)

        self.label_show_result = QtWidgets.QLabel()
        self.label_show_result.setFixedSize(200, 60)
        self.label_show_result.setMinimumSize(200, 60)
        result_layout.addWidget(self.label_show_result)

        fun_button.addLayout(model_layout)
        fun_button.addLayout(choose_layout)
        fun_button.addLayout(weights_layout)
        fun_button.addLayout(image_layout)
        fun_button.addLayout(result_layout)

        button_close = QtWidgets.QPushButton(u'Quit')
        button_close.setFixedSize(120, 60)
        button_close.setMinimumSize(120, 60)
        fun_button.addWidget(button_close)
        button_close.clicked.connect(self.quit_to_mainWindow_on_click)

        main_layout.addLayout(fun_button)
        main_layout.addWidget(self.label_show_image)
        self.setLayout(main_layout)
        self.show()
        print('GUI initiated')

    def quit_to_mainWindow_on_click(self):
        self.close()
        self.mainWindow.restart()

    def rbclicked(self):
        sender = self.sender()
        if sender == self.rb_group:
            if self.rb_group.checkedId() == 11:
                self.model = 'cvd'
            elif self.rb_group.checkedId() == 12:
                self.model = 'printed'
            elif self.rb_group.checkedId() == 13:
                self.model = 'handy'

    def select_model_on_click(self):
        print('The model is ', self.model)
        self.text_image_address.setText(self.demo_pic[self.model])
        self.text_weights_address.setText(self.weights_initial[self.model])

    def read_image_on_click(self):
        if Debug_flag: print('LCH: read_image_on_click initiated.')
        image_path = self.text_image_address.text()
        img = cv2.imread(image_path)
        img = cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), (self.img_cols, self.img_rows))
        self.image = img.copy()
        self.show_img(self.image)

    def show_img(self, frame):
        if Debug_flag: print('LCH: Main thread show img.')
        showImage = QtGui.QImage(frame.data, frame.shape[1], frame.shape[0], QtGui.QImage.Format_RGB888)
        self.label_show_image.setPixmap(QtGui.QPixmap.fromImage(showImage))

    def load_weights_on_click(self):
        if self.detector is None:
            weights_path = self.text_weights_address.text()
            if self.model == 'printed':
                self.detector = printed_digit_detector(weights_path)
            elif self.model == 'handy':
                self.detector = handy_digit_detector(weights_path)
            elif self.model == 'cvd':
                self.detector = cvd_detector(weights_path)
            self.button_load_weights.setText(u'Delete Model')
        else:
            del self.detector
            self.detector = None
            self.button_load_weights.setText(u'Load Model')

    def recognize_image_on_click(self):
        if self.detector is not None and self.image is not None:
            result = self.detector.detect([self.image])[0]
            self.label_show_result.setText('The recognition result is {}'.format(result))
        else:
            self.label_show_result.setText('Error! \nPlease check the image and model is loaded correctly.')

    def choose_image_on_click(self):
        openfile_name = QFileDialog.getOpenFileName(self, '选择图片', '', '')[0]
        self.text_image_address.setText(openfile_name)

    def choose_weights_on_click(self):
        openfile_name = QFileDialog.getOpenFileName(self, '选择权重', '', '')[0]
        self.text_weights_address.setText(openfile_name)