#!/usr/bin/ python
# -*- coding:utf-8 -*-
from ctypes import *
import numpy as np
import cv2
import os


class Analyze_process(multiprocessing.Process):
    def __init__(self, input_queue, output_queue, k_val):
        multiprocessing.Process.__init__(self)
        self.vis_queue = input_queue
        self.cmd_queue = output_queue
        self.pid_val = k_val

    def run(self):
        global Work_flag
        while Work_flag:
            analyze_flag = False
            while not self.vis_queue.empty():
                vis_message = self.vis_queue.get()
                if DEBUG_FLAG: print('LCH: The vis_message is', vis_message)
                analyze_flag = True
            if analyze_flag:
                cmd_message = self.analyze(vis_message)
                if DEBUG_FLAG: print('LCH: The cmd_message from Analyze_process is', cmd_message)
                self.cmd_queue.put(cmd_message)
        sys.exit()

    def analyze(self, message):
        pattern = re.compile(r':([-\d\.]+)')
        w_delta, l_delta = re.findall(pattern, message)
        w_delta = float(w_delta); l_delta = float(l_delta)
        if DEBUG_FLAG: print('LCH: The w_delta from Analyze_process is {0:.4f}, the l_delta from Analyze_process is '
                             '{1:.4f}'.format(w_delta, l_delta))
        output_message = self.get_speed(w_delta, l_delta)
        return output_message

    def get_speed(self, c_delta, s_delta):
        global max_l_speed
        global max_w_speed
        global c_delta_last
        global s_delta_last
        global w_speed_last
        global l_speed_last
        global scale_threshold
        global center_threshold

        kp_l, kd_l, kp_w, kd_w = self.pid_val
        if (c_delta * c_delta_last) < 0:
            w_speed = 0; l_speed = 0
            if DEBUG_FLAG: print("LCH: Protection mode from analyze_process, waiting next queue...")

        else:
            if DEBUG_FLAG: print("LCH: Action mode from analyze_process, analyzing...")
            if not abs(c_delta) < center_threshold:
                w_speed = kp_w * c_delta + kd_w * (c_delta - c_delta_last)
            else:
                w_speed = 0

            if not abs(s_delta) < scale_threshold:
                l_speed = kp_l * s_delta + kd_l * (s_delta - s_delta_last)
            else:
                l_speed = 0


            w_speed = self.map_speed(w_speed)
            l_speed = self.map_speed(l_speed)

            w_speed = max_w_speed if w_speed > max_w_speed else w_speed
            l_speed = max_l_speed if l_speed > max_l_speed else l_speed
            w_speed = -max_w_speed if w_speed < -max_w_speed else w_speed
            l_speed = -max_l_speed if l_speed < -max_l_speed else l_speed

        output_message = 'w_speed:{0:.4f};l_speed:{1:.4f};'.format(w_speed, l_speed)

        s_delta_last = s_delta
        c_delta_last = c_delta
        w_speed_last = w_speed
        l_speed_last = l_speed
        return output_message

    def map_speed(self, speed):
        if speed > 0:
            if speed > 0.1:
                speed = math.floor(speed*10)/10
            else: speed = 0.05
        else:
            if speed < 0.1:
                speed = math.ceil(speed*10)/10
            else:
                speed = -0.05
        return speed


if __name__ == '__main__':
    #################################
    ## Default Machine Parameter
    ## Check with Evironment
    max_l_speed = config['Handsfree']['max_l_speed']
    max_w_speed = config['Handsfree']['max_w_speed']
    w_speed_last = 0
    l_speed_last = 0
    phi_value = config['Handsfree']['phi_value']
    Work_flag = config['Handsfree']['Work_flag']
    pitch_val = 0
    roll_val = 0
    yaw_val = 0

    default_scale = config['Handsfree']['default_scale']
    c_delta_last = 0
    s_delta_last = 0
    center_threshold = config['Handsfree']['center_threshold']
    scale_threshold = config['Handsfree']['scale_threshold']
    k_val = config['Handsfree']['k_val']
    ###################################

    os.system('sudo chmod 777 /dev/ttyUSB0')

    so = cdll.LoadLibrary
    lib = so("./cpp_extension/libpycallclass.so")
    lib_hc = so("./cpp_extension/hc_sensor.so")
    handsfreeDriver = lib.handsfreeDriver
    hc_sensor = lib_hc.hc_sensor
    handsfreeDriver.restype = c_char_p
    hc_sensor.restype = c_char_p

    vis2ana_queue = multiprocessing.Queue()
    ana2cmd_queue = multiprocessing.Queue()

    Vision_task = Vision_process(vis2ana_queue)
    Analyze_task = Analyze_process(vis2ana_queue, ana2cmd_queue, k_val)
    Handsfree_task = Handsfree_process(ana2cmd_queue)
    Vision_task.start()
    Analyze_task.start()
    Handsfree_task.start()

    # print("\n EXIT MAIN PROCESS!")
