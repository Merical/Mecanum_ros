#!/usr/bin python3
# -*- coding:utf-8 -*-

from ctypes import *
import time
import os
import numpy as np

so = cdll.LoadLibrary
lib_hc = so("./cpp_extension/hc_sensor.so")
hc_sensor = lib_hc.hc_sensor
hc_sensor.restype = c_char_p

while True:
    dist = float(hc_sensor())
    print ('HC Sensor: Distance: %0.2f cm' %dist)
    time.sleep(0.1)
