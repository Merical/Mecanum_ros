#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math
import re
import yaml


class CMTFollower(object):
    def __init__(self):
        rospy.init_node("cmt_tracker")
        rospy.on_shutdown(self.shutdown)

        self.rate = rospy.get_param("~rate", 20)
        r = rospy.Rate(self.rate)

        self.chatter_topic = rospy.get_param("~chatter_topic", '/ob_vision/follower/chatter')
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 2.0)
        self.min_angular_speed = rospy.get_param("~min_angular_speed", 0.5)
        self.max_x = rospy.get_param("~max_x", 20.0)
        self.goal_x = rospy.get_param("~goal_x", 0.6)
        self.x_threshold = rospy.get_param("~x_threshold", 0.01)
        self.y_threshold = rospy.get_param("~y_threshold", 0.2)
        self.x_scale = rospy.get_param("~x_scale", 0.5)
        self.y_scale = rospy.get_param("~y_scale", 1.0)
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.3)
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.1)
        self.by_dist = rospy.get_param("~by_dist", True)
        self.shift_pid_val = rospy.get_param("~shift_pid_value", [1, 0.5, 0.05, 0.05])
        self.rotate_pid_val = rospy.get_param("~rotate_pid_value", [1, 0.5, 0.05, 0.05])
        self.dist_default = rospy.get_param("~dist_default", 0.8)
        self.motion_mode = rospy.get_param("~motion_mode", 0)  # 1 for mecanum shift, 0 for rotation
        self.config_file_path = rospy.get_param("~config_file_path", '/home/lishenghao/ros_workspace/SC0_ws/src/ocean_vision/config/pyconfig.yaml')
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        f = open(self.config_file_path, 'r')
        config = yaml.load(f.read())
        self.FRAME_WIDTH = config['Video']['FRAME_WIDTH']
        self.FRAME_HEIGHT = config['Video']['FRAME_HEIGHT']
        # self.pid_val = config['SmartCar']['k_val']
        self.scale_default = config['SmartCar']['scale_default']
        # self.dist_default = config['SmartCar']['dist_default']
        # self.by_dist = config['SmartCar']['by_dist']
        self.s_delta_last = 0
        self.c_delta_last = 0
        self.d_delta_last = 0
        self.lost_count = 0

        self.move_cmd = Twist()
        self.run()

    def callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
        cmd_message = re.findall(re.compile(r'\$\$.*?\&\&'), data.data)
        if len(cmd_message) > 0:
            cmd_para = re.findall(re.compile(r'[\d.]+'), cmd_message[-1])
            print('LCH: the cmd para is ', cmd_para)
            if len(cmd_para)==4:
                centerx = float(cmd_para[0])
                scale = float(cmd_para[3])
                dist = float(cmd_para[2])
                center_delta = (self.FRAME_WIDTH/2 - centerx)/float(self.FRAME_WIDTH)
                if self.by_dist:
                    dist_delta = -(self.dist_default - dist)
                    self.get_speed_dist(center_delta, dist_delta)
                else:
                    scale_delta = self.scale_default - scale
                    self.get_speed_scale(center_delta, scale_delta)
                self.lost_count = 0
            else:
                self.lost_count += 1
                center_delta = 0
                scale_delta = 0
                dist_delta = 0
                if self.by_dist:
                    self.get_speed_dist(center_delta, dist_delta, search_flag=True, count=self.lost_count)

                else:
                    self.get_speed_scale(center_delta, scale_delta, search_flag=True, count=self.lost_count)
                # print('LCH: the move cmd is ', self.move_cmd)
            self.cmd_vel_pub.publish(self.move_cmd)

    def run(self):
        # rospy.init_node('listener', anonymous=True)
        rospy.Subscriber(self.chatter_topic, String, self.callback)

    def get_speed_scale(self, c_delta, s_delta, search_flag=False, count=0):
        if not search_flag:
            kp_d, kd_d, kp_y, kd_y = self.shift_pid_val if self.motion_mode else self.rotate_pid_val
            if abs(c_delta) > self.y_threshold and abs(self.c_delta_last - c_delta) < 0.3:
                y_speed = kp_y * c_delta + kd_y * (c_delta - self.c_delta_last)
                y_speed = self.map_speed(y_speed)
                if self.motion_mode:
                    self.move_cmd.linear.y = math.copysign(min(self.max_linear_speed, max(self.max_linear_speed, abs(y_speed))), y_speed)
                else:
                    self.move_cmd.angular.z = math.copysign(max(self.min_angular_speed, min(self.max_angular_speed, abs(y_speed))), y_speed)

            else:
                self.move_cmd.linear.y = 0.0
                self.move_cmd.angular.z = 0.0

            if abs(s_delta) > self.x_threshold and abs(self.s_delta_last - s_delta) < 0.3:
                x_speed = kp_x * s_delta + kd_x * (s_delta - self.s_delta_last)
                x_speed = self.map_speed(x_speed)
                self.move_cmd.linear.x = math.copysign(min(self.max_linear_speed, max(self.min_linear_speed, abs(x_speed))), x_speed)
            else:
                self.move_cmd.linear.x = 0.0

            self.s_delta_last = s_delta
            self.c_delta_last = c_delta
        else:
            self.move_cmd.angular.z = self.max_angular_speed * 0.2 if count > 10 else 0
            self.move_cmd.linear.x = 0.0
            self.move_cmd.linear.y = 0.0

    def get_speed_dist(self, c_delta, d_delta, search_flag=False, count=0):
        if not search_flag:
            kp_d, kd_d, kp_y, kd_y = self.shift_pid_val if self.motion_mode else self.rotate_pid_val
            if abs(c_delta) > self.y_threshold and abs(self.c_delta_last - c_delta) < 0.3:
                y_speed = kp_y * c_delta + kd_y * (c_delta - self.c_delta_last)
                y_speed = self.map_speed(y_speed)
                if self.motion_mode:
                    self.move_cmd.linear.y = math.copysign(min(self.max_linear_speed, max(self.max_linear_speed, abs(y_speed))), y_speed)
                else:
                    self.move_cmd.angular.z = math.copysign(max(self.min_angular_speed, min(self.max_angular_speed, abs(y_speed))), y_speed)
            else:
                self.move_cmd.linear.y = 0.0
                self.move_cmd.angular.z = 0.0

            # if abs(d_delta) > self.x_threshold and abs(self.d_delta_last - d_delta) < 0.03:
            if abs(d_delta) > self.x_threshold:
                x_speed = kp_d * d_delta + kd_d * (d_delta - self.d_delta_last)
                x_speed = self.map_speed(x_speed)
                self.move_cmd.linear.x = math.copysign(min(self.max_linear_speed, max(self.min_linear_speed, abs(x_speed))), x_speed)
            else:
                self.move_cmd.linear.x = 0.0

            self.d_delta_last = d_delta
            self.c_delta_last = c_delta
        else:
            self.move_cmd.angular.z = math.copysign(self.c_delta_last, self.max_angular_speed) if count > 10 else 0
            self.move_cmd.linear.x = 0.0
            self.move_cmd.linear.y = 0.0

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

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':

    try:
        CMTFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("CMT follower node terminated.")
