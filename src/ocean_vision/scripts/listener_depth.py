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
        f = open('/home/robot/ros_workspace/SC0_ws/src/ocean_vision/config/pyconfig.yaml', 'r')
        config = yaml.load(f.read())
        self.FRAME_WIDTH = config['Video']['FRAME_WIDTH']
        self.FRAME_HEIGHT = config['Video']['FRAME_HEIGHT']
        self.pid_val = config['SmartCar']['k_val']
        self.scale_default = config['SmartCar']['scale_default']
        self.dist_default = config['SmartCar']['dist_default']
        self.by_dist = config['SmartCar']['by_dist']
        self.s_delta_last = 0
        self.c_delta_last = 0
        self.d_delta_last = 0
        self.lost_count = 0

        rospy.init_node("cmt_tracker")
        rospy.on_shutdown(self.shutdown)

        self.rate = rospy.get_param("~rate", 20)
        r = rospy.Rate(self.rate)

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
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

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
        rospy.Subscriber('chatter', String, self.callback)

    def get_speed_scale(self, c_delta, s_delta, search_flag=False, count=0):
        if not search_flag:
            kp_l, kd_l, kp_w, kd_w, _, _ = self.pid_val
            if not abs(c_delta) < self.y_threshold:
                w_speed = kp_w * c_delta + kd_w * (c_delta - self.c_delta_last)
                w_speed = self.map_speed(w_speed)
                self.move_cmd.angular.z = math.copysign(max(self.min_angular_speed, min(self.max_angular_speed, abs(w_speed))), w_speed)
            else:
                self.move_cmd.angular.z = 0.0

            if abs(s_delta) > self.x_threshold and abs(self.s_delta_last - s_delta) < 0.3:
                l_speed = kp_l * s_delta + kd_l * (s_delta - self.s_delta_last)
                l_speed = self.map_speed(l_speed)
                self.move_cmd.linear.x = math.copysign(min(self.max_linear_speed, max(self.min_linear_speed, abs(l_speed))), l_speed)
            else:
                self.move_cmd.linear.x = 0.0

            self.s_delta_last = s_delta
            self.c_delta_last = c_delta
        else:
            self.move_cmd.angular.z = self.max_angular_speed * 0.2 if count > 10 else 0
            self.move_cmd.linear.x = 0.0

    def get_speed_dist(self, c_delta, d_delta, search_flag=False, count=0):
        if not search_flag:
            _, _, kp_w, kd_w, kp_d, kd_d = self.pid_val
            if not abs(c_delta) < self.y_threshold:
                w_speed = kp_w * c_delta + kd_w * (c_delta - self.c_delta_last)
                w_speed = self.map_speed(w_speed)
                self.move_cmd.angular.z = math.copysign(max(self.min_angular_speed, min(self.max_angular_speed, abs(w_speed))), w_speed)
            else:
                self.move_cmd.angular.z = 0.0

            if abs(d_delta) > self.x_threshold and abs(self.d_delta_last - d_delta) < 0.03:
                l_speed = kp_d * d_delta + kd_d * (d_delta - self.d_delta_last)
                l_speed = self.map_speed(l_speed)
                self.move_cmd.linear.x = math.copysign(min(self.max_linear_speed, max(self.min_linear_speed, abs(l_speed))), l_speed)
            else:
                self.move_cmd.linear.x = 0.0

            self.d_delta_last = d_delta
            self.c_delta_last = c_delta
        else:
            self.move_cmd.angular.z = self.c_delta_last / abs(self.c_delta_last) * self.max_angular_speed * 0.2 if count > 10 else 0
            self.move_cmd.linear.x = 0.0

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
