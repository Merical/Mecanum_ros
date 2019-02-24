#!/usr/bin/python
# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math
import re
import yaml


class CMTFollower(object):
    def __init__(self):
        f = open('/home/robot/ros_workspace/my_handsfree_ws/src/ocean_vision/config/pyconfig.yaml', 'r')
        config = yaml.load(f.read())
        self.FRAME_WIDTH = config['Video']['FRAME_WIDTH']
        self.FRAME_HEIGHT = config['Video']['FRAME_HEIGHT']
        self.pid_val = config['Handsfree']['k_val']
        self.scale_default = config['Handsfree']['scale_default']
        self.s_delta_last = 0
        self.c_delta_last = 0
        self.lost_count = 0

        rospy.init_node("cmt_tracker")
        rospy.on_shutdown(self.shutdown)

        self.rate = rospy.get_param("~rate", 10)
        r = rospy.Rate(self.rate)

        self.max_angular_speed = rospy.get_param("~max_angular_speed", 2.0)
        self.min_angular_speed = rospy.get_param("~min_angular_speed", 0.5)
        self.max_x = rospy.get_param("~max_x", 20.0)
        self.goal_x = rospy.get_param("~goal_x", 0.6)
        self.x_threshold = rospy.get_param("~x_threshold", 0.1)
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
        cmd_para = re.findall(re.compile(r'[\d.]+'), data.data)
        #print('LCH: the cmd para is ', cmd_para)
        if len(cmd_para)==3:
            centerx = float(cmd_para[0])
            scale = float(cmd_para[2])
            center_delta = (self.FRAME_WIDTH/2 - centerx)/float(self.FRAME_WIDTH)
            scale_delta = self.scale_default - scale
            self.get_speed(center_delta, scale_delta)
            self.lost_count = 0
        else:
            self.lost_count += 1
            center_delta = 0
            scale_delta = 0
            self.get_speed(center_delta, scale_delta, search_flag=True, count=self.lost_count)
        print('LCH: the move cmd is ', self.move_cmd)
        self.cmd_vel_pub.publish(self.move_cmd)

    def run(self):
        # rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('chatter', String, self.callback)

    def get_speed(self, c_delta, s_delta, search_flag=False, count=0):
        if not search_flag:
            kp_l, kd_l, kp_w, kd_w = self.pid_val
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
        print 'get it'
    except rospy.ROSInterruptException:
        rospy.loginfo("CMT follower node terminated.")
