#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math
import re
from sc_msgs.msg import vision_line
import yaml


class LineAction(object):
    def __init__(self):
        rospy.init_node("line_action_node")
        rospy.on_shutdown(self.shutdown)

        self.rate = rospy.get_param("~rate", 20)
        r = rospy.Rate(self.rate)

        self.chatter_topic = rospy.get_param("~chatter_topic", '/ob_vision/line_tracker/chatter')
        self.forward_speed = rospy.get_param("~forward_speed", 0.5)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 2.0)
        self.min_angular_speed = rospy.get_param("~min_angular_speed", 0.1)
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.8)
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.1)
        self.translate_dead_zone = rospy.get_param("~translate_dead_zone", 10) # unit: pixel
        self.rotate_dead_zone = rospy.get_param("~rotate_dead_zone", 10) # unit: pixel
        self.translate_pid_val = rospy.get_param("~translate_pid_value", [0.1, 0, 0.5])
        self.rotate_pid_val = rospy.get_param("~rotate_pid_value", [1, 0, 0.5])
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        self.translate_x_last = 0
        self.rotate_x_last = 0

        self.move_cmd = Twist()
        self.run()

    def callback(self, line_pos):
        # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
        # print(line_pos.translate_x)
        # print(line_pos)
        self.move_cmd = Twist()
        if line_pos.translate_y and line_pos.rotate_y:
            self.move_cmd.linear.x = self.forward_speed
            self.get_speed(line_pos)
        self.cmd_vel_pub.publish(self.move_cmd)

    def run(self):
        rospy.Subscriber(self.chatter_topic, vision_line, self.callback)

    def get_speed(self, pos):
        kp_t, ki_t, kd_t = self.translate_pid_val
        kp_r, ki_r, kd_r = self.rotate_pid_val

        if abs(pos.translate_x) <= self.translate_dead_zone:
            self.move_cmd.linear.y = 0
        else:
            translate_x = float(pos.translate_x) / 100.
            print('LCH: the translate_x is ', translate_x)
            speed_t = kp_t * translate_x + kd_t * (self.translate_x_last - translate_x)
            # print('LCH: the speed_t is ', speed_t)
            # speed_t = self.map_speed(speed_t)
            self.move_cmd.linear.y = math.copysign(min(self.max_linear_speed, max(self.min_linear_speed, abs(speed_t))), speed_t)
            self.translate_x_last = translate_x

        if abs(pos.rotate_x) <= self.rotate_dead_zone:
            self.move_cmd.angular.z = 0
        else:
            rotate_x = float(pos.rotate_x) / 100.
            print('LCH: the rotate_X is ', rotate_x)
            speed_r = kp_r * rotate_x + kd_r * (self.rotate_x_last - rotate_x)
            # print('LCH: the speed_r is ', speed_r)
            # speed_r = self.map_speed(speed_r)
            self.move_cmd.angular.z = math.copysign(min(self.max_angular_speed, max(self.min_angular_speed, abs(speed_r))), speed_r)
            self.rotate_x_last = rotate_x

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
        LineAction()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("CMT follower node terminated.")
