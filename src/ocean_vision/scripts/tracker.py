#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
import math
import time
import re
import yaml


class CMTFollower(object):
    def __init__(self):
        f = open('/home/robot/ros_workspace/SC0_ws/src/ocean_vision/config/tracker.yaml', 'r')
        config = yaml.load(f.read())
        self.FRAME_WIDTH = config['Video']['FRAME_WIDTH']
        self.FRAME_HEIGHT = config['Video']['FRAME_HEIGHT']
        self.dist_default = config['SmartCar']['dist_default']

        self.center_delta = 0
        self.dist_delta = 0
        self.if_track = False
        self.do_job = True

        rospy.init_node("cmt_tracker")

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("wait for the action server to come up")
        self.move_base.wait_for_server(rospy.Duration(5))
        rospy.loginfo("waiting done")

        self.rate = rospy.get_param("~rate", 30)
        self.r = rospy.Rate(self.rate)

        rospy.on_shutdown(self.shutdown)
        self.run()

    def callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
        cmd_message = re.findall(re.compile(r'\$\$.*?\&\&'), data.data)
        if len(cmd_message) > 0:
            cmd_para = re.findall(re.compile(r'[\d.]+'), cmd_message[-1])
            # print('LCH: the cmd para is ', cmd_para)
            if len(cmd_para)==4:
                centerx = float(cmd_para[0])
                dist = float(cmd_para[2])
                self.center_delta = (self.FRAME_WIDTH/2 - centerx)/float(self.FRAME_WIDTH)
                self.dist_delta = -(self.dist_default - dist)
                self.if_track = True
            else:
                rospy.loginfo("The target lost, need redetect ...")
                self.if_track = False


    def run(self):
        rospy.Subscriber('chatter', String, self.callback)
        while self.do_job:
            if self.if_track:
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'base_link'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = self.dist_delta
                goal.target_pose.pose.position.y = self.center_delta
                goal.target_pose.pose.orientation.w = 1.0 #go forward
                print('LCH: the goal is ', goal)

                self.move_base.send_goal(goal)
                # success = self.move_base.wait_for_result(rospy.Duration(10))
                #
                # if not success:
                #     self.move_base.cancel_goal()
                #     rospy.loginfo("The base failed to move forward 3 meters for some reason")
                #     continue
                # else:
                #     # We made it!
                #     state = self.move_base.get_state()
                #     if state == GoalStatus.SUCCEEDED:
                #         rospy.loginfo("Hooray, the base moved 3 meters forward")

            else:
                rospy.loginfo("Holding")

            # self.r.sleep()
            time.sleep(2)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.do_job = False
        exit(0)


if __name__ == '__main__':
    try:
        CMTFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("CMT follower node terminated.")