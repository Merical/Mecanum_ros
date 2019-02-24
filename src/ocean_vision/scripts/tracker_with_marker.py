#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import roslib;roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math
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
        f = open('/home/lishenghao/ros_workspace/my_handsfree_ws/src/ocean_vision/config/tracker.yaml', 'r')
        config = yaml.load(f.read())
        self.FRAME_WIDTH = config['Video']['FRAME_WIDTH']
        self.FRAME_HEIGHT = config['Video']['FRAME_HEIGHT']
        self.dist_default = config['Handsfree']['dist_default']
        self.Horizontal_angle = config['Video']['Horizontal']
        # self.Horizontal_angle = self.Horizontal_angle / 180 * math.pi

        self.center_delta = 0
        self.dist_delta = 0
        self.if_track = False
        self.do_job = True

        rospy.init_node("cmt_tracker")

        self.pubilsher = rospy.Publisher('visualization_marker', Marker)
        self.Marker = Marker()
        self.Marker.header.frame_id = "base_link"
        self.Marker.type = self.Marker.CUBE
        self.Marker.action = self.Marker.MODIFY
        self.Marker.ns = "Target_Marker"
        self.Marker.scale.x = 0.2
        self.Marker.scale.y = 0.2
        self.Marker.scale.z = 0.2
        self.Marker.color.a = 1.0
        self.Marker.color.r = 1.0
        self.Marker.color.g = 1.0
        self.Marker.color.b = 0.0

        self.text = Marker()
        self.text.header.frame_id = "base_link"
        self.text.type = self.text.TEXT_VIEW_FACING
        self.text.action = self.text.MODIFY
        self.text.text = "Target"
        self.text.ns = "Target_Name"
        self.text.scale.x = 0.2
        self.text.scale.y = 0.2
        self.text.scale.z = 0.2
        self.text.color.a = 1.0
        self.text.color.r = 1.0
        self.text.color.g = 1.0
        self.text.color.b = 0.0

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("wait for the action server to come up")
        self.move_base.wait_for_server(rospy.Duration(5))
        rospy.loginfo("waiting done")

        self.rate = rospy.get_param("~rate", 10)
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
                self.get_marker(centerx, dist)
                self.center_delta = (self.FRAME_WIDTH/2 - centerx)/float(self.FRAME_WIDTH)
                self.dist_delta = -(self.dist_default - dist)
                self.if_track = True
            else:
                rospy.loginfo("The target lost, need redetect ...")
                self.if_track = False

    def get_marker(self, centerx, dist):
        def _toRad(angle):
            return angle / 180 * math.pi

        alpha = (self.FRAME_WIDTH/2 - centerx) * self.Horizontal_angle / self.FRAME_WIDTH
        # print('LCH: the alpha is ', alpha)
        shift = dist * math.tan(_toRad(alpha))
        # print('LCH: the shift is ', shift)
        # self.Marker.pose.orientation.w = 1.0
        # self.Marker.pose.position.x = shift
        # self.Marker.pose.position.y = dist
        self.Marker.pose.position.z = self.Marker.scale.z / 2
        self.Marker.pose.position.x = dist
        self.Marker.pose.position.y = shift

        self.text.pose.position.z = self.Marker.scale.z * 1.5
        self.text.pose.position.x = dist
        self.text.pose.position.y = shift

        self.pubilsher.publish(self.Marker)
        self.pubilsher.publish(self.text)


    def run(self):
        rospy.Subscriber('chatter', String, self.callback)
        count = 0
        while self.do_job:
            if self.if_track:
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'base_link'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = self.dist_delta
                goal.target_pose.pose.position.y = self.center_delta
                goal.target_pose.pose.orientation.w = 1.0 #go forward
                # print('LCH: the goal is ', goal)

                self.move_base.send_goal(goal)
                # if count == 20:
                #     self.move_base.send_goal(goal)
                #     count = 0
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
            count += 1

            self.r.sleep()
            # time.sleep(2)


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