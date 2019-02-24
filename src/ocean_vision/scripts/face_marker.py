#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import String
from cob_perception_msgs.msg import DetectionArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import geometry_msgs
import math
import tf
import cv2
import re
import yaml


class FaceMarker(object):
    def __init__(self):
        self.FRAME_WIDTH = rospy.get_param("~frame_wdith", 640)
        self.FRAME_HEIGHT = rospy.get_param("~frame_height", 480)
        self.Horizontal_angle = rospy.get_param("~horizontal_angle", 70)
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.marker_frame = rospy.get_param("~marker_frame_id", "odom")

        self.depth_map = np.zeros([self.FRAME_HEIGHT, self.FRAME_WIDTH])
        rospy.init_node("face_marker")
        rospy.on_shutdown(self.shutdown)

        self.listener = tf.TransformListener()
        self._bridge = CvBridge()
        self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.depth_callback)

        self.rate = rospy.get_param("~rate", 20)
        r = rospy.Rate(self.rate)
        self.people_id = {}
        self.people_count = 0
        self.Markers_publisher = rospy.Publisher('visualization_people', MarkerArray)
        self.Name_publisher = rospy.Publisher('visualization_name', MarkerArray)
        self.Markers = MarkerArray()
        self.Name = MarkerArray()

        self.run()

    def get_new_marker(self, label):
        marker = Marker()
        marker.header.frame_id = self.marker_frame
        # marker.type = marker.CUBE
        marker.type = marker.MODIFY
        marker.ns = label
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.4
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        return marker

    def get_new_name(self, label):
        text = Marker()
        text.header.frame_id = self.marker_frame
        # text.type = text.TEXT_VIEW_FACING
        text.type = text.MODIFY
        text.text = label
        text.ns = label
        text.scale.x = 0.2
        text.scale.y = 0.2
        text.scale.z = 0.2
        text.color.a = 1.0
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 0.0
        return text

    def callback(self, data):
        def _toRad(angle):
            return angle / 180 * math.pi
        # rospy.loginfo(rospy.get_caller_id() + ' I heard %s', data)
        detections = data.detections
        for i, detection in enumerate(detections):
            detect_point = geometry_msgs.msg.PointStamped()
            detect_point.header.frame_id = "base_link"

            x = float(detection.mask.roi.x)
            y = float(detection.mask.roi.y)
            width = float(detection.mask.roi.width)
            height = float(detection.mask.roi.height)
            label = detection.label
            id = detection.id

            if not label in self.people_id:
                self.people_id[label] = self.people_count
                self.Markers.markers.append(self.get_new_marker(label))
                self.Name.markers.append(self.get_new_name(label))
                self.people_count += 1

            dist = self.get_depth((int(y+height/2), int(x+width/2)))
            alpha = (self.FRAME_WIDTH/2 - (x + width/2)) * self.Horizontal_angle / self.FRAME_WIDTH
            shift = dist * math.tan(_toRad(alpha))

            detect_point.point.x = dist
            detect_point.point.y = shift
            detect_point.point.z = 0

            map_point = self.listener.transformPoint(self.marker_frame, detect_point)

            self.Markers.markers[self.people_id[label]].pose.position.z = self.Markers.markers[self.people_id[label]].scale.z/2
            self.Markers.markers[self.people_id[label]].pose.position.x = map_point.point.x
            self.Markers.markers[self.people_id[label]].pose.position.y = map_point.point.y
            self.Markers.markers[self.people_id[label]].type = self.Markers.markers[self.people_id[label]].CUBE

            self.Name.markers[self.people_id[label]].pose.position.z = self.Markers.markers[self.people_id[label]].scale.z*1.5
            self.Name.markers[self.people_id[label]].pose.position.x = map_point.point.x
            self.Name.markers[self.people_id[label]].pose.position.y = map_point.point.y
            self.Name.markers[self.people_id[label]].type = self.Name.markers[self.people_id[label]].TEXT_VIEW_FACING

            rospy.loginfo(rospy.get_caller_id() + ' the x is {0}, the y is {1}, the width is {2}, the height is {3}, the label is {4}, the id is {5}, i is {6}'.format(x, y, width, height, label, id, i))
            rospy.loginfo("detect_point: (%.2f, %.2f, %.2f) ------> map_point: (%.2f, %.2f, %.2f) at time %.2f",
                          detect_point.point.x, detect_point.point.y, detect_point.point.z,
                          map_point.point.x, map_point.point.y, map_point.point.z, map_point.header.stamp.to_nsec())
        self.Markers_publisher.publish(self.Markers)
        self.Name_publisher.publish(self.Name)

    def depth_callback(self, data):
        try:
            cv_depth = self._bridge.imgmsg_to_cv2(data, "32FC1")
            self.depth_map = cv_depth.copy()
        except CvBridgeError as e:
            print(e)

        # cv2.imshow('depth', self.depth_map)
        # cv2.waitKey(10)

    def get_depth(self, lcoation):
        return self.depth_map[lcoation]/1000


    def run(self):
        # rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('/face_recognizer/faces', DetectionArray, self.callback)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        rospy.sleep(1)


if __name__ == '__main__':

    try:
        FaceMarker()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("CMT follower node terminated.")
