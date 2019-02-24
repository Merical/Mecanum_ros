#! /usr/bin python

import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    listener.waitForTransform("/odom", "/base_link", rospy.Time(), rospy.Duration(4))
    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            listener.waitForTransform("/odom", "/base_link", now, rospy.Duration(1)) # transform from map to base_link
            (trans, rot) = listener.lookupTransform('/odom', '/base_link', now)
            print(trans)
            print(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('pass')

        # angular = 4 * math.atan2(trans[1], trans[0])
        # linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        # cmd = geometry_msgs.msg.Twist()
        # cmd.linear.x = linear
        # cmd.angular.z = angular
        # turtle_vel.publish(cmd)

        rate.sleep()