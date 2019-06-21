#!/usr/bin/env python
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt

class NavTest():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 3)

        # Are we running in the fake simulator?
        self.fake_test = rospy.get_param("~fake_test", False)

        # Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED','SUCCEEDED',
                       'ABORTED', 'REJECTED','PREEMPTING', 'RECALLING',
                       'RECALLED','LOST']

        # Set up the goal locations. Poses are defined in the map frame.
        # An easy way to find the pose coordinates is to point-and-click
        # Nav Goals in RViz when running in the simulator.
        # Pose coordinates are then displayed in the terminal
        # that was used to launch RViz.
        locations = list()

        locations.append(Pose(Point(2.882, -0.605, 0.000),
                                       Quaternion(0.000, 0.000, -0.655, 0.755)))
        locations.append(Pose(Point(-0.342, -3.026, 0.000),
                                         Quaternion(0.000, 0.000, 0.998, -0.069)))
        locations.append(Pose(Point(-3.658, 0.778, 0.000),
                                         Quaternion(0.000, 0.000, 0.903, 0.429)))

        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/mobile_base_controller/cmd_vel', Twist, queue_size=5)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")

        self.move_base.wait_for_server(rospy.Duration(30))
        rospy.loginfo("Connected to move base server")

        # A variable to hold the initial pose of the robot to be set by the user in RViz
        initial_pose = PoseWithCovarianceStamped()
        # Variables to keep track of success rate, running time, and distance traveled
        n_locations = len(locations)
        n_goals = 0
        n_successes = 0
        i = 0
        distance_traveled = 0
        start_time = rospy.Time.now()
        running_time = 0
        location = ""
        last_location = ""
        # Get the initial pose from the user
        rospy.loginfo("Click on the map in RViz to set the intial pose...")
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        self.last_location = Pose()
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
        # Make sure we have the initial pose
        while initial_pose.header.stamp == "":
            rospy.sleep(1)
        rospy.loginfo("Starting navigation test")

        # Begin the main loop and run through a sequence of locations
        while not rospy.is_shutdown():
            loc = locations[i]

            # Set up the next goal location
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = loc
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()

            # Let the user know where the robot is going next
            rospy.loginfo("Going to: " + str(loc))
            # Start the robot toward the next location
            self.move_base.send_goal(self.goal)

            # Allow 5 minutes to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))

            # Check for success or failure
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                else:
                    rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))

            # How long have we been running?
            running_time = rospy.Time.now() - start_time
            running_time = running_time.secs / 60.0

            i += 1
            if i < 3:
                rospy.sleep(self.rest_time)
            else:
                rospy.signal_shutdown('close')

    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)



def trunc(f, n):

    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])

if __name__ == '__main__':
    try:
        NavTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")
