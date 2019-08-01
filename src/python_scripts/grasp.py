#! /usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('grasp', anonymous = True)
robot = moveit_commander.RobotCommander()