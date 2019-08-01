#!/usr/bin/env python

import sys
import copy
import rospy
import random
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math 




moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_interface', anonymous = True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
print "Group names are: %s" % "," .join(robot.get_group_names())
manipulator = moveit_commander.MoveGroupCommander("panda_arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, queue_size = 1)

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.x = 0.0
pose_target.orientation.y = 0.0
pose_target.orientation.z = 0.0
pose_target.orientation.w = 1

pose_target.position.x = 0.0
pose_target.position.y = 0.35
pose_target.position.z = 0.6
manipulator.set_pose_target(pose_target)



# group_variable_values = group.get_current_joint_values() # get variable of joints

# group_variable_values[0] = random.uniform(-1.5,1.5) 
# group_variable_values[1] = random.uniform(-1.5,1.5) 
# group_variable_values[2] = random.uniform(-1.5,1.5)         #assign to joint 5 variable
# group_variable_values[3] = random.uniform(-1.5,1.5)          #assign to joint 5 variable
# group_variable_values[4] = random.uniform(-1.5,1.5)          #assign to joint 5 variable
# group_variable_values[5] = random.uniform(-1.5,1.5)         #assign to joint 5 variable
# group_variable_values[6] = random.uniform(-1.5,1.5) 

# group.set_joint_value_target(group_variable_values)    #set new variable

plan1 = manipulator.plan()    #planning trajectory
manipulator.go(wait = True)

moveit_commander.roscpp_shutdown()
