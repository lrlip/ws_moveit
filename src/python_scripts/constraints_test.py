#!/usr/bin/env python

#author: Lars Lipman
# http://wiki.ros.org/robotican/Tutorials/Arm%20manipulation
# https://quaternions.online/

import sys
import copy
import rospy
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint, PositionConstraint, TrajectoryConstraints, JointConstraint

def add_waypoint(waypoints, new_point):

	cur_pose = manipulator.get_current_pose().pose
	print "add_waypoint: new point", new_point
	next_pose = cur_pose
	next_pose.position.x = new_point[0]
	next_pose.position.y = new_point[1]
	next_pose.position.z = new_point[2]
	# next_pose.orientation.x = 
	# next_pose.orientation.y =
	# next_pose.orientation.z = 
	# next_pose.orientation.w = 

	waypoints.append(copy.deepcopy(next_pose))
	return waypoints

# Contains the home location of the robot
def get_home_location():
	
	home_pose = manipulator.get_current_pose().pose

	home_pose.position.x = 0.5
	home_pose.position.y = 0.0
	home_pose.position.z = 0.6

	#UR5
	# home_pose.orientation.x = 0.5
	# home_pose.orientation.y = 0.5
	# home_pose.orientation.z = -0.5
	# home_pose.orientation.w = 0.5

	#IIWA
	home_pose.orientation.x = -1.0
	home_pose.orientation.y = 0.0
	home_pose.orientation.z = 0.0
	home_pose.orientation.w = 0.0

	return home_pose

# Contains the pickup location of the tool (PERCEPTION)
def get_pick_location():

	pick_pose = manipulator.get_current_pose().pose

	pick_pose.position.x = 0.2
	pick_pose.position.y = 0.4
	pick_pose.position.z = 0.3

	# If same pose required dont change
	# pick_pose.orientation.x = 1
	# pick_pose.orientation.y = 0
	# pick_pose.orientation.z = 0
	# pick_pose.orientation.w = 0

	return pick_pose

#Contains the place location of the tool (Perception)
def get_place_location():

	place_pose = manipulator.get_current_pose().pose

	place_pose.position.x = 0.2
	place_pose.position.y = -0.4
	place_pose.position.z = 0.3

	# If same pose required dont change
	# place_pose.orientation.x = 1
	# place_pose.orientation.y = 0
	# place_pose.orientation.z = 0
	# place_pose.orientation.w = 0

	return place_pose

# Creates carthesians movements, needed for the last 10 cm of the pick and place
def carthesian_movement_relative(rela_x, rela_y, rela_z):

	waypoints = []

	start_pick_pose = manipulator.get_current_pose().pose
	start_pick_pose.position.x += rela_x
	start_pick_pose.position.y += rela_y
	start_pick_pose.position.z += rela_z


	waypoints.append(copy.deepcopy(start_pick_pose))
	(plan_cart, fraction) = manipulator.compute_cartesian_path(
		waypoints,
											0.01,       # eef_step
											0.0)        # jump threshold
	manipulator.execute(plan_cart, wait=True)
	manipulator.stop()
	manipulator.clear_pose_targets()

# Makes the robot go to the desired goal
def go_to_point(goal_pose):

	manipulator.set_pose_target(goal_pose)
	plan = manipulator.go(wait=True)
	print "plan = ", plan
	if plan == False:
		print "Disabeling constraints..."
		disable_upright_path_constraints()
		plan = manipulator.go(wait=True)

		manipulator.stop()
		manipulator.clear_pose_targets()
		time.sleep(1)
		print "Moved to position", goal_pose

#Sequence for picking up object: Could possible be combined with placing
def pick_up_object():
	
	carthesian_movement_relative(0,0,-0.1)
	#CLOSE FINGERS
	print "closing fingers"
	time.sleep(1)

	carthesian_movement_relative(0,0,0.1)
	print "back at pickup position"
	time.sleep(1)

#Sequence for placing object: Could be Combined with picking
def place_object():
	
	carthesian_movement_relative(0,0,-0.1)
	#open FINGERS
	print "opening fingers"
	time.sleep(1)

	carthesian_movement_relative(0,0,0.1)
	print "back at placing position"
	time.sleep(1)

def init_upright_path_constraints():

	cur_pose = manipulator.get_current_pose().pose
	upright_constraints = Constraints()

	upright_constraints.name = "upright"
	orientation_constraint = OrientationConstraint()
	# orientation_constraint.header = pose.header
	orientation_constraint.header.frame_id = "world"
	orientation_constraint.link_name = manipulator.get_end_effector_link()
	print "end link: ", manipulator.get_end_effector_link()
	orientation_constraint.orientation = cur_pose.orientation
	#IIWA
	orientation_constraint.absolute_x_axis_tolerance = 0.1
	orientation_constraint.absolute_y_axis_tolerance = 0.1
	orientation_constraint.absolute_z_axis_tolerance = 3.14

	#UR5
	# orientation_constraint.absolute_x_axis_tolerance = 3.14
	# orientation_constraint.absolute_y_axis_tolerance = 0.1
	# orientation_constraint.absolute_z_axis_tolerance = 0.1

	orientation_constraint.weight = 1.0

	upright_constraints.orientation_constraints.append(orientation_constraint)

	return upright_constraints

def limit_base_joint_constraint():

	ur5_constraints = Constraints()
	shoulder_pan_joint_constraint = JointConstraint()
	shoulder_lift_joint_constraint = JointConstraint()
	wrist_1_joint_constraint = JointConstraint()

	shoulder_pan_joint_constraint.joint_name = "shoulder_pan_joint"
	shoulder_pan_joint_constraint.position = 0
	shoulder_pan_joint_constraint.tolerance_above = 3.14
	shoulder_pan_joint_constraint.tolerance_below = 3.14
	shoulder_pan_joint_constraint.weight = 1

	shoulder_lift_joint_constraint.joint_name = "shoulder_lift_joint"
	shoulder_lift_joint_constraint.position = 0
	shoulder_lift_joint_constraint.tolerance_above = 0
	shoulder_lift_joint_constraint.tolerance_below = 3.14
	shoulder_lift_joint_constraint.weight = 1

	wrist_1_joint_constraint.joint_name = "wrist_1_joint"
	wrist_1_joint_constraint.position = 0
	wrist_1_joint_constraint.tolerance_above = 3.14
	wrist_1_joint_constraint.tolerance_below = 3.14
	wrist_1_joint_constraint.weight = 1

	ur5_constraints.joint_constraints.append(shoulder_pan_joint_constraint)
	ur5_constraints.joint_constraints.append(shoulder_lift_joint_constraint)
	ur5_constraints.joint_constraints.append(wrist_1_joint_constraint)

	return ur5_constraints

def enable_upright_path_constraints(constraints):
	manipulator.set_path_constraints(constraints)

def disable_upright_path_constraints():
	manipulator.set_path_constraints(None)

if __name__ == '__main__':

	# intialize moveit commander and rospy node
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('pick_and_place', anonymous = True)

	#this object is the outer leverl interface to the robot
	robot = moveit_commander.RobotCommander()
	#this object is the interface to the world surrounding the robot
	scene = moveit_commander.PlanningSceneInterface()

	# print "The Groupnames are: ", robot.get_group_names()
	manipulator_name    = "manipulator"    #give groupname you want to shoe
	endeffector_name    = "endeffector"
	manipulator = moveit_commander.MoveGroupCommander(manipulator_name)
	endeffector = moveit_commander.MoveGroupCommander(endeffector_name)


	#Is used to display the trajectoryies in RVIZ
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
		moveit_msgs.msg.DisplayTrajectory, queue_size = 20)
	manipulator.set_planning_time(10)

	############### START OF PROGRAM #################
	
	###############################
	### Orientation Constraints ###
	###############################

	# #Getting the required positions
	# # print "Starting pose is: ", manipulator.get_current_pose().pose

	# #Moving the sequence
	# disable_upright_path_constraints() #check if all constraints are released

	# home_pose = get_home_location()
	# go_to_point(home_pose)

	# #picking up sequence
	# pick_pose = get_pick_location()
	# # upright_constraints = init_upright_path_constraints()
	# # enable_upright_path_constraints(upright_constraints)
	# go_to_point(pick_pose)
	# disable_upright_path_constraints()


	# place_pose = get_place_location()
	# upright_constraints = init_upright_path_constraints()
	# enable_upright_path_constraints(upright_constraints)
	# go_to_point(place_pose)
	# disable_upright_path_constraints()

	# 	#place object
	# # place_object()
	# # upright_constraints = init_upright_path_constraints()
	# # enable_upright_path_constraints(upright_constraints)
	# go_to_point(home_pose)
	# disable_upright_path_constraints()	

	################################
	### Joint Constraint on BASE ###
	################################

	# Getting the requried pose
	print "Starting pose is: ", manipulator.get_current_pose().pose

	disable_upright_path_constraints()

	ur5_constraint = limit_base_joint_constraint()
	enable_upright_path_constraints(ur5_constraint)

	home_pose = get_home_location()
	go_to_point(home_pose)

	pick_pose = get_pick_location()

	go_to_point(pick_pose)


	place_pose = get_place_location()
	disable_upright_path_constraints()
	upright_constraints = init_upright_path_constraints()
	enable_upright_path_constraints(upright_constraints)
	go_to_point(place_pose)

	go_to_point(home_pose)

