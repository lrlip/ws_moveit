#!/usr/bin/env python

#author: Lars Lipman
# http://wiki.ros.org/robotican/Tutorials/Arm%20manipulation
# https://quaternions.online/


# roslaunch ur_gazebo ur5.launch limited:=true
# roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true
# roslaunch ur5_moveit_config moveit_rviz.launch config:=true



import sys
import copy
import rospy
import time
import math
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint, PositionConstraint, TrajectoryConstraints, JointConstraint

# Adds a new waypoint to the sets of existing waypoints
def add_waypoint(waypoints, px, py, pz):

	cur_pose = manipulator.get_current_pose().pose
	next_pose = cur_pose
	next_pose.position.x = px
	next_pose.position.y = py
	next_pose.position.z = pz
	# next_pose.orientation.x = 
	# next_pose.orientation.y =
	# next_pose.orientation.z = 
	# next_pose.orientation.w = 

	waypoints.append(copy.deepcopy(next_pose))
	return waypoints

# Contains the home location of the robot
def get_home_location():
	
	home_pose = manipulator.get_current_pose().pose

	home_pose.position.x = 0.6
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
	pick_pose.position.x = 0.3
	pick_pose.position.y = 0.4
	pick_pose.position.z = 0.25

	# pick_pose.orientation.x = 1
	# pick_pose.orientation.y = 0
	# pick_pose.orientation.z = 0
	# pick_pose.orientation.w = 0

	return pick_pose

#Contains the place location of the tool (Perception)
def get_place_location():

	place_pose = manipulator.get_current_pose().pose
	place_pose.position.x = 0.3
	place_pose.position.y = -0.4
	place_pose.position.z = 0.25

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
	(plan_cart, fraction) = manipulator.compute_cartesian_path(waypoints,
											0.01,       # eef_step
											0.0)        # jump threshold
	manipulator.execute(plan_cart, wait=True)
	manipulator.stop()
	manipulator.clear_pose_targets()


def relative_joint_change_ur5(elbow, shoulder_lift, shoulder_pan, wrist1, wrist2, wrist3):
	
	joint_goal = manipulator.get_current_joint_values()
	print joint_goal
	joint_goal[0] += elbow
	joint_goal[1] += shoulder_lift
	joint_goal[2] += shoulder_pan
	joint_goal[3] += wrist1
	joint_goal[4] += wrist2
	joint_goal[5] += wrist3
	print joint_goal

	manipulator.go(joint_goal, wait = True)
	manipulator.stop()

# Makes the robot go to the desired goal
def go_to_point(goal_pose):

	manipulator.set_pose_target(goal_pose)
	plan = manipulator.go(wait=True)
	manipulator.stop()
	print "plan = ", plan

	# if plan == False:
	# 	print "Disabeling constraints..."
	# 	disable_upright_path_constraints()
	# 	plan = manipulator.go(wait=True)

	# 	manipulator.stop()
	# 	manipulator.clear_pose_targets()
	# 	time.sleep(1)
	# 	print "Moved to position", goal_pose

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

#make circular movement
def make_circular_movement(radius):
	loops = 3
	steps = 256
	loop_waypoints = []
	total_waypoints = []
	current_pose = manipulator.get_current_pose().pose
	for i in range(loops): 
		print i
		for j in range(steps): #calculate trajectory
			x_location = current_pose.position.x + radius*math.cos(2*pi/steps * j)
			y_location = current_pose.position.y + radius*math.sin(2*pi/steps * j)
			z_location = current_pose.position.z
			loop_waypoints = add_waypoint(loop_waypoints, x_location, y_location, z_location)

	loop_waypoints.append(copy.deepcopy(current_pose)) #Go back to the middle of the cup

	(circ_plan, fraction) = manipulator.compute_cartesian_path(loop_waypoints, 0.005, 0.0)

	manipulator.execute(circ_plan, wait = True)
	manipulator.stop()
	manipulator.clear_pose_targets()

def rotate_to_quarternion(ox,oy,oz,ow):
	cur_pose = manipulator.get_current_pose().pose
	next_pose = cur_pose
	# next_pose.position.x = px
	# next_pose.position.y = py
	# next_pose.position.z = pz
	next_pose.orientation.x = ox 
	next_pose.orientation.y = oy
	next_pose.orientation.z = oz 
	next_pose.orientation.w = ow

	manipulator.set_pose_target(next_pose)
	plan = manipulator.go(wait = True)

# Stir the coffee
def stir_the_coffee():

	make_circular_movement(0.02)
	carthesian_movement_relative(0.02,0,0.07)
	relative_joint_change_ur5(0,0,0,0,pi/3,0)
	carthesian_movement_relative(0,0,-0.02)
	carthesian_movement_relative(0,0,0.02)
	carthesian_movement_relative(0,0,-0.02)
	carthesian_movement_relative(0,0,0.02)
	
#Create Environment
def create_environment():

	scene.remove_world_object("pole")
	scene.remove_world_object("table")
	scene.remove_world_object("part")
	scene.remove_world_object("groundplane")

	rospy.sleep(1) # Needs to initialize the environment creating

	# table = PoseStamped()
	# table.header.frame_id = robot.get_planning_frame()
	# table.pose.position.x = 0.
	# table.pose.position.y = 0.7
	# table.pose.position.z = 0.05

	# scene.add_box("table", table, (1, 0.5, math.fabs(table.pose.position.z*2)))

	# groundplane = PoseStamped()
	# groundplane.header.frame_id = robot.get_planning_frame()
	# groundplane.pose.position.x = 0.
	# groundplane.pose.position.y = 0.
	# groundplane.pose.position.z = -0.035

	# scene.add_box("groundplane", groundplane, (2,2,0.05))
	
	p = PoseStamped()
	p.header.frame_id = robot.get_planning_frame()
	p.pose.position.x = 0.7
	p.pose.position.y = -0.4
	p.pose.position.z = 0.85
	p.pose.orientation.w = 1.0
	scene.add_box("pole", p, (0.3, 0.1, 1.0))
 
	p.pose.position.y = -0.2
	p.pose.position.z = 0.175
	scene.add_box("table", p, (0.5, 1.5, 0.35))
 
	p.pose.position.x = 0.6
	p.pose.position.y = -0.2
	p.pose.position.z = 0.5
	scene.add_box("part", p, (0.06, 0.06, 0.3))

def Gripper_controller(value):
	if value > 0.8:
		print "Maximum value is 0.8. The value is set back to 0.8"
		value = 0.8
	if value < 0.0:
		print "minimum value is 0.0. Value is set on 0.0"
		value = 0.0

	gripper_pose = endeffector.get_current_joint_values()
	gripper_pose[0] = value
	gripper_pose[1] = -value
	gripper_pose[2] = value
	gripper_pose[3] = value
	gripper_pose[4] = -value
	gripper_pose[5] = value

	endeffector.go(gripper_pose, wait = True)
	endeffector.stop()	


if __name__ == '__main__':

	# intialize moveit commander and rospy node
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('pick_and_place', anonymous = True)

	#this object is the outer leverl interface to the robot
	robot = moveit_commander.RobotCommander()
	print "robot: ", robot
	#this object is the interface to the world surrounding the robot
	scene = moveit_commander.PlanningSceneInterface()

	### Create Environment
	create_environment() #Create the environment
	print "wait..."
	rospy.sleep(2)

	print "The Groupnames are: ", robot.get_group_names()
	manipulator_name    = "manipulator"    #give groupname you want to shoe
	endeffector_name    = "robotiq_gripper"
	manipulator = moveit_commander.MoveGroupCommander(manipulator_name)
	endeffector = moveit_commander.MoveGroupCommander(endeffector_name)


	#Is used to display the trajectoryies in RVIZ
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
		moveit_msgs.msg.DisplayTrajectory, queue_size = 20)
	manipulator.set_planning_time(10)

	############################
	##### START OF PROGRAM #####
	############################

	# Getting the requried pose
	print "Starting pose is: ", manipulator.get_current_pose().pose
	print "manipulator: ", endeffector.get_current_joint_values()


	home_pose = get_home_location()
	go_to_point(home_pose)
	rospy.sleep(1)

	Gripper_controller(0.2)
	Gripper_controller(0.9)
	Gripper_controller(0.0)

	# pick_pose = get_pick_location()
	# go_to_point(pick_pose)
	# rospy.sleep(1)

	# place_pose = get_place_location()
	# go_to_point(place_pose)
	# rospy.sleep(1)
	robot.manipulator.pick("part")


	# rospy.spin()
	# roscpp_shutdown()
