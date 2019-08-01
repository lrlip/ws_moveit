#!/usr/bin/env python

#author: Lars Lipman
# Maintained by :Lars Lipman
# Email: lars@lipman.eu
# http://wiki.ros.org/robotican/Tutorials/Arm%20manipulation
# https://quaternions.online/

import sys, copy, rospy, time, tf, math, moveit_commander, actionlib
import numpy as np
import moveit_msgs.msg
import geometry_msgs.msg
import time
import tf
from math import pi
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint, PositionConstraint, TrajectoryConstraints, JointConstraint

obj_mark = {}
obj_mark["mug_01"] = ["1", "11"]
obj_mark["mug_02"] = ["2", "12"]
obj_mark["mug_03"] = ["3", "13"]
obj_mark["placemat_01"] = ["5"]
obj_mark["placemat_02"] = ["6"]
obj_mark["placemat_03"] = ["7"]


class Item(object):

	pose = None

	def __init__(self, name):
		self.name = name


	def get_location(self,times):
		# self.pose = MADHUKAR_location(self.name)
		name = self.name
		## WORKAROUND
		if name == "home":
			pose = get_home_location()
		elif name == "mug":
			pose = get_mug_location(times)
		elif name == "mug_place":
			pose = get_mug_place_location(times)
		elif name == "coffee_spoon":
			pose = get_coffee_spoon_location()
		elif name == "coffee_container":
			pose = get_coffee_container_location()

		self.pose = pose

	def go_to_location(self,times=None):

		self.get_location(times)
		manipulator_pose = manipulator.get_current_pose().pose
		# if  self.pose == manipulator_pose:
		# 	continue
		manipulator.set_pose_target(self.pose)
		plan = manipulator.go(wait = True)
		manipulator.stop()
		manipulator.clear_pose_targets()



	def carthesian_movement_absolute(self, x = None, y = None, z = None ):
		waypoints = []
		self.get_location()
	
		goal_pose = manipulator.get_current_pose().pose
		goal_pose.position.x = self.pose.position.x + x
		goal_pose.position.y = self.pose.position.y + y
		goal_pose.position.z = self.pose.position.z + z

		waypoints.append(copy.deepcopy(goal_pose))
		(plan_cart, fraction) = manipulator.compute_cartesian_path(waypoints, 
												0.01,
												0.0)
		manipulator.execute(plan_cart, wait=True)
		manipulator.stop()
		manipulator.clear_pose_targets()

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

def create_environment():

	scene.remove_world_object("groundplane")

	rospy.sleep(1) # Needs to initialize the environment creating

	p = PoseStamped()
	p.header.frame_id = robot.get_planning_frame()
	# p.pose.position.x = 0.7
	# p.pose.position.y = 0.0
	# p.pose.position.z = 0.1

	# scene.add_box("table", p, (0.9, 0.9, math.fabs(p.pose.position.z*2)))

	p.pose.position.x = 0.
	p.pose.position.y = 0.
	p.pose.position.z = -0.05
	scene.add_box("groundplane", p, (2,2,0.1))

	p.pose.position.x = 0.8
	p.pose.position.y = 0.
	p.pose.position.z = 0.3
	scene.add_box("table", p, (0.7,1.5,0.05))

	p.pose.position.x = 0.
	p.pose.position.y = 0.7
	p.pose.position.z = 0.6
	scene.add_box("schelf",p,(1,0.3,0.05))

	p.pose.position.x = 0.55
	p.pose.position.y = 0.7
	p.pose.position.z = 0.7
	scene.add_box('Left_shelf',p,(0.1,0.3,0.1))

	p.pose.position.x = -0.55
	p.pose.position.y = 0.7
	p.pose.position.z = 0.7
	scene.add_box('right_shelf',p,(0.1,0.3,0.1))
	
# Contains the home location of the robot
def make_goal_pose(posx,posy,posz,rotz,roty,rotx):

	quaternion = tf.transformations.quaternion_from_euler(rotz, roty, rotx, 'rzyx')
	euler = tf.transformations.euler_from_quaternion(quaternion,'sxyz')
	print euler
	R = tf.transformations.euler_matrix(euler[0],euler[1], euler[2])
	print R
	goal_pose = manipulator.get_current_pose().pose
	print goal_pose
	goal_pose.position.x = posx - R[0][2]*Gripper_length
	goal_pose.position.y = posy - R[1][2]*Gripper_length
	goal_pose.position.z = posz - R[2][2]*Gripper_length
	print goal_pose
	goal_pose.orientation.x = quaternion[0]
	goal_pose.orientation.y = quaternion[1]
	goal_pose.orientation.z = quaternion[2]
	goal_pose.orientation.w = quaternion[3]
	print goal_pose

	return goal_pose

def get_home_location():
	
	goal_pose = make_goal_pose(0.5,			#position object
								0.0,		#position object
								0.6,		#position object
								pi/2,		#rotz
								pi,			#roty
								0)			#rotx
	# goal_pose.position.x = 0.5
	# goal_pose.position.y = 0.0
	# goal_pose.position.z = 0.7

	# quaternion = tf.transformations.quaternion_from_euler(pi/2,pi,0, 'rzyx')
	# goal_pose.orientation.x = quaternion[0]
	# goal_pose.orientation.y = quaternion[1]
	# goal_pose.orientation.z = quaternion[2]
	# goal_pose.orientation.w = quaternion[3]

	return goal_pose

# Contains the pickup location of the tool (PERCEPTION)
def get_mug_location(xtimes = 0):


	goal_pose = make_goal_pose(-0.2+0.2*xtimes,			#position object
								0.7 - 0.1,				#position object - approach
								0.75,					#position object
								0,						#rotz
								0,						#roty
								-pi/2)					#rotx

	return goal_pose

#Contains the place location of the tool (Perception)
def get_mug_place_location(ytimes = 0):

	goal_pose = make_goal_pose(0.7 ,			#position object
								-0.4 + 0.1*ytimes,				#position object - approach
								0.4 + 0.13,					#position object
								-pi/2,						#rotz
								0,						#roty
								-pi/2)					#rotx

	return goal_pose

def get_coffee_spoon_location():

	goal_pose = make_goal_pose(0.75-0.15, 
								0,
								0.5,
								0, 
								-pi/4,
								pi)

	return goal_pose

# def get_coffee_container_location():

# 	goal_pose = manipulator.get_current_pose().pose
# 	goal_pose.position.x = 0.6
# 	goal_pose.position.y = 0.1 #till 0.4
# 	goal_pose.position.z = 0.45

# 	quaternion = tf.transformations.quaternion_from_euler(0,pi/4,0, 'rzyx')
# 	goal_pose.orientation.x = quaternion[0]
# 	goal_pose.orientation.y = quaternion[1]
# 	goal_pose.orientation.z = quaternion[2]
# 	goal_pose.orientation.w = quaternion[3]
# 	return goal_pose

def get_coffee_end_container_location():
	
	goal_pose = manipulator.get_current_pose().pose
	goal_pose.position.x += -0.0
	goal_pose.position.y += 0 #till 0.4
	goal_pose.position.z += -0.05

	quaternion = tf.transformations.quaternion_from_euler(pi,-pi/2,0, 'rzyx')
	goal_pose.orientation.x = quaternion[0]
	goal_pose.orientation.y = quaternion[1]
	goal_pose.orientation.z = quaternion[2]
	goal_pose.orientation.w = quaternion[3]
	return goal_pose

def carthesian_movement_gripper_direction(distance):

	q = manipulator.get_current_pose().pose
	quaternion = [q.orientation.x, q.orientation.y, q.orientation.z, q.orientation.w]
	euler = tf.transformations.euler_from_quaternion(quaternion,'sxyz')
	R = tf.transformations.euler_matrix(euler[0],euler[1], euler[2])
	carthesian_movement_relative(R[0][2]*distance,R[1][2]*distance,R[2][2]*distance)

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

#Gripper close
def gripper_close():
	# gripper_goal.command.position = 0.43 #must be 0.4
	gripper_goal.command.position = 0.28
	gripper.send_goal(gripper_goal)
	gripper.wait_for_result()

#Gripper open
def gripper_open():
	# gripper_goal.command.position = 0
	gripper_goal.command.position = 0
	gripper.send_goal(gripper_goal)
	gripper.wait_for_result()

#Go to location when item is not known
def no_item_go_to_location(goal_pose):

	manipulator.set_pose_target(goal_pose)
	plan = manipulator.go(wait = True)
	manipulator.stop()
	manipulator.clear_pose_targets()

#Sequence for picking up object: Could possible be combined with placing
def pick_up_object():
	
	# Make sure hand is open
	gripper_open()
	carthesian_movement_gripper_direction(0.1)
	#CLOSE FINGERS
	gripper_close()
	carthesian_movement_relative(0,0,0.02)
		
#Sequence for placing object: Could be Combined with picking
def place_object():

	carthesian_movement_relative(0,0,-0.1)

	#open FINGERS
	gripper_open()
	carthesian_movement_gripper_direction(-0.1)
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
	manipulator.stop()
# Stir the coffee
def stir_the_coffee():

	make_circular_movement(0.02)
	carthesian_movement_relative(0.02,0,0.07)
	iiwa_joint_move(0,0,0,0,0,pi/2,0) ; raw_input("press space")
	carthesian_movement_relative(0,0,-0.02)
	carthesian_movement_relative(0,0,0.02)
	carthesian_movement_relative(0,0,-0.02)
	carthesian_movement_relative(0,0,0.02)
	
# Depostis the coffe
def iiwa_joint_move(j1, j2, j3, j4, j5, j6, j7):
	joint_goal = manipulator.get_current_joint_values()
	print "joint", joint_goal
	joint_goal[0] += j1
	joint_goal[1] += j2
	joint_goal[2] += j3
	joint_goal[3] += j4
	joint_goal[4] += j5
	joint_goal[5] += j6
	joint_goal[6] += j7
	# joint_goal[7] += gripper
	print "turning"
	manipulator.go(joint_goal, wait=True)
	manipulator.stop()


def init_orient_constraints(x_tol, y_tol, z_tol):

	if x_tol < 0.1:
		x_tol = 0.1
	if y_tol <0.1:
		y_tol = 0.1
	if z_tol < 0.1:
		z_tol = 0.1

	print x_tol, y_tol, z_tol
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
	orientation_constraint.absolute_x_axis_tolerance = x_tol
	orientation_constraint.absolute_y_axis_tolerance = y_tol
	orientation_constraint.absolute_z_axis_tolerance = z_tol

	orientation_constraint.weight = 1.0

	upright_constraints.orientation_constraints.append(orientation_constraint)
	return upright_constraints

def enable_upright_path_constraints(constraints):
	manipulator.set_path_constraints(constraints)

def disable_upright_path_constraints():
	manipulator.set_path_constraints(None)

if __name__ == '__main__':

	# intialize moveit commander and rospy node
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('pick_and_place', anonymous = True)

	rospy.wait_for_service("/clear_octomap")
	clear_octomap = rospy.ServiceProxy("/clear_octomap", Empty)

	gripper = actionlib.SimpleActionClient("iiwa/gripper_controller/gripper_cmd", GripperCommandAction)
	gripper.wait_for_server()

	manipulator_name    = "manipulator"    #give groupname you want to shoe
	endeffector_name    = "gripper"
	robot 		= moveit_commander.RobotCommander()
	manipulator = moveit_commander.MoveGroupCommander(manipulator_name)
	endeffector = moveit_commander.MoveGroupCommander(endeffector_name)
	# manipulator.allow_replanning(True)
	tf_listener = tf.TransformListener()
	rate = rospy.Rate(10)

	gripper_goal = GripperCommandGoal()
	gripper_goal.command.max_effort = 10.0

	#this object is the interface to the world surrounding the robot
	scene = moveit_commander.PlanningSceneInterface()
	### Create Environment
	create_environment() #Create the environment
	#Is used to display the trajectoryies in RVIZ
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
		moveit_msgs.msg.DisplayTrajectory, queue_size = 20)
	manipulator.set_planning_time(10)
	############################
	##### START OF PROGRAM #####
	############################
	home = Item("home")
	mug_place = Item("mug_place")
	mug = Item("mug")
	coffee_spoon = Item("coffee_spoon")
	coffee_container = Item("coffee_container")
	
	Gripper_length = 0.13 #Define global variable

	# disable_upright_path_constraints()
	home.go_to_location(); 			print "go_to_location home"

	for i in range(3):
		mug.go_to_location(i); 			print "go_to_location mug"
		pick_up_object(); 				print "pick_up_object"
		mug_place.go_to_location(i)
		place_object()

	print "go to coffee_spoon"
	for i in range(2):
		coffee_spoon.go_to_location(); 	print "go_to_location coffee_spoon"
		carthesian_movement_gripper_direction(0.1)
		gripper_close()
		carthesian_movement_relative(0.2,0,0) 
		carthesian_movement_relative(0,0,0.05)
		coffee_end_container_pose = get_coffee_end_container_location() 
		no_item_go_to_location(coffee_end_container_pose) 
		

		#keep the current pose during operation
		goal_pose = get_mug_place_location(0) 
		cur_pose = manipulator.get_current_pose().pose 
		goal_pose.orientation = cur_pose.orientation 
	# goal_pose.orientation.y = cur_pose.orientation.y
	# goal_pose.orientation.z = cur_pose.orientation.z
	# goal_pose.orientation.w = cur_pose.orientation.w
		no_item_go_to_location(goal_pose)
		iiwa_joint_move(0,0,0,0,0,0,pi/2) 
		iiwa_joint_move(0,0,0,0,0,0,-pi/2) 


	goal_pose = get_home_location()
	mug_pose = get_mug_location()
	goal_pose.position = mug_pose.position

	no_item_go_to_location(goal_pose)









	# for i in range(2):
	# 	print "go to coffee_container: ", i
	# coffee_container.go_to_location()
	# 	print "taking coffee:, ", i
	# 	take_coffee()
	# 	mug_place.carthesian_movement_absolute(0,0,0)
	# 	print "go to mug location", i
	# 	mug_place.go_to_location()
	# 	disable_upright_path_constraints()
	# 	print "deposit coffee:, ", i
	# 	deposit_coffee()

	# coffee_spoon.go_to_location()
	# place_object()

	# rospy.sleep(5)



	# table = Item("mug_place")
	# table.go_to_location()
	# place_object()
