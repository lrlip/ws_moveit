#!/usr/bin/env python

''' 
author: Lars Lipman 
Maintained by :Lars Lipman 
Email: lars@lipman.eu
Last modified: 12-July-19

Usefull Sites
http://wiki.ros.org/robotican/Tutorials/Arm%20manipulation
https://quaternions.online/

This python file contains the script for moving the arm of the kuka arm in
combination with the alvar marker trackers. The different steps in this document
contains notes of the code description.

RUN:
roslaunch iiwa_gazebo iiwa_gazebo.launch
roslaunch ar_track_alvar ar_track_kinect.launch
roslaunch robotiq_iiwa14 robotiq_iiwa14_planning_execution.launch 

./sequence_program_ar.py



To Be Done

1. The tracking of the ar_tracking in RVIZ is not robust, a better
way of keeping track of the tf needs to be found. Proposed method: compose a
period in which the positions of the tackers are saved. When the desired tracker
is needed, first check if the actual current position is found, if not take the
marker positon out of the library.

2. Motion planning: The motion planning by RVIZ is not robust, sometimes no
path is found. Check if a method exist in which always a path is found, or build
a motion planner by yourself. At the moment the motion planning is done on the basis
of minimal effort (So the lest cost of energy). Try to get rid of this effort-control,
for more smooth paths.
	a. Carthesian Path
	The problem with the current Carthesian planner is that it wants to make a straigh 
	path without taking obstacles, (or itself) into account. Possible a new method could
	be written that make sure that the shortes carthesian path is made without collsion.

3. The gripping in gazebo don't work, since the gripper is position controlled. A possible 
solution could be to implement an effor controller in 



'''

import sys, copy, rospy, time, tf, moveit_commander, actionlib
import numpy as np
import moveit_msgs.msg
import geometry_msgs.msg
import time
from math 					import pi
from ar_track_alvar_msgs.msg import AlvarMarkers
from control_msgs.msg 		import (GripperCommandAction, GripperCommandGoal)
from geometry_msgs.msg 		import PoseStamped
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg 		import RobotState, Constraints, OrientationConstraint, PositionConstraint, TrajectoryConstraints, JointConstraint
from quaternion 			import Quaternion
from std_msgs.msg 			import String
from std_srvs.srv 			import Empty
from tf 					import TransformListener

#Create a list of alvar markers 
obj_mark = {}
obj_mark["mug_01"] = ["ar_marker_11"] #, 11]
obj_mark["mug_02"] = ["ar_marker_12"] #, 12]
obj_mark["mug_03"] = ["ar_marker_13"] #, 13]
obj_mark["placemat_01"] = ["ar_marker_5"]
obj_mark["placemat_02"] = ["ar_marker_6"]
obj_mark["placemat_03"] = ["ar_marker_7"]

Gripper_length = 0.22 #Define global variable



class ArTagReader(object):
	def __init__(self):
		self.markers = []

	def callback(self, msg):
		self.markers = msg.markers

class Item(object):

	pose = None

	def __init__(self, name):
		self.name = name


	def get_location(self):

		if self.name == 'home':
			self.pose = get_home_location()
		else: 
			reader = ArTagReader()
			quat = Quaternion()
			self.tf_listener = TransformListener()
			rate = rospy.Rate(10.0)

			#get the location wrt world frame
			goal_ar_tag = ''.join(obj_mark[self.name])
			# self.tf_listener.waitForTransform('/world','/camera_link',rospy.Time(), rospy.Duration(4.0))
			# camera_pose, camera_rot = self.tf_listener.lookupTransform('/world', '/camera_link', rospy.Time(0))
			self.tf_listener.waitForTransform('/world',goal_ar_tag,rospy.Time(), rospy.Duration(10.0))
			ar_tag_pose, ar_tag_rot = self.tf_listener.lookupTransform('/world', goal_ar_tag, rospy.Time(0))

			#Set location of the marker in world frame
			marker_pose = manipulator.get_current_pose().pose
			marker_pose.position.x = ar_tag_pose[0]
			marker_pose.position.y = ar_tag_pose[1]
			marker_pose.position.z = ar_tag_pose[2]
			print self.name

			# check orientation of the gripper for the mugges
			if abs(marker_pose.position.x) >= abs(marker_pose.position.y):
				if marker_pose.position.x >= 0:

					print 'object in front'
					goal_pose = make_goal_pose(marker_pose.position.x ,
												marker_pose.position.y,
												marker_pose.position.z+0.1,
												-pi/2,
												0,
												-pi/2)
				elif marker_pose.position.x < 0:
					print 'object back'
					goal_pose = make_goal_pose(marker_pose.position.x,
												marker_pose.position.y,
												marker_pose.position.z,
												pi/2,
												0,
												-pi/2)
			elif abs(marker_pose.position.x) < abs(marker_pose.position.y):
				if marker_pose.position.y >= 0:
					print 'object left'
					goal_pose = make_goal_pose(marker_pose.position.x,
												marker_pose.position.y,
												marker_pose.position.z+0.1,
												0,
												0,
												-pi/2)
				elif marker_pose.position.y < 0:
					print 'object right'
					goal_pose = make_goal_pose(marker_pose.position.x,
												marker_pose.position.y,
												marker_pose.position.z,
												pi,
												0,
												-pi/2)
			print "if statement", goal_pose
			self.pose = goal_pose


	def go_to_location(self):

		#TODO Check at arrival if the mug is still at the place
		self.get_location()
		# manipulator_pose = manipulator.get_current_pose().pose
		# if  self.pose == manipulator_pose:
		# 	continue
		manipulator.set_pose_target(self.pose)
		plan = manipulator.go(wait = True)
		manipulator.stop()
		manipulator.clear_pose_targets()


	#Fucntion to make the endeffector follow an absolute straight path
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

	# p.pose.position.x = 0.
	# p.pose.position.y = 0.7
	# p.pose.position.z = 0.6
	# scene.add_box("schelf",p,(1,0.3,0.05))

	p.pose.position.x = 0.55
	p.pose.position.y = 0.7
	p.pose.position.z = 0.7
	scene.add_box('Left_shelf',p,(0.1,0.3,0.1))

	p.pose.position.x = -0.55
	p.pose.position.y = 0.7
	p.pose.position.z = 0.7
	scene.add_box('right_shelf',p,(0.1,0.3,0.1))
	
#The goal position of the Arm is constructed in such a way that it is in front of the Gripper.
#The gripper now only have to make a movement forward to end up in the correct location.
def make_goal_pose(posx,posy,posz,rotz,roty,rotx):

	quaternion = tf.transformations.quaternion_from_euler(rotz, roty, rotx, 'rzyx')
	euler = tf.transformations.euler_from_quaternion(quaternion,'sxyz')
	# print euler
	R = tf.transformations.euler_matrix(euler[0],euler[1], euler[2])
	# print R
	goal_pose = manipulator.get_current_pose().pose
	# print goal_pose
	goal_pose.position.x = posx - R[0][2]*Gripper_length
	goal_pose.position.y = posy - R[1][2]*Gripper_length
	goal_pose.position.z = posz - R[2][2]*Gripper_length
	# print goal_pose
	goal_pose.orientation.x = quaternion[0]
	goal_pose.orientation.y = quaternion[1]
	goal_pose.orientation.z = quaternion[2]
	goal_pose.orientation.w = quaternion[3]
	print goal_pose

	return goal_pose

#The Home locatoin
def get_home_location():
	
	goal_pose = make_goal_pose(0.7,			#position object
								0.0,		#position object
								0.7,		#position object
								-pi/2,		#rotz
								0,			#roty
								-pi/2)			#rotx

	return goal_pose

# The location of the spoon in the container
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

#This will make the gripper move in the direction of the gripper such that it can 
#pick up the object
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
	home = Item('home')
	mug_01 = Item("mug_01")
	mug_02 = Item("mug_02")
	mug_03 = Item("mug_03")
	placemat_01 = Item("placemat_01")
	placemat_02 = Item("placemat_02")
	placemat_03 = Item("placemat_03")


	

	# disable_upright_path_constraints()
	# home.go_to_location()	;	print "go_to_location home"
	home.go_to_location() 
	mug_01.go_to_location()
	placemat_01.go_to_location()
	mug_02.go_to_location()
	placemat_02.go_to_location()
	mug_03.go_to_location()
	placemat_03.go_to_location()
	# mug_03.go_to_location()
	# placemat_03.go_to_location()
	# home.go_to_location()

