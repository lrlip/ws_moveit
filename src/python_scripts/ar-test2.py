#! /usr/bin/env python

from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped
from math import pi
import numpy
# import fetch_api
import sys, copy, rospy, time, tf, math, moveit_commander, actionlib
import moveit_msgs.msg
from std_srvs.srv import Empty
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)
from tf import TransformListener
from quaternion import Quaternion




obj_mark = {}
obj_mark["mug_01"] = ['ar_marker_1'] #, 11]
obj_mark["mug_02"] = ['ar_marker_2'] #, 12]
obj_mark["mug_03"] = ['ar_marker_3'] #, 13]
obj_mark["placemat_01"] = ['ar_marker_5']
obj_mark["placemat_02"] = ['ar_marker_6']
obj_mark["placemat_03"] = ['ar_marker_7']


Gripper_length = 0.13 #Define global variable



def wait_for_time():
	"""Wait for simulated time to begin.
	"""
	while rospy.Time().now().to_sec() == 0:
		pass

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
	# print goal_pose

	return goal_pose

class ArTagReader(object):
	def __init__(self):
		self.markers = []

	def callback(self, msg):
		self.markers = msg.markers


class Item(object):

	def __init__(self, name):
		self.name = name

	def get_location(self):
		name = self.name

		reader = ArTagReader()
		quat = Quaternion()
		self.tf_listener = TransformListener()
		rate = rospy.Rate(10.0)

		goal_ar_tag = ''.join(obj_mark[name])
		self.tf_listener.waitForTransform('/world','/camera_link',rospy.Time(), rospy.Duration(4.0))
		camera_pose, camera_rot = self.tf_listener.lookupTransform('/world', '/camera_link', rospy.Time(0))
		self.tf_listener.waitForTransform('/world',goal_ar_tag,rospy.Time(), rospy.Duration(4,0))
		ar_tag_pose, ar_tag_rot = self.tf_listener.lookupTransform('/world', goal_ar_tag, rospy.Time(0))

		print
		print "camera_pose = ", camera_pose
		print "ar_tag_pose = ", ar_tag_pose

		camera_pose_2 = quat.qv_mult(camera_rot, camera_pose)
		print 
		print "camera_pose_2 = ", camera_pose_2
		
		# sub_alvar = rospy.Subscriber(
		# 	'ar_pose_marker', AlvarMarkers, callback=reader.callback)

		# # wait till the markers become active: IMPORTANT
		# while len(reader.markers) == 0:
		# 	rospy.sleep(0.1)

		# #Check all markers to see which corresponds to the location needed
		# for marker in reader.markers:                      
		# 	if [marker.id] == obj_mark[name]:
		# 		print "marker_id = ", marker.id

		# 		marker_camera_pose = marker.pose
		# 		print
		# 		print 'marker_camera_pose = ', marker_camera_pose

		# 		break

		#marker_camera_pose had the position and orientation of the marker wrt the camera
		marker_pose = manipulator.get_current_pose().pose

		marker_pose.position.x = ar_tag_pose[0]
		marker_pose.position.y = ar_tag_pose[1]
		marker_pose.position.z = ar_tag_pose[2]

		print
		print 'marker_pose = ', marker_pose
		print

		# check orientation of the gripper for the mugges
		if abs(marker_pose.position.x) >= abs(marker_pose.position.y):
			if marker_pose.position.x >= 0:
				print 'object in front'
				goal_pose = make_goal_pose(marker_pose.position.x ,
											marker_pose.position.y,
											marker_pose.position.z,
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
											marker_pose.position.z,
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

		return goal_pose



if __name__ == '__main__':
	rospy.init_node('arm_demo', )
	wait_for_time()


	moveit_commander.roscpp_initialize(sys.argv)

	rospy.wait_for_service("/clear_octomap")
	clear_octomap = rospy.ServiceProxy("/clear_octomap", Empty)

	gripper = actionlib.SimpleActionClient("iiwa/gripper_controller/gripper_cmd", GripperCommandAction)
	gripper.wait_for_server()

	manipulator_name    = "manipulator"    #give groupname you want to shoe
	endeffector_name    = "gripper"
	robot       = moveit_commander.RobotCommander()
	manipulator = moveit_commander.MoveGroupCommander(manipulator_name)
	endeffector = moveit_commander.MoveGroupCommander(endeffector_name)
	# manipulator.allow_replanning(True)
	tf_listener = tf.TransformListener()




	mug_01 = Item("mug_02")

	print mug_01.get_location()




