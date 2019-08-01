#!/usr/bin/env python

#author: Lars Lipman

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
    # home_pose.orientation.x = 0.923
    # home_pose.orientation.y = -0.38
    # home_pose.orientation.z = 0
    # home_pose.orientation.w =

    return home_pose

# Contains the pickup location of the tool (PERCEPTION)
def get_pick_location():

    pick_pose = manipulator.get_current_pose().pose

    pick_pose.position.x = 0.4
    pick_pose.position.y = 0.3
    pick_pose.position.z = 0.3
    # pick_pose.orientation.x = 0.923
    # pick_pose.orientation.y = -0.38
    # pick_pose.orientation.z = 0
    # pick_pose.orientation.w = 0

    return pick_pose

#Contains the place location of the tool (Perception)
def get_place_location():

    place_pose = manipulator.get_current_pose().pose

    place_pose.position.x = 0.3
    place_pose.position.y = -0.3
    place_pose.position.z = 0.3
    # place_pose.orientation.x = 0.923
    # place_pose.orientation.y = -0.38
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


if __name__ == '__main__':

    # intialize moveit commander and rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pick_and_place', anonymous = True)

    #this object is the outer leverl interface to the robot
    robot = moveit_commander.RobotCommander()
    #this object is the interface to the world surrounding the robot
    scene = moveit_commander.PlanningSceneInterface()

    # print "The Groupnames are", join(robot.get_group_names())
    arm_name            = "panda_arm"    #give groupname you want to shoe
    endeffector_name    = "hand"
    manipulator = moveit_commander.MoveGroupCommander(arm_name)
    endeffector = moveit_commander.MoveGroupCommander(endeffector_name)

    #Is used to display the trajectoryies in RVIZ
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory, queue_size = 20)

    ############### START OF PROGRAM #################

    #Getting the required positions
    home_pose  = get_home_location()
    pick_pose   = get_pick_location()
    place_pose  = get_place_location()

    #Moving the sequence
    go_to_point(home_pose)
    go_to_point(pick_pose)
    pick_up_object()
    go_to_point(place_pose)
    place_object()
    go_to_point(home_pose)
