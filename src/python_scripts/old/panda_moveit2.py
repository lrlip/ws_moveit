#!/usr/bin/env python

import sys
import copy
import rospy
import time
import random
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
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

def start_moving(waypoints):

    (plan_cart, fraction) = manipulator.compute_cartesian_path(
                                            waypoints,
                                            0.01,       # eef_step
                                            0.0)        # jump threshold
    manipulator.execute(plan_cart, wait=True)
    manipulator.stop()
    manipulator.clear_pose_targets()

    print "Start_moving: Position is correct now"

def smooth_path_traject(cur_pose, des_pose):
    #TODO: Check if cur_pose is not equal to des_pose

    waypoints = []


    A_3 = np.array([cur_pose.position.x, cur_pose.position.y, cur_pose.position.z]) #3d vector
    B_3 = np.array([des_pose.position.x, des_pose.position.y, des_pose.position.z])          #3d vector desired point
    
    if (A_3[2] and B_3[2]) > 0.55:
        collision_radius = 0.3                     # Collision radius of the kuka
    else:
        collision_radius = 0.4


    A_2 = np.array([A_3[0], A_3[1],0])          # point A - Start vector (topview)
    B_2 = np.array([B_3[0], B_3[1],0])          # point B - End vector (topview)
    C_2 = np.array([0,0,0])                     # point C - Center point of the kuka


    #directin vectors
    vector_AC = A_2 - C_2                       # Vector AC
    vector_AB = B_2 - A_2                       # Vector AB

    length_AB = np.linalg.norm(vector_AB)                       #|AB|
    length_AC = np.linalg.norm(vector_AC)                       #|AC|
    cross_AC = np.linalg.norm(np.cross(vector_AB,vector_AC))    #|AC X AB|, crossproduct
    # D is a point on line AB closest to point C
    length_CD = cross_AC/length_AB                              #|AC X AB| / |AB|
    
    print "vectAB", vector_AB
    print "vectAC", vector_AC
    print "puntA", A_2
    print "puntB", B_2
    print "puntC", C_2
    print "distAB", length_AB
    print "distAC", length_AC
    print "crossAC", cross_AC
    print "distCD", length_CD


    if length_CD > collision_radius:
        waypoints = add_waypoint(waypoints, B_3)
        print "SmoothPathTrajectory: If statement: ", waypoints

    else:
        length_AD = math.sqrt(length_AC**2 - length_CD**2)          #|AD|
        length_BD = length_AB - length_AD

        D_2 = vector_AB / length_AB * length_AD + A_2                    # point D
        vector_CD = D_2 - C_2
        vector_CE = vector_CD / length_CD * collision_radius        #length CE is collision_radius
        length_DE = collision_radius-length_CD

        length_AE = math.sqrt(length_AD**2 + length_DE**2)
        length_EB = math.sqrt(length_BD**2 + length_DE**2)
        length_AEB = length_AE+length_EB
        E_2 = vector_CE - C_2                                       #Location of Point E (in case Centerpoint changes)
        #For smooth trajectory, elevation in z-direction is calculated in a smooth way
        z_difference = B_3[2]-A_3[2]
        E_3 = np.array([E_2[0], E_2[1], length_AE/length_AEB*z_difference+A_3[2]])
        
        waypoints = add_waypoint(waypoints, E_3)
        waypoints = add_waypoint(waypoints, B_3)

        print "distAD", length_AD
        print "distBD", length_BD
        print "puntD", D_2
        print "vectCD", vector_CD
        print "vectCE", vector_CE
        print "distDE", length_DE
        print "puntE", E_2
        print "puntE3d", E_3




    return waypoints

def go_to_start():
    waypoints = []
    cur_pose = manipulator.get_current_pose().pose

    start_pose = copy.deepcopy(cur_pose)
    start_pose.position.x = 0.5
    start_pose.position.y = 0.0
    start_pose.position.z = 0.6
    start_pose.orientation.x = 0.923
    start_pose.orientation.y = -0.38
    start_pose.orientation.z = 0
    start_pose.orientation.w = 0

    waypoints = smooth_path_traject(cur_pose, start_pose)

    start_moving(waypoints)


if __name__ == '__main__':

    # intialize moveit commander and rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python', anonymous = True)

    #this object is the outer leverl interface to the robot
    robot = moveit_commander.RobotCommander()
    #this object is the interface to the world surrounding the robot
    scene = moveit_commander.PlanningSceneInterface()

    print "The Groupnames are", robot.get_group_names()
    arm_name = "manipulator"    #give groupname you want to shoe
    # endeffector_name = "hand"
    manipulator = moveit_commander.MoveGroupCommander(arm_name)
    # endeffector = moveit_commander.MoveGroupCommander(endeffector_name)

    #Is used to display the trajectoryies in RVIZ
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory, queue_size = 20)

    go_to_start()
    time.sleep(1)
    waypoints = []

    cur_pose = manipulator.get_current_pose().pose
    des_pose = copy.deepcopy(cur_pose)

    des_pose.position.x = -0.5
    des_pose.position.y = 0.0
    des_pose.position.z = 0.3
    des_pose.orientation.x = 0.923
    des_pose.orientation.y = -0.38
    des_pose.orientation.z = 0
    des_pose.orientation.w = 0

    waypoints = smooth_path_traject(cur_pose, des_pose)
    start_moving(waypoints)
    # go_to_pose_cart()
    print manipulator.get_current_pose().pose



moveit_commander.roscpp_shutdown()
