#!/usr/bin/env python

#Joy_to_twist.py is started by Madhukar

#Changelog Lars Lipman
# 31-7-19: Odom is added in the file joy_to_twist_odom.py
# 01-08-19: Deadman_switch is added in joy_to_twist_odom.py and removed from client.py an server.c



import rospy
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from math import pi, cos, sin
import tf.transformations
import tf
import time

t = None
odom_msg = None
x_pos = 0.0
y_pos = 0.0
theta = 0.0
linear_scale = 0.6
angular_scale = 0.8
publish_tf = True      # Must the TF been published
dead_man_switch_pressed = False


def joy_cb(data, pub):
    global t
    global dead_man_switch_pressed
    t = Twist()
    t.linear.x = linear_scale*data.axes[1]          # left-rigth left joystick                  GOING FORWARD
    t.linear.y = linear_scale*data.axes[0]          # Up down left joystick                     GOING SIDEWAYS
    t.angular.z = angular_scale*data.axes[3]        # left - right right joystick (logitech)
    t.linear.z = data.axes[2]                       # LT button (logitech) Deadmanswitch
    twist_pub.publish(t) 
    if data.axes[2] <= 0.0:
        dead_man_switch_pressed = True
    else:
        dead_man_switch_pressed = False

if __name__ == '__main__':
    # starts the node
    rospy.init_node("joy_to_twist", anonymous = True)
    # publishing to "joy_to_twist" to control
    twist_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    # publishing odometry
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)

    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, joy_cb, twist_pub)

    current_time = rospy.get_time()
    speed_time = rospy.Time(0.0)

    # publishing to odomatry
    Hz = 50.0   # Frequency
    dt = 1/Hz   # Periode between each senc
    dx = 0.0    # initial dx/dt
    dy = 0.0    # initial dy/dt
    dth = 0.0   # initial dth/dt
    # vx = 0.0
    # vy = 0.0
    # vth = 0.0

    rate = rospy.Rate(Hz)
    t = Twist()
    odom_msg = Odometry()
    while not rospy.is_shutdown():
        # print dead_man_switch_pressed
        # if LT is pressed more than halfway
        # When switch is pressed
        if dead_man_switch_pressed:
            dx = t.linear.x * dt
            dy = t.linear.y * dt
            dth = t.angular.z*dt
            print dx, dy, dth
            x_pos += (cos(theta) * dx - sin(theta) * dy)  # New xy coordinates
            y_pos += (sin(theta) * dx + cos(theta) * dy)
            theta += dth;

            if (theta >= 2*pi):
                theta -= 2*pi
            if (theta <= -2*pi):
                theta += 2*pi

            odom_quat = tf.transformations.quaternion_from_euler(0,0,theta)
            empty_quat = tf.transformations.quaternion_from_euler(0,0,0)

            if publish_tf:
                # tf_msg = TransformStamped()
                # tf_msg.header.frame_id = "/odom"
                # tf_msg.child_frame_id = "/base_link"
                # tf_msg.transform.translation.x = x_pos
                # tf_msg.transform.translation.y = y_pos
                # tf_msg.transform.translation.z = 0.0
                # tf_msg.transform.rotation = odom_quat 
                # tf_msg.header.stamp = current_time

                broadcaster = tf.TransformBroadcaster()
                broadcaster.sendTransform((x_pos, y_pos, 0.0),
                                                  odom_quat, 
                                                  rospy.Time.now(),
                                                  "/odom",
                                                  "/base_link")

            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = "/odom"
            odom_msg.pose.pose.position.x = x_pos
            odom_msg.pose.pose.position.y = y_pos
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation.x = odom_quat[0]
            odom_msg.pose.pose.orientation.y = odom_quat[1]
            odom_msg.pose.pose.orientation.z = odom_quat[2]
            odom_msg.pose.pose.orientation.w = odom_quat[3]

            # If there is no movement
            if (dx == 0 and dy == 0):
                odom_msg.pose.covariance[0] = 1e-9
                odom_msg.pose.covariance[7] = 1e-3
                odom_msg.pose.covariance[8] = 1e-9
                odom_msg.pose.covariance[14] = 1e6
                odom_msg.pose.covariance[21] = 1e6
                odom_msg.pose.covariance[28] = 1e6
                odom_msg.pose.covariance[35] = 1e-9
                odom_msg.twist.covariance[0] = 1e-9
                odom_msg.twist.covariance[7] = 1e-3
                odom_msg.twist.covariance[8] = 1e-9
                odom_msg.twist.covariance[14] = 1e6
                odom_msg.twist.covariance[21] = 1e6
                odom_msg.twist.covariance[28] = 1e6
                odom_msg.twist.covariance[35] = 1e-9
            else:
                odom_msg.pose.covariance[0] = 1e-3
                odom_msg.pose.covariance[7] = 1e-3
                odom_msg.pose.covariance[8] = 0.0   
                odom_msg.pose.covariance[14] = 1e6
                odom_msg.pose.covariance[21] = 1e6
                odom_msg.pose.covariance[28] = 1e6
                odom_msg.pose.covariance[35] = 1e3
                odom_msg.twist.covariance[0] = 1e-3
                odom_msg.twist.covariance[7] = 1e-3
                odom_msg.twist.covariance[8] = 0.0
                odom_msg.twist.covariance[14] = 1e6
                odom_msg.twist.covariance[21] = 1e6
                odom_msg.twist.covariance[28] = 1e6
                odom_msg.twist.covariance[35] = 1e3
            odom_msg.child_frame_id = "/base_link"
            odom_msg.twist.twist.linear.x = dx
            odom_msg.twist.twist.linear.y = dy
            odom_msg.twist.twist.angular.z = dth

            odom_pub.publish(odom_msg)
            twist_pub.publish(t)

        else:
            dx = 0.0
            dy = 0.0
            dth = 0.0
            print dx, dy, dth

            odom_quat = tf.transformations.quaternion_from_euler(0,0,theta)
            empty_quat = tf.transformations.quaternion_from_euler(0,0,0)

            if publish_tf:
                broadcaster = tf.TransformBroadcaster()
                broadcaster.sendTransform((x_pos, y_pos, 0.0),
                                                  odom_quat, 
                                                  rospy.Time.now(),
                                                  "/odom",
                                                  "/base_link")

            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = "/odom"
            odom_msg.pose.pose.position.x = x_pos
            odom_msg.pose.pose.position.y = y_pos
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation.x = odom_quat[0]
            odom_msg.pose.pose.orientation.y = odom_quat[1]
            odom_msg.pose.pose.orientation.z = odom_quat[2]
            odom_msg.pose.pose.orientation.w = odom_quat[3]

            odom_msg.pose.covariance[0] = 1e-9
            odom_msg.pose.covariance[7] = 1e-3
            odom_msg.pose.covariance[8] = 1e-9
            odom_msg.pose.covariance[14] = 1e6
            odom_msg.pose.covariance[21] = 1e6
            odom_msg.pose.covariance[28] = 1e6
            odom_msg.pose.covariance[35] = 1e-9
            odom_msg.twist.covariance[0] = 1e-9
            odom_msg.twist.covariance[7] = 1e-3
            odom_msg.twist.covariance[8] = 1e-9
            odom_msg.twist.covariance[14] = 1e6
            odom_msg.twist.covariance[21] = 1e6
            odom_msg.twist.covariance[28] = 1e6
            odom_msg.twist.covariance[35] = 1e-9

            odom_msg.child_frame_id = "/base_link"
            odom_msg.twist.twist.linear.x = dx
            odom_msg.twist.twist.linear.y = dy
            odom_msg.twist.twist.angular.z = dth

            odom_pub.publish(odom_msg)
            twist_pub.publish(t)



        rate.sleep()
