#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

twist_pub = None
target_twist = None
last_twist = None
last_twist_send_time = None
g_vel_scales = [0.5,0.5,0.8]
g_vel_ramps = [1,1]

def ramped_vel(v_prev, v_target, t_prev, t_now, ramp_rate):
	step = ramp_rate * (t_now - t_prev).to_sec()
	sign = 1.0 if (v_target > v_prev) else -1.0
	error = math.fabs(v_target - v_prev)
	if error < step: 
		return v_target
	else:
		return v_prev + sign * step

def ramped_twist(prev, target, t_prev, t_now, ramps):
	tw = Twist()
	tw.angular.z = ramped_vel(prev.angular.z, target.angular.z, t_prev,
	t_now, ramps[0])
	tw.linear.x = ramped_vel(prev.linear.x, target.linear.x, t_prev,
	t_now, ramps[1])
	tw.linear.y = ramped_vel(prev.linear.y, target.linear.y, t_prev,
	t_now, ramps[1])	
	return tw

def send_twist():
	global last_twist_send_time, target_twist, last_twist, g_vel_scales, g_vel_ramps, twist_pub
	t_now = rospy.Time.now()
	last_twist = ramped_twist(last_twist, target_twist, last_twist_send_time, t_now, g_vel_ramps)
	last_twist_send_time = t_now
	twist_pub.publish(last_twist)

def joy_cb(data, twist_pub):
	global target_twist, last_twist, g_vel_scales
	target_twist.linear.x = g_vel_scales[0]*data.axes[1]
	target_twist.linear.y = g_vel_scales[0]*data.axes[0]
	target_twist.angular.z = g_vel_scales[1]*data.axes[3]
	target_twist.linear.z = g_vel_scales[2]*data.axes[2]
	twist_pub.publish(target_twist)

if __name__ == '__main__':
	rospy.init_node("joy_to_twist")
	last_twist_send_time = rospy.Time.now()
	twist_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
	rospy.Subscriber("joy", Joy, joy_cb, twist_pub)
	target_twist = Twist()
	last_twist = Twist()
	
	rate = rospy.Rate(50)
	while not rospy.is_shutdown():
		send_twist()
		rate.sleep()
