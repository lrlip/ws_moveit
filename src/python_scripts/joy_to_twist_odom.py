#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry

t = None
linear_scale = 0.5
angular_scale = 0.8

def joy_cb(data, pub):
	global t
	t = Twist()
	t.linear.x = linear_scale*data.axes[1]
	t.linear.y = linear_scale*data.axes[0]
	t.angular.z = angular_scale*data.axes[3]
	t.linear.z = data.axes[2]
	pub.publish(t)	

if __name__ == '__main__':
	# starts the node
	rospy.init_node("joy_to_twist")
	# publishing to "joy_to_twist" to control
	twist_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

	odom_pub = rospy.Publisher("odom", Odometry, queue_size = 1)

	# subscribed to joystick inputs on topic "joy"
	rospy.Subscriber("joy", Joy, joy_cb, twist_pub)

	# publishing to odomatry
	odom_pub

	rate = rospy.Rate(50)
	t = Twist()
	while not rospy.is_shutdown():
		twist_pub.publish(t)
		rate.sleep()
