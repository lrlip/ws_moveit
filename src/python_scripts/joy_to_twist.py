#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

t = None

def joy_cb(data, pub):
	global t
	t = Twist()
	t.linear.x = 0.5*data.axes[1]
	t.linear.y = 0.5*data.axes[0]
	t.angular.z = 0.8*data.axes[3]
	t.linear.z = data.axes[2]
	pub.publish(t)	

if __name__ == '__main__':
	# starts the node
	rospy.init_node("joy_to_twist")
	# publishing to "joy_to_twist" to control
	pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
	# subscribed to joystick inputs on topic "joy"
	rospy.Subscriber("joy", Joy, joy_cb, pub)
	rate = rospy.Rate(50)
	t = Twist()
	while not rospy.is_shutdown():
		pub.publish(t)
		rate.sleep()
