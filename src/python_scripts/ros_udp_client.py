#!/usr/bin/env python
import rospy
import socket
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

UDP_IP = "192.168.1.3"
UDP_PORT = 8080

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT

def callback(msg):
	global pose
	pose = msg
	Message = "%f %f %f %f %f %f"%(pose.linear.x, pose.linear.y,
			pose.linear.z, pose.angular.x,
			pose.angular.y, pose.angular.z)
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	sock.sendto(Message, (UDP_IP, UDP_PORT))
	
if __name__ == '__main__':
	rospy.init_node("ros_udp_client")
	pub = rospy.Publisher("test_vel", Twist, queue_size=1)
	rospy.Subscriber("cmd_vel", Twist, callback)
	rate = rospy.Rate(50)
	twist = Twist()

	while not rospy.is_shutdown():
		pub.publish(twist)
		rate.sleep()
