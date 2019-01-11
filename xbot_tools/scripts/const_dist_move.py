#!/usr/bin/env python
#coding=utf-8

"""
This program makes xbot automnomous get the target point in map. The main funcution is testing the base movements of xbot such as moving forward and rotating.
Copyright (c) 2016 Peng Wang (Rocwang).  All rights reserved.
This program is free software; you can redistribute or modify it.
More details about xbot robot platform is available in http://wiki.ros.org/Robots/Xbot.
"""

import rospy, sys, termios, tty, math, time, cv2, numpy, PyKDL
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped,Point
from geometry_msgs.msg import Quaternion


alfa=0.1
beta=0.31415926/5
class LoopControl(object):
	"""docstring for LoopControl"""
	def __init__(self):
		self.help="""

		"""
		# if rospy.has_param("~cascPath"):
		# 	self.cascPath = rospy.get_param("~cascPath")
		# else:
		# 	rospy.set_param("~cascPath","../scripts/haarcascade_frontalface_default.xml")
		self.control_pub=rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 1)
		rospy.Subscriber("odom",Odometry,self.odom_callback)
		self.target=PointStamped()
		rospy.spin()




	def odom_callback(self,odom):
		cmd=Twist()
		dx=0-odom.pose.pose.position.x
		dy=0-odom.pose.pose.position.y
		cmd.linear.x=alfa*dx
		cmd.angular.z=0#beta*dy
		self.control_pub.publish(cmd)








if __name__ == '__main__':
	rospy.init_node('loop_control')
	try:
		rospy.loginfo('initialization system for loop control...')
		LoopControl()
		print 'process loop control done and quit.'
	except rospy.ROSInterruptException:
		rospy.loginfo('node loop_control termindated.')





