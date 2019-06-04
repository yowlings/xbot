#!/usr/bin/env python
# coding=utf-8
######################################################################################
#> File Name: auto_rotate.py
#> Author:Rocwang 
#> Mail: wangpeng@droid.ac.cn;
#> Github:https://github.com/yowlings
#> Created Time: 2018年06月14日 星期四 16时14分27秒
######################################################################################

import rospy, sys, termios, tty
import time
from geometry_msgs.msg import Twist


class multi_keybroad_handle():
	def __init__(self):
		self.pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 1)
		self.old_settings = termios.tcgetattr(sys.stdin)
		r=rospy.Rate(20)
		try:
			while not rospy.is_shutdown():
				self.cmd = Twist()
				self.cmd.linear.x = 0.5
				# self.cmd.angular.z = 3.1415926*30/180
				self.pub.publish(self.cmd)
				r.sleep()

		except :
			print 'error'
		finally:
			self.pub.publish(self.cmd)





if __name__=='__main__':
	rospy.init_node('xbot_auto_control')
	try:
		rospy.loginfo( "initialization system")
		multi_keybroad_handle()
		print "process done and quit"
	except rospy.ROSInterruptException:
		rospy.loginfo("node terminated.")
		kkk

