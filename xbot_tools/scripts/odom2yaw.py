#!/usr/bin/env python
# coding=utf-8
######################################################################################
#> File Name: auto_rotate.py
#> Author:Rocwang 
#> Mail: wangpeng@droid.ac.cn;
#> Github:https://github.com/yowlings
#> Created Time: 2018年06月14日 星期四 16时14分27秒
######################################################################################


import rospy
import tf
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


alfa=0.1
beta=0.31415926/5
class Odom2Yaw(object):
	"""docstring for Odom2Yaw"""
	def __init__(self):
		self.help="""

		"""
		# if rospy.has_param("~cascPath"):
		# 	self.cascPath = rospy.get_param("~cascPath")
		# else:
		# 	rospy.set_param("~cascPath","../scripts/haarcascade_frontalface_default.xml")
		self.yaw_pub=rospy.Publisher('/yaw', Float64, queue_size = 1)
		rospy.Subscriber("/odom",Odometry,self.odom_callback)
		rospy.spin()




	def odom_callback(self,odom):
		yaw = Float64()
		orie = odom.pose.pose.orientation
		o=[orie.x,orie.y,orie.z,orie.w]

		rpy= tf.transformations.euler_from_quaternion(o)
		yaw.data = rpy[2]*180/3.1415926
		self.yaw_pub.publish(yaw)

if __name__=='__main__':
	rospy.init_node('odom2yaw')
	try:
		rospy.loginfo( "initialization system")
		Odom2Yaw()
		print "process done and quit"
	except rospy.ROSInterruptException:
		rospy.loginfo("node terminated.")