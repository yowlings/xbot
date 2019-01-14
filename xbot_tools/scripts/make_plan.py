#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Author: yowlings@gmail.com
# @Date:   2018-12-26 10:22:42
# @Last Modified by:   yowlings@gmail.com
# @Last Modified time: 2019-01-03 15:55:45
import rospy
from nav_msgs.msg import Path


def callback(data):

	# print len(data.poses)
	# print data
	rospy.loginfo(len(data.poses))

def listener():

	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('/qbot1/move_base/NavfnROS/plan', Path, callback)
	rospy.spin()


if __name__ == '__main__':
	listener()