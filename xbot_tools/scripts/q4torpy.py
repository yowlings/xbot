# -*- coding: utf-8 -*-
# @Author: wangpeng@droid.ac.cn
# @Date:   2018-12-29 10:38:52
# @Last Modified by:   wangpeng@droid.ac.cn
# @Last Modified time: 2018-12-29 11:33:46
import rospy
import tf
# test1
orientation1=[0,0,0.9818,0.1899]
orientation2=[0,0,0.991,0.1335]
rpy1= tf.transformations.euler_from_quaternion(orientation1)
rpy2=tf.transformations.euler_from_quaternion(orientation2)


dy1=abs(rpy1[2]-rpy2[2])*180/3.1415926

#test2
orientation1=[0,0,-0.760734,0.64906]
orientation2=[0,0,-0.7435,0.668745]
rpy1= tf.transformations.euler_from_quaternion(orientation1)
rpy2=tf.transformations.euler_from_quaternion(orientation2)


dy2=abs(rpy1[2]-rpy2[2])*180/3.1415926

print dy1
print dy2
print '-----------------------------------'
print (dy2+dy1)/2
