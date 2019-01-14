# -*- coding: utf-8 -*-
# @Author: yowlings@gmail.com
# @Date:   2019-01-07 17:58:48
# @Last Modified by:   yowlings@gmail.com
# @Last Modified time: 2019-01-07 17:59:00
#!/usr/bin/env python

from beginner_tutorials.srv import *
import rospy

def handle_add_two_ints(req):
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()