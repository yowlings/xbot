#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8
import math





if __name__ == '__main__':
	rospy.init_node('check_topic')
	pub = rospy.Publisher("/mobile_base/commands/led", UInt8, queue_size = 1)
	index = 0
	r=rospy.Rate(2)
	cmd = UInt8()
	while not rospy.is_shutdown():
		cmd.data = math.pow(2,index)-1
		pub.publish(cmd)
		index =index+1
		if index > 4:
			index = 0
		r.sleep()
	rospy.spin()