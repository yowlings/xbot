#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseActionGoal
import math



if __name__ == '__main__':
		rospy.init_node('pub_goal_set')
		pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size = 1)
		goals=MoveBaseActionGoal()
		goals.header.frame_id='map'
		goals.goal_id.id='wd'
		goals.goal.target_pose.header.frame_id='map'
		goals.goal.target_pose.pose.position.x=0.552559435368
		goals.goal.target_pose.pose.position.y=-1.06640863419
		goals.goal.target_pose.pose.position.z=0.0
		goals.goal.target_pose.pose.orientation.x=0.0
		goals.goal.target_pose.pose.orientation.y=0.0
		goals.goal.target_pose.pose.orientation.z=-0.682496084898
		goals.goal.target_pose.pose.orientation.w=0.730889248859
		rate=rospy.Rate(0.1)
		while not rospy.is_shutdown():
			
			pub.publish(goals)
			rate.sleep()
