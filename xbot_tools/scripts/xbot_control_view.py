#!/usr/bin/env python
# coding=utf-8
######################################################################################
#> File Name: xbot_control_view.py
#> Author:Rocwang 
#> Mail: wangpeng@droid.ac.cn;
#> Github:https://github.com/yowlings
#> Created Time: 2018年10月17日 星期三 16时09分49秒
######################################################################################
from PyQt5 import QtCore, QtGui, QtWidgets
from xbot_ui import Ui_Dialog
import rospy, sys, termios, tty

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from xbot_msgs.msg import ExtraSensor, CoreSensor

class XbotGUI():
	"""docstring for XbotGUI"""
	def __init__(self):
		self.app = QtWidgets.QApplication(sys.argv)
		self.Dialog = QtWidgets.QDialog()
		self.ui = Ui_Dialog()
		self.ui.setupUi(self.Dialog)
		self.Dialog.show()
		self.init_ros()
		sys.exit(self.app.exec_())
		rospy.spin()
		

	def init_ros(self):
		self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odomCB)
		self.core_sensor_sub = rospy.Subscriber("/mobile_base/sensors/core", CoreSensor, self.core_sensorCB)
		self.extra_sensor_sub = rospy.Subscriber("/mobile_base/sensors/extra", ExtraSensor, self.extra_sensorCB)


	def odomCB(self,msg):
		self.ui.l_linear_x.setText(str(msg.twist.twist.linear.x))
		self.ui.l_angular_z.setText(str(msg.twist.twist.angular.z))
		


	def core_sensorCB(self,msg):
		self.ui.l_left_encoder.setText(str(msg.left_encoder))
		self.ui.l_right_encoder.setText(str(msg.right_encoder))
		self.ui.l_power.setText(str(msg.battery_percent))
		if msg.stop_button_clicked == True:
			state = "开关被按下"
		else:
			state = "开关未按下"
		self.ui.l_switch_state.setText(state)
		
	def extra_sensorCB(self,msg):
		self.ui.l_yaw_degree.setText(str(msg.yaw_platform_degree))
		self.ui.l_pitch_degree.setText(str(msg.pitch_platform_degree))
		if msg.sound_is_mutex == True:
			state = "静音"
		else:
			state = "外放"
		self.ui.l_mute_state.setText(state)

		self.ui.l_yaw.setText(str(msg.yaw))
		self.ui.l_pitch.setText(str(msg.pitch))
		self.ui.l_roll.setText(str(msg.roll))
		self.ui.l_mag_x.setText(str(msg.mag_x))
		self.ui.l_mag_y.setText(str(msg.mag_y))
		self.ui.l_mag_z.setText(str(msg.mag_z))
		self.ui.l_acc_x.setText(str(msg.acc_x))
		self.ui.l_acc_y.setText(str(msg.acc_y))
		self.ui.l_acc_z.setText(str(msg.acc_z))
		self.ui.l_w_x.setText(str(msg.gyro_x))
		self.ui.l_w_y.setText(str(msg.gyro_y))
		self.ui.l_w_z.setText(str(msg.gyro_z))



if __name__ == "__main__":
	rospy.init_node('xbot_control_view')
	try:
		rospy.loginfo( "initialization system")
		XbotGUI()
		print "process done and quit"
	except rospy.ROSInterruptException:
		rospy.loginfo("node terminated.")

    
