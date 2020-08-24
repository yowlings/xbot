#!/usr/bin/env python
#coding=utf-8
import os
import cv2
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

def getCameraId():
  cmd = "ls /dev/video* > vs.out"
  os.system(cmd)
  vf = open('vs.out','r')
  vs = vf.readlines()
  vf.close()
  vn = len(vs)
  # print vs
  for dev in vs:
    vd = dev[:-1]
    cmd = "udevadm info "+vd+" |grep 'ID_VENDOR_ID=0bda' > vs.info"
    os.system(cmd)
    infof = open("vs.info",'r')
    info = infof.readlines()
    infof.close()
    # print info
    if len(info)!=0:
      return int(vd[-1])
      
  return -1




if __name__ == '__main__':
  rospy.init_node('camera_image')
  pub = rospy.Publisher('/xbot/camera/image', Image, queue_size=5)
  index = getCameraId()
  if index == -1:
    rospy.loginfo("Not find face camera")
  else:
    capture = cv2.VideoCapture(index)
    cv_bridge = CvBridge()

    while not rospy.is_shutdown():
        ret, frame = capture.read()
        frame = cv2.flip(frame,1)   #镜像操作
        msg = cv_bridge.cv2_to_imgmsg(frame,"bgr8")
        pub.publish(msg)

