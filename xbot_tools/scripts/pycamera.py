#!/usr/bin/env python
# coding=utf-8
######################################################################################
#> File Name: pycamera.py
#> Author:Rocwang 
#> Mail: yowlings@gmail.com;
#> Github:https://github.com/yowlings
#> Created Time: 2018年07月05日 星期四 11时40分33秒
######################################################################################
# created by Huang Lu
# 27/08/2016 17:05:45 
# Department of EE, Tsinghua Univ.

import cv2
import numpy as np


cv2.namedWindow("capture")
cap = cv2.VideoCapture(1)
while(1):
  # get a frame
	ret, frame = cap.read()
	# show a frame
	# print len(frame[0])
	cv2.imshow("capture", frame)
	# cv2.imwrite("1b.jpg",frame)
	if cv2.waitKey(1) & 0xFF == ord('q'):
	  break
cap.release()
cv2.destroyAllWindows() 
