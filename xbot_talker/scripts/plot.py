#!/usr/bin/env python
#coding=utf-8

import numpy as np
import matplotlib.pyplot as plt

def draw():
	data1 = np.loadtxt('/home/wxyzd/axbot_ws/src/xbot_talker/cache/test1.txt')
	print(type(data1))
	print(data1)
	x = np.linspace(0,825,826)
	plt.plot(x, data1, c='r', ls='--',marker='o')
	plt.xlabel('number')
	plt.ylabel('u')
	plt.title('LM')
	plt.show()


if __name__ == '__main__':
	draw()

