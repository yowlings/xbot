#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Author: roc
# @Date:   2019-01-17 11:59:04
# @Last Modified by:   yowlings@gmail.com
# @Last Modified time: 2019-01-18 21:30:06
###########################################################
#
#该软件实现了以下功能：
#１、判断测试机是否连接了串口设备，并输出串口设备清单；
#２、如果连接了串口设备，确定串口设备与xbot(PCB1),sensor(PCB2),rplidar的对应关系
#３、输出从串口设备读取的一段完整的串口数据帧
#４、按照串口协议解析出数据帧中对应码段的真实数据
#５、将串口映射表文件写入系统的/etc/udev/rules.d/目录下
#
###########################################################


import serial.tools.list_ports
import string,os,commands,struct
import serial
import string
import binascii
import time
from time import sleep



rplidar_pluged = False

xbot_pluged = False
label_xbot = 'aa552d'
ss_xbot = ''

sensor_pluged=False
label_sensor = 'aa5547'
ss_sensor = ''

qbot_pluged = False
label_qbot = 'aa5518'
ss_qbot=''

pcb6_pluged = False
label_pcb6 = 'aa5539'
ss_pcb6 = ''




print '#######################################################################'
print '第一步：输出串口设备清单...'
port_list = list(serial.tools.list_ports.comports())
port_num = len(port_list)
if port_num == 0:
   print('当前未插入任何串口，请检查')
   exit()
else:
	print '当前插入了'+str(port_num)+'个串口。'
	print '串口信息清单如下：'
	print '----------------------------------------------------------------'
	for i in range(0,port_num):
		print port_list[i]
		print '----------------------------------------------------------------'
print '#######################################################################'
print '第二步：确定串口对应关系...'

label_rplidar = 'CP2102'
for i in range(0, port_num):
	if( label_rplidar in str(port_list[i])):
		rplidar_pluged = True
		print 'rplidar: '+ str(port_list[i]).split('-')[0]
		port_list.remove(port_list[i])
		break
if not rplidar_pluged:
	print '激光雷达未接入，请检查...'


def recv(serial):
	while True:
		data = serial.read_all()
		if data == '':
			continue
		else:
			break
		sleep(0.02)
	return data



port_num = len(port_list)






for i in range(0, port_num):
	ss = ''
	dev_name = str(port_list[i]).split(' ')[0]
	try:
		serial1 = serial.Serial(dev_name, 115200, timeout=0.5)
		
	except Exception as e:
		raise e
		# print e
	finally:
		pass


	while len(ss)/2<151:
		s =recv(serial1)
			
		if s != b'' :
			data= str(binascii.b2a_hex(s))
			
			ss = ss+data
		else:
			print 'receive null'
			# serial.write(data)
			# 
			

	if(label_sensor in ss ):
		print('sensor: '+dev_name)
		sensor_dev_name = dev_name
		ss_sensor = 'aa55'+ss.split('aa55')[1]		
		sensor_pluged = True

	elif(label_xbot in ss):
		print('xbot: '+dev_name)
		xbot_dev_name = dev_name
		ss_xbot = 'aa55'+ss.split('aa55')[1]		
		xbot_pluged = True
	elif(label_pcb6 in ss):
		print('pcb6: '+dev_name)
		pcb6_dev_name = dev_name
		print ss
		ss_pcb6 = 'aa55'+ss.split('aa55')[1]
		pcb6_pluged = True
	elif(label_qbot in ss):
		print('qbot: '+dev_name)
		qbot_dev_name = dev_name
		ss_qbot = 'aa55'+ss.split('aa55')[1]		
		qbot_pluged = True


	

print '#######################################################################'
print '第三步：输出插入设备完整的一帧数据...'
if(rplidar_pluged):
	print '激光雷达设备已插入'
else:
	print '激光雷达设备未插入'
if(xbot_pluged):
	print 'xbot(PCB1): '+ss_xbot
else:
	print 'xbot(PCB1)设备未插入'

if(sensor_pluged):
	print 'sensor(PCB2): '+ss_sensor
else:
	print 'sensor(PCB2)设备未插入'

if(qbot_pluged):
	print 'qbot: '+ss_qbot
else:
	print 'qbot设备未插入'

if(pcb6_pluged):
	print 'pcb6: '+ss_pcb6
else:
	print 'pcb6设备未插入'


print '#######################################################################'
print '第四步：解码验证插入设备数据帧'

def check(s):
	l=len(s)
	i=1
	x=int(s[0:2],16)
	while i<l/2-1:
		x=x^int(s[i*2:i*2+2],16)
		i=i+1
	if x==int(s[-2:],16):
		return True
	else:
		return False
		




if(xbot_pluged):
	if(check(ss_xbot)):
		print "xbot数据校验通过"
		payload = ss_xbot[6:-2]
		label = payload[0:2]
		if(label != '10'):
			print '[数据错误]标志位不为0x10'
		else:
			left_encoder = int(payload[2:6],16)

	else:
		print "[校验错误]xbot数据校验未通过"


if(sensor_pluged):
	if(check(ss_sensor)):
		print "sensor数据校验通过"
		payload = ss_sensor[6:-2]
		label = payload[0:2]
		if(label != '11'):
			print '[数据错误]标志位不为0x10'
		else:
			# print len(payload)
			yaw_platform = struct.unpack('B', payload[2:4].decode('hex'))[0]
			

			pitch_platform = struct.unpack('B', payload[4:6].decode('hex'))[0]


			volume = struct.unpack('B', payload[6:8].decode('hex'))[0]

			acc_x = struct.unpack('<f', payload[8:16].decode('hex'))[0]
			acc_y = struct.unpack('<f', payload[16:24].decode('hex'))[0]
			acc_z = struct.unpack('<f', payload[24:32].decode('hex'))[0]
			gyro_x = struct.unpack('<f', payload[32:40].decode('hex'))[0]
			gyro_y = struct.unpack('<f', payload[40:48].decode('hex'))[0]
			gyro_z = struct.unpack('<f', payload[48:56].decode('hex'))[0]
			mag_x = struct.unpack('<f', payload[56:64].decode('hex'))[0]
			mag_y = struct.unpack('<f', payload[64:72].decode('hex'))[0]
			mag_z = struct.unpack('<f', payload[72:80].decode('hex'))[0]

			yaw = -struct.unpack('<f', payload[80:88].decode('hex'))[0]
			pitch = struct.unpack('<f', payload[88:96].decode('hex'))[0]
			roll = struct.unpack('<f', payload[96:104].decode('hex'))[0]
			q1 = struct.unpack('<f', payload[104:112].decode('hex'))[0]
			q2 = struct.unpack('<f', payload[112:120].decode('hex'))[0]
			q3 = struct.unpack('<f', payload[120:128].decode('hex'))[0]
			q4 = struct.unpack('<f', payload[128:136].decode('hex'))[0]
			
			error_status = int(payload[136:138])
			time_stamp = struct.unpack('<H', payload[138:142].decode('hex'))[0]
			print 'sensor数据解析通过，测试无故障，可以投入使用'


	else:
		print "sensor数据校验未通过"

if(qbot_pluged):
	if(check(ss_qbot)):
		print "qbot数据校验通过"
	else:
		print "qbot数据校验未通过"

if(pcb6_pluged):
	if(check(ss_pcb6)):
		print "pcb6数据校验通过"
	else:
		print "pcb6数据校验未通过"


print '#######################################################################'
print '第五步：获取对应设备的序列号，将串口映射表文件写入系统的/etc/udev/rules.d/目录下'

if(xbot_pluged):
	cmd = 'udevadm info -a -n '+xbot_dev_name+ '|grep AK0'
	(status, output) = commands.getstatusoutput(cmd)
	xbot_serial = output.split('"')[1]
	print 'xbot(PCB1)芯片序列号：'+xbot_serial
else:
	print 'xbot(PCB1)设备未插入'

if(sensor_pluged):
	cmd = 'udevadm info -a -n '+sensor_dev_name+ '|grep ATTRS{serial}'
	(status, output) = commands.getstatusoutput(cmd)

	sensor_serial = output.split('"')[1]
	print 'sensor(PCB2)芯片序列号：'+sensor_serial
else:
	print 'sensor(PCB2)设备未插入'

if(qbot_pluged):
	cmd = 'udevadm info -a -n '+qbot_dev_name+ '|grep AK0'
	(status, output) = commands.getstatusoutput(cmd)
	qbot_serial = output.split('"')[1]
	print 'qbot芯片序列号：'+qbot_serial
else:
	print 'qbot设备未插入'

if(pcb6_pluged):
	cmd = 'udevadm info -a -n '+pcb6_dev_name+ '|grep ATTRS{serial}'
	(status, output) = commands.getstatusoutput(cmd)

	pcb6_serial = output.split('"')[1]
	print 'pcb6芯片序列号：'+pcb6_serial
else:
	print 'pcb6设备未插入'




# (status, output) = commands.getstatusoutput('udevadm info -a -n /dev/ttyUSB1 |grep AK0')
# print output
