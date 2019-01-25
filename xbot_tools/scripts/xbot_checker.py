#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Author: roc
# @Date:   2019-01-17 11:59:04
# @Last Modified by:   yowlings@gmail.com
# @Last Modified time: 2019-01-19 22:16:10
###########################################################
#
#该软件实现了以下功能：
#１、判断测试机是否连接了串口设备，并输出串口设备清单；
#２、如果连接了串口设备，确定串口设备与PCB1(PCB1),PCB2(PCB2),rplidar的对应关系
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

PCB1_pluged = False
label_PCB1 = 'aa552d'
ss_PCB1 = ''

PCB2_pluged=False
label_PCB2 = 'aa5547'
ss_PCB2 = ''




print '\033[1;34m １、输出已接入串口设备清单... \033[0m'
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
print '\033[1;34m ２、确定设备接入状态及对应关系... \033[0m'


label_rplidar = 'CP2102'
for i in range(0, port_num):
	if( label_rplidar in str(port_list[i])):
		rplidar_pluged = True
		tmp='rplidar: '+ str(port_list[i]).split('-')[0]
		print '\033[1;32m '+tmp +' \033[0m'
		tmp='rplidar: '+ str(port_list[i]).split('-')[0]
		port_list.remove(port_list[i])
		
		break
if not rplidar_pluged:
	print '\033[1;32m 激光雷达未接入，请检查... \033[0m'


def recv(serial):
	start = time.time()
	while True:
		data = serial.read_all()
		if data == '':
			if time.time()-start >10:
				
				break
			
		else:
			break
		sleep(0.02)
	return data



port_num = len(port_list)
if port_num == 0:
	exit()






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

	

	while len(ss)/2<151 :
		s =recv(serial1)

		if s != b'' :
			data= str(binascii.b2a_hex(s))
			
			ss = ss+data
		else:
			print '\033[1;31m 已连接串口超过10秒未接收到数据，请确认是否未供电。 \033[0m'
			break
			# serial.write(data)
			# 


	if(label_PCB2 in ss ):
		PCB2_dev_name = dev_name
		ss_PCB2 = 'aa55'+ss.split('aa55')[1]
		PCB2_pluged = True

	elif(label_PCB1 in ss):
		
		PCB1_dev_name = dev_name
		ss_PCB1 = 'aa55'+ss.split('aa55')[1]
		PCB1_pluged = True

if(PCB1_pluged):
	tmp = 'PCB1: '+PCB1_dev_name
	print '\033[1;32m '+tmp +' \033[0m'
else:
	print '\033[1;31m PCB1未接入 \033[0m'

if(PCB2_pluged):
	tmp = 'PCB2: '+PCB2_dev_name
	print '\033[1;32m '+tmp +' \033[0m'
else:
	print '\033[1;31m PCB2未接入 \033[0m'

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



print '\033[1;34m ３、PCB1回传数据测试 \033[0m'

if(PCB1_pluged):
	print 'PCB1数据帧: '+ss_PCB1
	if(check(ss_PCB1)):
		print '\033[1;31m PCB1数据校验通过 \033[0m'
		payload = ss_PCB1[6:-2]
		label = payload[0:2]
		if(label != '10'):
			print '[数据错误]标志位不为0x10'
		else:
			left_encoder = int(payload[2:6],16)

	else:
		print '\033[1;31m [校验错误]PCB1数据校验未通过 \033[0m'
else:
	print '\033[1;31m PCB1设备未接入 \033[0m'









print '\033[1;34m ４、PCB1下发数据测试 \033[0m'

if(PCB1_pluged):
	pass
else:
	print '\033[1;31m PCB1设备未接入 \033[0m'










print '\033[1;34m ５、PCB２回传数据测试 \033[0m'
if(PCB2_pluged):
	print 'PCB2数据帧: '+ss_PCB2
	if(check(ss_PCB2)):
		print '\033[1;32m PCB2数据校验通过 \033[0m'
		payload = ss_PCB2[6:-2]
		label = payload[0:2]
		if(label != '11'):
			print '\033[1;31m [数据错误]标志位不为0x11 \033[0m'
		else:
			# print len(payload)
			yaw_platform = struct.unpack('B', payload[2:4].decode('hex'))[0]
			

			pitch_platform = struct.unpack('B', payload[4:6].decode('hex'))[0]


			volume = struct.unpack('B', payload[6:8].decode('hex'))[0]

			acc_x = struct.unpack('<h', payload[8:12].decode('hex'))[0]
			acc_y = struct.unpack('<h', payload[12:16].decode('hex'))[0]
			acc_z = struct.unpack('<h', payload[16:20].decode('hex'))[0]
			gyro_x = struct.unpack('<h', payload[20:24].decode('hex'))[0]
			gyro_y = struct.unpack('<h', payload[24:28].decode('hex'))[0]
			gyro_z = struct.unpack('<h', payload[28:32].decode('hex'))[0]
			mag_x = struct.unpack('<h', payload[32:36].decode('hex'))[0]
			mag_y = struct.unpack('<h', payload[36:40].decode('hex'))[0]
			mag_z = struct.unpack('<h', payload[40:44].decode('hex'))[0]

			yaw = -struct.unpack('<f', payload[44:52].decode('hex'))[0]
			pitch = struct.unpack('<f', payload[52:60].decode('hex'))[0]
			roll = struct.unpack('<f', payload[60:68].decode('hex'))[0]
			q1 = struct.unpack('<f', payload[68:76].decode('hex'))[0]
			q2 = struct.unpack('<f', payload[76:84].decode('hex'))[0]
			q3 = struct.unpack('<f', payload[84:92].decode('hex'))[0]
			q4 = struct.unpack('<f', payload[92:100].decode('hex'))[0]
			
			error_status = struct.unpack('B', payload[100:102].decode('hex'))[0]
			time_stamp = struct.unpack('<H', payload[102:106].decode('hex'))[0]
			print '\033[1;32m PCB2数据解析通过\033[0m'

	else:
		print '\033[1;31m PCB2数据校验未通过 \033[0m'

else:
	print '\033[1;31m PCB2设备未接入 \033[0m'


print '\033[1;34m ６、PCB２下发数据测试 \033[0m'

if(PCB2_pluged):
	pass
else:
	print '\033[1;31m PCB2设备未接入 \033[0m'





# print '#######################################################################'
# print '第五步：获取对应设备的序列号，将串口映射表文件写入系统的/etc/udev/rules.d/目录下'
print '\033[1;34m 7、获取对应设备的序列号，将串口映射表文件写入系统的/etc/udev/rules.d/目录下 \033[0m'
if(PCB1_pluged):
	cmd = 'udevadm info -a -n '+PCB1_dev_name+ '|grep ATTRS{serial}'
	(status, output) = commands.getstatusoutput(cmd)
	PCB1_serial = output.split('"')[1]
	print 'PCB1(PCB1)芯片序列号：'+PCB1_serial

	f=open("../udev/71-xbot.rules",'r')  #your path!
	s= f.read()
	f.close()
	s= s+',ATTRS{serial}==\"'+PCB1_serial+'\"'
	f=open("/etc/udev/rules.d/71-xbot.rules",'w')
	f.write(s)
	f.close()
	print '\033[1;32m PCB1端口映射文件写入系统成功\033[0m'
else:
	print 'PCB1(PCB1)设备未插入'

if(PCB2_pluged):
	cmd = 'udevadm info -a -n '+PCB2_dev_name+ '|grep ATTRS{serial}'
	(status, output) = commands.getstatusoutput(cmd)

	PCB2_serial = output.split('"')[1]
	print 'PCB2(PCB2)芯片序列号：'+PCB2_serial

	f=open("../udev/72-sensor.rules",'r')  #your path!
	s= f.read()
	f.close()
	s= s+',ATTRS{serial}==\"'+PCB2_serial+'\"'
	f=open("/etc/udev/rules.d/72-sensor.rules",'w')
	f.write(s)
	f.close()
	print '\033[1;32m PCB2端口映射文件写入系统成功\033[0m'

else:
	print 'PCB2(PCB2)设备未插入'





