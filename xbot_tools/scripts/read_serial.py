import serial
import string
import binascii
from time import sleep

def recv(serial):
	while True:
		data = serial.read_all()
		if data == '':
			continue
		else:
			break
		sleep(0.02)
	return data

if __name__ == '__main__':
	serial = serial.Serial('/dev/ttyUSB0', 4800, timeout=0.5)  #/dev/ttyUSB0
	if serial.isOpen() :
		print("open success")
	else :
		print("open failed")

	while True:
		s =recv(serial)
		if s != b'' :
			data= str(binascii.b2a_hex(s))
			print("receive : ",data)
			# serial.write(data)
