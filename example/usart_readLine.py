#this is a serial example
import serial 
from time import sleep

ser = serial.Serial('/dev/ttyAMA0', 9600, timeout = 0.5)
def recv(serial):
	while True:
		data = serial.readline()
#		serial.flushInput()
		if data:
			print 1, data
			return data


while True:
	data = recv(ser)
	ser.write(data)
