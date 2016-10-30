#this is a serial example
import serial 
from time import sleep

ser = serial.Serial('/dev/ttyAMA0', 9600, timeout = 0.5)
def recv(serial):
	while True:
		count = serial.inWaiting()		
		if count != 0:
			data = serial.read(count)
			serial.flushInput()
			sleep(0.1)
			return data

#chr() change num(10) into a ASCII
#int('55', 16) char 0x55 to num(10)
disMesure = chr(int('55', 16))
tempMesure = chr(int('50', 16))

while True:
	ser.write(disMesure)
	distance  = recv(ser) 
	print 'dis: ', ord(distance[0])*256 + ord(distance[1])

#	ser.write(tem)
#	temporature = recv(ser)
#	print 'temp: ', ord(temporature) - 45
