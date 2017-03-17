import numpy as np
import serial
from time import sleep

class HXServo :
	#register addresses
	POSITION_REGADDR               	= 0x01
	SPEED_REGADDR           	= 0x02
	ADDR_REGADDR                	= 0x03
	BAUDRATE_REGADDR       		= 0x04
	COMMUNICATION_MODE_REGADDR 	= 0X21
	P_GAIN_REGADDR                	= 0x05
	D_GAIN_REGADDR                	= 0x06
	I_GAIN_REGADDR                	= 0x07
	PDI_GAIN_REGADDR              	= 0x08
	PDI_DEADBAND_REGADDR          	= 0x1f
	ACCELERATION_REGADDR          	= 0x20
	CW_FLEXIBLE_MARGIN_REGADDR    	= 0x09
	CCW_FLEXIBLE_MARGIN_REGADDR   	= 0x0a
	CW_FLEXIBLE_SLOPE_REGADDR     	= 0x0b
	CCW_FLEXIBLE_SLOPE_REGADDR    	= 0x0c
	WORKING_MODE_REGADDR          	= 0x0d
	SERVO_STATE_REGADDR           	= 0x0e
	MAX_POSITION_REGADDR          	= 0x0f
	MIN_POSITION_REGADDR          	= 0x10
	MAX_TEMPERATURE_REGADDR       	= 0x11
	MAX_VOLTAGE_REGADDR           	= 0x12
	MIN_VOLTAGE_REGADDR           	= 0x13
	MAX_TORQUE_REGADDR            	= 0x14
	ALL_WRITEREAD_REGADDR         	= 0x15
	CURRENT_POSITION_REGADDR      	= 0x16
	CURRENT_SPEED_REGADDR         	= 0x17
	CURRENT_TORQUE_REGADDR        	= 0x18
	CURRENT_VOLTAGE_REGADDR       	= 0x19
	CURRENT_TEMPERATURE_REGADDR   	= 0x1a
	ALL_READONLY_REGADDR          	= 0x1b
	FIRMWARE_VERSION_REGADDR      	= 0x1c
	MODEL_CODE_REGADDR            	= 0x1d
	WRITE_FLASH_REGADDR           	= 0x1e
	#setting status
	SETTING_OK       		= 0x01
	SETTING_FAIL     		= 0x02
	#cont				
	READ_REG         		= 0x01
	WRITE_REG       		= 0x02
	ANSWER           		= 0x03
	MASTER_BUSADDR   		= 0x0001
	BROADCAST_BUSADDR		= 1000
	def __init__(self, serialPort):
		self.serialPort = serialPort

	def write(self, devAddr, regAddr, data):
		'''write data to servo device'''
		checkSum = 0
		ret = 0
		if ((devAddr > self.BROADCAST_BUSADDR) or devAddr <= self.MASTER_BUSADDR):
			return ret
		buff = [0xaa, 0x55]	#frame header
		#master bus address, 2 bytes
		buff.append((self.MASTER_BUSADDR >> 8) & 0xff)
		buff.append(self.MASTER_BUSADDR & 0xff)
		
		#device address, 2 bytes
		buff.append((devAddr >> 8) & 0xff)
		buff.append(devAddr & 0xff)
		if buff[-1] == 0xaa:
			buff.append(0)

		#frame length, 1byte
		buff.append(10)
		
		#frame type: write_reg, 1 byte
		buff.append(self.WRITE_REG &0xff)
		if buff[-1] == 0xaa:
			buff.append(0)

		#register address, 1byte
		buff.append(regAddr & 0xff)
		if (buff[-1] == 0xaa):
			buff.append(0)

		#data, 2 bytes
		buff.append((data>>8) & 0xff)
		if buff[-1] == 0xaa:
			buff.append(0)
		buff.append(data & 0xff)
		if buff[-1] == 0xaa:
			buff.append(0)

		#checksum, 1 byte
		checksum = self.checkSumCal(buff, 2, len(buff))
		print hex(checksum)
		buff.append(checksum)
		if buff[-1] == 0xaa:
			buff.append(0)

		#tail
		buff.append(0xaa)
		buff.append(0x81)
			
		#send data by serial port
		print buff
		sendData = ''
		for i in range(0, len(buff)):
			sendData += chr(buff[i])
		self.serialPort.write(sendData)
		print "sendData:", sendData
		print "length of sendData:", len(sendData)
		
		#receive data
		sleep(1)
		receivedData = self.serialPort.readline()
		print "receivedData:", receivedData		
		print "length of receivedData:", len(receivedData)
		self.serialPort.flushInput()
		if len(receivedData) == 0:
			return 0
		receivedBuff = []
		for x in xrange(0, len(receivedData)):
			receivedBuff.append(ord(receivedData[x]))	#transfer chr to a number
		index = len(receivedData)
		print "receivedBuff:", receivedBuff
		
		if ((index >= 13) and (receivedBuff[0] == 0xaa) and (receivedBuff[1] == 0x55) 
			and (receivedBuff[index - 1] == 0x81) and (receivedBuff[index - 2] == 0xaa)):
			#delete frame header and tail
			buffData = []
			for i in xrange(2, index - 2):
				buffData.append(receivedBuff[i])
				if receivedBuff[i] == 0xaa:
					continue

			if((((buffData[0]<<8)|(buffData[1]))==devAddr) and (((buffData[2]<<8)|(buffData[3]))== self.MASTER_BUSADDR)):
				if((buffData[4]==9) and (buffData[5]==self.ANSWER) and (buffData[6]==regAddr)):
					checksum = self.checkSumCal (buffData, 0, 8);
				if((checksum==buffData[8]) and (buffData[7]== self.SETTING_OK)):
					ret = self.SETTING_OK;
		return ret

	def read(self, devAddr, regAddr):
		'''read data from servo device'''
		checkSum = 0
		ret = 0
		if ((devAddr > self.BROADCAST_BUSADDR) or devAddr <= self.MASTER_BUSADDR):
			return ret
		buff = [0xaa, 0x55]	#frame header
		#master bus address, 2 bytes
		buff.append((self.MASTER_BUSADDR >> 8) & 0xff)
		buff.append(self.MASTER_BUSADDR & 0xff)
		
		#device address, 2 bytes
		buff.append((devAddr >> 8) & 0xff)
		buff.append(devAddr & 0xff)
		if buff[-1] == 0xaa:
			buff.append(0)

		#frame length, 1byte
		buff.append(8)
		
		#frame type: write_reg, 1 byte
		buff.append(self.READ_REG &0xff)
		if buff[-1] == 0xaa:
			buff.append(0)

		#register address, 1byte
		buff.append(regAddr & 0xff)
		if (buff[-1] == 0xaa):
			buff.append(0)

		#checksum, 1 byte
		checksum = self.checkSumCal(buff, 2, len(buff))
		print hex(checksum)
		buff.append(checksum)
		if buff[-1] == 0xaa:
			buff.append(0)

		#tail
		buff.append(0xaa)
		buff.append(0x81)
		
		#send data by serial port
		print buff
		sendData = ''
		for i in range(0, len(buff)):
			sendData += chr(buff[i])
		self.serialPort.write(sendData)
		print "sendData:", sendData
		print "length of sendData:", len(sendData)

		#receive data
		sleep(1)
		receivedData = self.serialPort.readline()
		print "receivedData:", receivedData		
		print "length of receivedData:", len(receivedData)
		self.serialPort.flushInput()
		if len(receivedData) == 0:
			return 0
		receivedBuff = []
		for x in xrange(0, len(receivedData)):
			receivedBuff.append(ord(receivedData[x]))	#transfer chr to a number
		index = len(receivedData)
		print receivedBuff

		buffData = []
		if ((index >= 13) and (receivedBuff[0] == 0xaa) and (receivedBuff[1] == 0x55) 
			and (receivedBuff[index - 1] == 0x81) and (receivedBuff[index - 2] == 0xaa)):
			for i in xrange(2, index - 2):
				buffData.append(receivedBuff[i])
				if receivedBuff[i] == 0xaa:
					continue
			if((((buffData[0]<<8)|(buffData[1]))==devAddr) and (((buffData[2]<<8)|(buffData[3]))==self.MASTER_BUSADDR)):
				if((buffData[4]==9) and (buffData[5]==self.ANSWER) and (buffData[6]==regAddr)):
					checksum = self.checkSumCal (buffData, 0, 9);
				if(checksum==buffData[9]):
					ret = (buffData[7] << 8) | buffData[8];
		return ret

	def checkSumCal(self, data, start, end):
		check = 0x00 
		for x in xrange(start, end):
			print hex(data[x])
			check = int(check) ^ int(data[x])
		return check

ser = serial.Serial('/dev/ttyUSB0', 921600, timeout = 0.5)
servo = HXServo(ser)
ans = servo.read(0x07, servo.P_GAIN_REGADDR)
servo.write(7, servo.POSITION_REGADDR, 1000)
#servo.write(1000, 0x1d, 1)
