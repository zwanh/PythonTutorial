import numpy as np
import serial
from time import sleep
import string
import binascii

class HXServo :
	'''servo comunication'''
	def __init__(self):
		'''initialize the constant var'''
		self.ser = serial.Serial('/dev/ttyUSB0', 921600, timeout = 0.5)
		#register addresses statement
		self.HX_POSITION               = 0x01
		self.HX_SPEED                  = 0x02
		self.HX_ADDR                   = 0x03
		self.HX_BAUDRATE               = 0x04
		self.HX_COMMUNICATION_MODE     = 0x21
		self.HX_P_GAIN                 = 0x05
		self.HX_D_GAIN                 = 0x06
		self.HX_I_GAIN                 = 0x07
		self.HX_PDI_GAIN               = 0x08
		self.HX_PDI_DEADBAND           = 0x1f
		self.HX_ACCELERATION           = 0x20
		self.HX_CW_FLEXIBLE_MARGIN     = 0x09
		self.HX_CCW_FLEXIBLE_MARGIN    = 0x0a
		self.HX_CW_FLEXIBLE_SLOPE      = 0x0b
		self.HX_CCW_FLEXIBLE_SLOPE     = 0x0c
		self.HX_WORKING_MODE           = 0x0d
		self.HX_SERVO_STATE            = 0x0e
		self.HX_MAX_POSITION           = 0x0f
		self.HX_MIN_POSITION           = 0x10
		self.HX_MAX_TEMPERATURE        = 0x11
		self.HX_MAX_VOLTAGE            = 0x12
		self.HX_MIN_VOLTAGE            = 0x13
		self.HX_MAX_TORQUE             = 0x14
		self.HX_ALL_WRITEREAD          = 0x15
		self.HX_CURRENT_POSITION       = 0x16
		self.HX_CURRENT_SPEED          = 0x17
		self.HX_CURRENT_TORQUE         = 0x18
		self.HX_CURRENT_VOLTAGE        = 0x19
		self.HX_CURRENT_TEMPERATURE    = 0x1a
		self.HX_ALL_READONLY           = 0x1b
		self.HX_FIRMWARE_VERSION       = 0x1c
		self.HX_MODEL_CODE             = 0x1d
		self.HX_WRITE_FLASH            = 0x1e
		#setting status
		self.SETTING_OK                = 0x01
		self.SETTING_FAIL              = 0x02
		#control bytes
		self.READ_REG                  = 0x01
		self.WRITE_REG                 = 0x02
		self.ANSWER                    = 0x03
		self.MASTER_BUSADDR            = 0x0001
		self.BROADCAST_BUSADDR         = 1000
	def write(self, devAddr, regAddr, data):
		'''write data to servo device'''
		checkSum = 0
		index = 0
		ret = 0
		if ((devAddr > self.BROADCAST_BUSADDR) or devAddr <= self.MASTER_BUSADDR):
			return ret
		buff = [0xaa, 0x55]	#hea
		#master bus address
		buff.append((self.MASTER_BUSADDR >> 8) & 0xff)
		buff.append(self.MASTER_BUSADDR & 0xff)
		#device address
		buff.append((devAddr >> 8) & 0xff)
		buff.append(devAddr & 0xff)
		if buff[len(buff) - 1] == 0xaa:
			buff.append(0)
		#length
		buff.append(10)
		
		#type
		buff.append(self.WRITE_REG &0xff)
		if buff[len(buff) - 1] == 0xaa:
			buff.append(0)
		#register address
		buff.append(regAddr & 0xff)
		if (buff[len(buff) - 1] == 0xaa):
			buff.append(0)
		#data
		buff.append((data>>8) & 0xff)
		if buff[len(buff)-1] == 0xaa:
			buff.append(0)
		buff.append(data & 0xff)
		if buff[len(buff) - 1] == 0xaa:
			buff.append(0)
		#checksum
		checksum = self.checkSumCal(buff, 2, len(buff))
		print hex(checksum)
		buff.append(checksum)
		if buff[len(buff) - 1] == 0xaa:
			buff.append(0)
		#tail
		buff.append(0xaa)
		buff.append(0x81)
			
		print buff
		strBuff = ''
		for i in range(0, len(buff)):
			strBuff += chr(buff[i])
		self.ser.write(strBuff)
		print "strBuff:", strBuff
		print "length of strBuff:", len(strBuff)
		
		#receive data
		reBuff = []
		sleep(1)
		data = self.ser.readline()
		print 1, data		
		self.ser.flushInput()
		
		buffData = np.zeros(24)
		if ((index >= 13) and (buff[0] == 0xaa) and (buff[1] == 0x55) 
			and (buff[index - 1] == 0x81) and (buff[index - 2] == 0xaa)):
			for i in xrange(2, index - 2):
				buffData[i - 2] = buff[i]
				if buff[i] == 0xaa:
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
		index = 0
		ret = 0
		if ((devAddr >= self.BROADCAST_BUSADDR) or devAddr <= self.MASTER_BUSADDR):
			return ret
		buff = [0xaa, 0x55]	#hea
		#master bus address
		buff.append((self.MASTER_BUSADDR >> 8) & 0xff)
		buff.append(self.MASTER_BUSADDR & 0xff)
		#device address
		buff.append((devAddr >> 8) & 0xff)
		buff.append(devAddr & 0xff)
		if buff[len(buff) - 1] == 0xaa:
			buff.append(0)
		#length
		buff.append(8)
		
		#type
		buff.append(self.READ_REG &0xff)
		if buff[len(buff) - 1] == 0xaa:
			buff.append(0)
		#register address
		buff.append(regAddr & 0xff)
		if (buff[len(buff) - 1] == 0xaa):
			buff.append(0)
		#checksum
		checksum = self.checkSumCal(buff, 2, len(buff))
		print hex(checksum)
		buff.append(checksum)
		if buff[len(buff) - 1] == 0xaa:
			buff.append(0)
		#tail
		buff.append(0xaa)
		buff.append(0x81)
		
		
		print buff
		strBuff = ''
		for i in range(0, len(buff)):
			strBuff += chr(buff[i])
		self.ser.write(strBuff)
		print "strBuff:", strBuff
		print "length of strBuff:", len(strBuff)
		#receive data
		reBuff = []
		sleep(1)
		data = self.ser.readline()
		print 1, data		
		self.ser.flushInput()

		#rceive nothing?
		print "received data: ", data, len(data)
		while True:
			index = self.ser.inWaiting()
			print 'index:', index
			if index != 0:
				reBuff = self.ser.read(64)
				self.ser.flushInput()
				break
			sleep(0.1)
		buffData = np.zeros(24)
		print 'received buff:', reBuff
		print 'length of buff', len(reBuff)
		if ((index >= 13) and (buff[0] == 0xaa) and (buff[1] == 0x55) 
			and (buff[index - 1] == 0x81) and (buff[index - 2] == 0xaa)):
			for i in xrange(2, index - 2):
				buffData[i - 2] = buff[i]
				if buff[i] == 0xaa:
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


servo = HXServo()
#ans = servo.read(0x07, servo.HX_P_GAIN)
servo.write(1000, servo.HX_BAUDRATE, 5)
servo.write(1000, 0x1d, 1)
