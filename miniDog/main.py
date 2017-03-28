import numpy as np
#import matplotlib.pyplot as plt
#import mpl_toolkits.mplot3d.axes3d as p3
#import matplotlib.animation as animation
from trajectoryInterpolation import *		#import all class and functions
from dogPlatform import *
import threading
from gaitGenerator import *
from HXServo import *

from time import sleep
#import Adafruit_PCA9685
import serial

frameNum = 3
stepTime = 0.3		#units:s
alfa = 0.5		#smooth coefficiency
dog = dogPlatform()
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout = 0.005)
servo = HXServo(ser)
#pwm = Adafruit_PCA9685.PCA9685()
#pwm.set_pwm_freq(50)
	
def set_servo_angle(angle):
#	for i in range(0, 8):
#		pwm.set_pwm(i, i, int(angle[i]*2.276+102.4))
	servo.setPosition(2, int(angle[0] * 11.378))
	servo.setPosition(6, int(angle[1] * 11.378))
	servo.setPosition(3, int(angle[2] * 11.378))
	servo.setPosition(7, int(angle[3] * 11.378))
	servo.setPosition(4, int(angle[4] * 11.378))
	servo.setPosition(8, int(angle[5] * 11.378))
	servo.setPosition(5, int(angle[6] * 11.378))
	servo.setPosition(9, int(angle[7] * 11.378))
def motorDriver(joints):
	print 'joints:', joints
	global bias
	leg1 = [-joints[0][1] / 2 + bias[0][0],   joints[0][2] / 2 + bias[0][1]]
	leg2 = [ joints[1][1] / 2 + bias[1][0],  -joints[1][2] / 2 + bias[1][1]]
	leg3 = [-joints[2][1] / 2 + bias[2][0],  -joints[2][2] / 2 + bias[2][1]]
	leg4 = [ joints[3][1] / 2 + bias[3][0],   joints[3][2] / 2 + bias[3][1]]
	leg = leg1 + leg2 + leg3 + leg4
	set_servo_angle(leg)	

def updatePose(num, Tcps, dTcps, dog, lines = None):
	print 'num', num
	if num  == 0:
		tcp = alfa * (dog.getLegTcp() + dTcps[num]) + (1 - alfa) * Tcps[num]
	else:
		tcp = alfa * (dog.getLegTcp() + dTcps[num] - dTcps[num - 1]) + (1 - alfa) * Tcps[num]
	dog.setLegTcp(tcp)
	legPose = dog.getLegPose()
	motorDriver(legPose)
	timer = threading.Timer(stepTime, updatePose, [(num + 1) % (2 * frameNum), Tcps, dTcps, dog])
	timer.start()

def myMain():
	global bias
#	ser = serial.Serial('/dev/ttyAMA0', 115200, timeout = 0.5)
#	servo = HXServo(ser)
	bias = np.array([[-0, 0, -0],\
			 [-0, -0, 0],\
			 [-0, -0, -0],\
			 [-0, 0, 0]])
	print 'Setting servo mode'
	for i in range(2, 10):
		servo.setWorkingMode(i,2)
	print 'Getting bias parameters...'
	bias[0,0] = 156#servo.getPosition(2)*0.08789
	bias[0,1] = 194#servo.getPosition(6)*0.08789
	bias[1,0] = 198#servo.getPosition(3)*0.08789
	bias[1,1] = 185#servo.getPosition(7)*0.08789
	bias[2,0] = 147#servo.getPosition(4)*0.08789
	bias[2,1] = 168#servo.getPosition(8)*0.08789
	bias[3,0] = 226#servo.getPosition(5)*0.08789
	bias[3,1] = 187#servo.getPosition(9)*0.08789
	print "bias:", bias
	
	initLegPose = np.array([ [0, 0, 0],\
				 [0, 0, 0],\
				 [0, 0, 0],\
				 [0, 0, 0]])
	motorDriver(initLegPose)
	
	#init plot
	
	pace = gaitPace(-145)
	trot = gaitTrot(-145)
	walk = gaitWalk(-145)
	Tcps = pace.getTcps(15, 0, frameNum)
	print 'Tcps:', Tcps[1]
	dTcps = pace.getTcpsRelative(15, 0, frameNum)
	print 'dTcps:', dTcps[1]
	dog.setLegTcp(Tcps[1])
	legPose = dog.getLegPose()
	print 'legPose:', legPose
	print("init")
#	updatePose(0, Tcps, dTcps,dog)
	num = 0
	while True:
		print "num:", num
		if num  == 0:
			tcp = alfa * (dog.getLegTcp() + dTcps[num]) + (1 - alfa) * Tcps[num]
		else:
			tcp = alfa * (dog.getLegTcp() + dTcps[num] - dTcps[num - 1]) + (1 - alfa) * Tcps[num]
		dog.setLegTcp(tcp)
		legPose = dog.getLegPose()
		motorDriver(legPose)
		num = (num + 2) % (2 * frameNum)
		sleep(stepTime * 0.001)
if __name__== '__main__':
	myMain()
