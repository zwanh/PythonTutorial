#from __future__ import *
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
from trajectoryInterpolation import *		#import all class and functions
from dogPlatform import *
import threading
from gaitGenerator import *
import time
import Adafruit_PCA9685

frameNum = 5
stepTime = 25		#units:ms
enableAni = False	#enable animation or not	
alfa = 0.5		#smooth coefficiency

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

bias = np.array([[-7, 0, -19],\
				 [-3, -2, 18],\
				 [-10, -8, -10],\
				 [-6, 4, 10]])

def set_servo_angle(angle):
	for i in range(0, 12):
		pwm.set_pwm(i, i, int(angle[i]*2.26+105))


def motorDriver(joints):
	print(joints)
	joints = joints + bias
	leg1 = [90 - joints[0][0], 90 - joints[0][1], 90 - joints[0][2]]
	leg2 = [90 - joints[1][0], 90 + joints[1][1], 90 + joints[1][2]]
	leg3 = [90 - joints[2][0], 90 - joints[2][1], 90 - joints[2][2]]
	leg4 = [90 - joints[3][0], 90 + joints[3][1], 90 +  joints[3][2]]
	leg = leg1 + leg2 + leg3 + leg4
	set_servo_angle(leg)	

def update_fig(num, Tcps, dTcps, dog, lines = None):
	print 'num', num
	if enableAni :
		#smooth algorithem, dTcps are same as Tcps' steps. and the Tcps is a standard gait
		#with the programe running, dog's gait will be same as Tcps
		if num  == 0 :
			tcp = alfa * (dog.getLegTcp() + dTcps[num]) + (1 - alfa) * Tcps[num]
		else:
			tcp = alfa * (dog.getLegTcp() + dTcps[num] - dTcps[num - 1]) + (1 - alfa) * Tcps[num]
		data = plotData(tcp, dog)
		for line, dat in zip(lines, data):
			line.set_data(dat[0:2])
			line.set_3d_properties(dat[2])
		return lines
	else:
		if num  == 0:
			tcp = alfa * (dog.getLegTcp() + dTcps[num]) + (1 - alfa) * Tcps[num]
		else:
			tcp = alfa * (dog.getLegTcp() + dTcps[num] - dTcps[num - 1]) + (1 - alfa) * Tcps[num]
		dog.setLegTcp(tcp)
		legPose = dog.getLegPose()
		motorDriver(legPose)
		t = threading.Timer(stepTime *0.001, update_fig, [(num + 1) % (2 * frameNum), Tcps, dTcps, dog])
		t.start()

def plotData(tcp, dog):
	dog.setLegTcp(tcp)
	fix0 = dog.getFixPoints()
	fix = fix0[0:4, 0:3].T
	knee = dog.getLegKnee()
	leg = np.zeros((4, 3, 3))
	for i in range(4):
		x = np.array([fix0[i][0], knee[i][0], tcp[i][0]])
		y = np.array([fix0[i][1], knee[i][1], tcp[i][1]])
		z = np.array([fix0[i][2], knee[i][2], tcp[i][2]])
		leg[i] = np.array([x,y,z])
	data = [leg[0], leg[1], leg[2], leg[3], fix]
	return data

dog = dogPlatform()

initLegPose1 = np.array([[0, 0, 0],\
						[0, 0, 0],\
						[0, 0, 0],\
						[0, 0, 0]])
dog.setLegPose(initLegPose1)
motorDriver(initLegPose1)

#init plot
data = plotData(dog.getLegTcp(), dog)
pace = gaitPace(-95)
trot = gaitTrot(-80)
walk = gaitWalk(-80)
Tcps = pace.getTcps(20, 50, frameNum)
dTcps = pace.getTcpsRelative(20, 50, frameNum)
#show figure and animation, need a screen
if (enableAni) :
	fig = plt.figure()
	ax = p3.Axes3D(fig)
	ax.set_xlim3d([-200,200])
	ax.set_ylim3d([-200,200])
	ax.set_zlim3d([-200,200])
	ax.set_title('dog simulation')
	#ax.axis('equal')
	lines = [ax.plot(dat[0,0:], dat[1,0:], dat[2,0:])[0] for dat in data]
	dogAni = animation.FuncAnimation(fig, update_fig,frameNum * 2, repeat = True, fargs = (Tcps, dTcps, dog, lines), interval = stepTime)
	plt.show()
#don't show figure and animation, not need a screen
else :
	print("init")
	update_fig(0, Tcps, dTcps,dog)
