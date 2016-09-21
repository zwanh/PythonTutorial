import numpy as np
from trajectoryInterpolation import *		#import all class and functions
from dogPlatform import *
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

def genDogLegTcps(dog):
	'''generate a series of dog legTcps'''
	
	tcp = dog.getLegTcp()
	dTcp1 = np.array([[0, 30, 50, 0],\
			  [0, 0, 0, 0],\
			  [0, 0, 0, 0],\
			  [0, 0, 0, 0]])
	dTcp2 = np.array([[0, 50, 0, 0],\
			  [0, 0, 0, 0],\
			  [0, 0, 0, 0],\
			  [0, 0, 0, 0]])
	A = tcpTrajectoryInterp(tcp, dTcp1, dTcp2)
	Tcps = [tcpOnTrajectory(A, t/2.) for t in range(3)]
	return Tcps

dog = dogPlatform()

initLegPose = np.array([[0, -45, 90],\
			[0, -45, 90],\
			[0, -45, 90],\
			[0, -45, 90]])
dog.setLegPose(initLegPose)
dogShow(dog)
print 'init pos', dog.getLegTcp()
Tcps = genDogLegTcps(dog)
dog.setLegTcp(Tcps[1])
dogShow(dog)
print 'new pos', dog.getLegTcp()
