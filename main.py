import numpy as np
from trajectoryInterpolation import *		#import all class and functions
from dogPlatform import *

dog = dogPlatform()

tcp = np.array([[-150, 200, -130, 1],\
               [250, 250, -130, 1],\
               [-250, -250, -130, 1],\
               [150, -200, -130, 1]])


initLegPose = np.array([[0, -45, 90],\
			[0, -45, 90],\
			[0, -45, 90],\
			[0, -45, 90]])
dog.setLegPose(initLegPose)
#dogShow(dog)
tcp = dog.getLegTcp()
print('tcp', tcp)
dTcp1 = np.array([[0, 10, 20, 0],\
		  [0, 10, 20, 0],\
		  [0, 10, 20, 0],\
		  [0, 10, 20, 0]])

dTcp2 = np.array([[0, 20, 0, 0],\
		  [0, 20, 0, 0],\
		  [0, 20, 0, 0],\
		  [0, 20, 0, 0]])
A = tcpTrajectoryInterp(tcp, dTcp1, dTcp2)
t = 1 
newTcp = tcpOnTrajectory(A,t)
print('newTcp', newTcp)
