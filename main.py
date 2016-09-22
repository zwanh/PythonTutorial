import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
from trajectoryInterpolation import *		#import all class and functions
from dogPlatform import *
frameNum = 20

def genDogLegTcps(dog):
	'''generate a series of dog legTcps'''
	tcp = dog.getLegTcp()
	dTcp1 = np.array([[0, 30, 50, 0],\
			  [0, -30, 0, 0],\
			  [0, -30, 0, 0],\
			  [0, 30, 50, 0]])
	dTcp2 = np.array([[0, 60, 0, 0],\
			  [0, -60, 0, 0],\
			  [0, -60, 0, 0],\
			  [0, 60, 0, 0]])
	A1 = tcpTrajectoryInterp(tcp, dTcp1, dTcp2)
	
	tcp = tcp + dTcp2
	dTcp1 = np.array([[0, -30, 0, 0],\
			  [0, 30, 50, 0],\
			  [0, 30, 50, 0],\
			  [0, -30, 0, 0]])
	dTcp2 = np.array([[0, -60, 0, 0],\
			  [0, 60, 0, 0],\
			  [0, 60, 0, 0],\
			  [0, -60, 0, 0]])
	A2 = tcpTrajectoryInterp(tcp, dTcp1, dTcp2)
	Tcps1 = [tcpOnTrajectory(A1, np.float(t)/frameNum) for t in range(1, frameNum+1)]
	Tcps2 = [tcpOnTrajectory(A2, np.float(t)/frameNum) for t in range(1, frameNum+1)]
	Tcps = Tcps1 + Tcps2
	return Tcps

def update_fig(num, Tcps, lines, dog):
	print 'num', num
	tcp = Tcps[num]
	data = plotData(tcp, dog)
	for line, dat in zip(lines, data):
		line.set_data(dat[0:2])
		line.set_3d_properties(dat[2])
	return lines

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

fig = plt.figure()
ax = p3.Axes3D(fig)

dog = dogPlatform()
initLegPose = np.array([[0, -25, 50],\
			[0, -25, 50],\
			[0, -25, 50],\
			[0, -25, 50]])
dog.setLegPose(initLegPose)
tcp = dog.getLegTcp()
dTcp1 = np.array([[0, -30, 0, 0],\
		  [0, 30, 0, 0],\
		  [0, 30, 0, 0],\
		  [0, -30, 0, 0]])
dog.setLegTcp(tcp + dTcp1)
#dogShow(dog)

#init plot
data = plotData(dog.getLegTcp(), dog)
lines = [ax.plot(dat[0,0:], dat[1,0:], dat[2,0:])[0] for dat in data]

ax.set_xlim3d([-200,200])
ax.set_ylim3d([-200,200])
ax.set_zlim3d([-200,200])
ax.set_title('dog simulation')
#ax.axis('equal')

Tcps = genDogLegTcps(dog)
dogAni = animation.FuncAnimation(fig, update_fig,frameNum * 2, repeat = True,
			 fargs = (Tcps, lines, dog), interval = 50)
#update_fig(frameNum, Tcps, lines, dog)
plt.show()


