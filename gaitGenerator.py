#this classes generat some standard gaits, like walk, pace, trot and run
import numpy as np
from dogPlatform import *
from trajectoryInterpolation import *           #import all class and functions

class gaitPace:
	'''generate walk gait, step order:14-23-14-23'''
	def __init__(self):
		self.dog = dogPlatform()
		self.initLegTcp = np.array([[-self.dog.width / 2.0, self.dog.height / 2.0, -150, 1],
					[self.dog.width / 2.0, self.dog.height / 2.0, -150, 1],
					[-self.dog.width / 2.0, -self.dog.height / 2.0, -150, 1],
					[self.dog.width / 2.0, -self.dog.height / 2.0, -150, 1]])
		self.dTcp = np.array(  [[0, -1, 0, 0],
					[0, 1, 0, 0],
					[0, 1, 0, 0],
					[0, -1, 0, 0]])
		self.dtcpForward_1 = np.array([[0, 1, 0, 0],
					     [0, -1, 0, 0],
				 	     [0, -1, 0, 0],
					     [0, 1, 0, 0]])
		self.dtcpLift_1 = np.array([[0, 0, 1, 0],
					    [0, 0, 0, 0],
					    [0, 0, 0, 0],
					    [0, 0, 1, 0]])
		self.dtcpPut_1 = np.array([[0, 2, 0, 0],
					   [0, -2, 0, 0],
					   [0, -2, 0, 0],
					   [0, 2, 0, 0]])

		self.dtcpForward_2 = np.array([[0, -1, 0, 0],
					     [0, 1, 0, 0],
				 	     [0, 1, 0, 0],
					     [0, -1, 0, 0]])
		self.dtcpLift_2 = np.array([[0, 0, 0, 0],
					    [0, 0, 1, 0],
					    [0, 0, 1, 0],
					    [0, 0, 0, 0]])
		self.dtcpPut_2 = np.array([[0, -2, 0, 0],
					   [0, 2, 0, 0],
					   [0, 2, 0, 0],
					   [0, -2, 0, 0]])
	def getTcps(self, step, lift, frameNum):
		tcp = self.initLegTcp + self.dTcp * step / 2.0
		dTcp1 = self.dtcpForward_1 * step / 2.0 + self.dtcpLift_1 * lift
		dTcp2 = self.dtcpPut_1 * step / 2.0
		A1 = tcpTrajectoryInterp(tcp, dTcp1, dTcp2)
		tcp = tcp + dTcp2
		dTcp1 = self.dtcpForward_2 * step / 2.0 + self.dtcpLift_2 * lift
		dTcp2 = self.dtcpPut_2 * step / 2.0
		A2 = tcpTrajectoryInterp(tcp, dTcp1, dTcp2)
		Tcps1 = [tcpOnTrajectory(A1, np.float(t) / frameNum) for t in range(1, frameNum + 1)]
		Tcps2 = [tcpOnTrajectory(A2, np.float(t) / frameNum) for t in range(1, frameNum + 1)]
		Tcps = Tcps1 + Tcps2
		return Tcps
	def getTcpsRelative(self, step, lift, frameNum):
		tcp = np.zeros((4,4))
		dTcp1 = self.dtcpForward_1 * step / 2.0 + self.dtcpLift_1 * lift
		dTcp2 = self.dtcpPut_1 * step / 2.0
		A1 = tcpTrajectoryInterp(tcp, dTcp1, dTcp2)
		tcp = tcp + dTcp2
		dTcp1 = self.dtcpForward_2 * step / 2.0 + self.dtcpLift_2 * lift
		dTcp2 = self.dtcpPut_2 * step / 2.0
		A2 = tcpTrajectoryInterp(tcp, dTcp1, dTcp2)
		Tcps1 = [tcpOnTrajectory(A1, np.float(t) / frameNum) for t in range(1, frameNum + 1)]
		Tcps2 = [tcpOnTrajectory(A2, np.float(t) / frameNum) for t in range(1, frameNum + 1)]
		Tcps = Tcps1 + Tcps2
		return Tcps



class gaitTrot:
	'''generate walk gait, step order:14-23-14-23'''
	def __init__(self):
		self.dog = dogPlatform()
		self.initLegTcp = np.array([[-self.dog.width / 2.0, self.dog.height / 2.0, -150, 1],
					[self.dog.width / 2.0, self.dog.height / 2.0, -150, 1],
					[-self.dog.width / 2.0, -self.dog.height / 2.0, -150, 1],
					[self.dog.width / 2.0, -self.dog.height / 2.0, -150, 1]])
		self.dTcp = np.array(  [[0, -1, 0, 0],
					[0, 1, 0, 0],
					[0, -1, 0, 0],
					[0, 1, 0, 0]])
		self.dtcpForward_1 = np.array([[0, 1, 0, 0],
					     [0, -1, 0, 0],
				 	     [0, 1, 0, 0],
					     [0, -1, 0, 0]])
		self.dtcpLift_1 = np.array([[0, 0, 1, 0],
					    [0, 0, 0, 0],
					    [0, 0, 1, 0],
					    [0, 0, 0, 0]])
		self.dtcpPut_1 = np.array([[0, 2, 0, 0],
					   [0, -2, 0, 0],
					   [0, 2, 0, 0],
					   [0, -2, 0, 0]])

		self.dtcpForward_2 = np.array([[0, -1, 0, 0],
					     [0, 1, 0, 0],
				 	     [0, -1, 0, 0],
					     [0, 1, 0, 0]])
		self.dtcpLift_2 = np.array([[0, 0, 0, 0],
					    [0, 0, 1, 0],
					    [0, 0, 0, 0],
					    [0, 0, 1, 0]])
		self.dtcpPut_2 = np.array([[0, -2, 0, 0],
					   [0, 2, 0, 0],
					   [0, -2, 0, 0],
					   [0, 2, 0, 0]])
	def getTcps(self, step, lift, frameNum):
		tcp = self.initLegTcp + self.dTcp * step / 2.0
		dTcp1 = self.dtcpForward_1 * step / 2.0 + self.dtcpLift_1 * lift
		dTcp2 = self.dtcpPut_1 * step / 2.0
		A1 = tcpTrajectoryInterp(tcp, dTcp1, dTcp2)
		tcp = tcp + dTcp2
		dTcp1 = self.dtcpForward_2 * step / 2.0 + self.dtcpLift_2 * lift
		dTcp2 = self.dtcpPut_2 * step / 2.0
		A2 = tcpTrajectoryInterp(tcp, dTcp1, dTcp2)
		Tcps1 = [tcpOnTrajectory(A1, np.float(t) / frameNum) for t in range(1, frameNum + 1)]
		Tcps2 = [tcpOnTrajectory(A2, np.float(t) / frameNum) for t in range(1, frameNum + 1)]
		Tcps = Tcps1 + Tcps2
		return Tcps
