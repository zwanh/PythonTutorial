# this is a class that controls four legged robot
import numpy as np

class dogPlatform:
	'''Record the dog's status'''
	def __init__(self):
		'''Initialize the dog's status'''
		self.bodyPose = np.zeros(6)		#record dog body's pose (x,y,z,yaw,pitch,roll)
		self.legTcp = np.zeros((4,4))		#record four legPose(x,y,z,1)
		self.legPose = np.zeros((4,3))		#record four legs' joins
		#dimensiton parameters
		self.width = 200
		self.height = 300
		self.legFix = np.array([[-self.width / 2., self.height / 2., 0],\
					[self.width / 2., self.height / 2., 0],\
					[-self.width / 2., -self.height / 2., 0],\
					[self.width / 2., -self.height / 2., 0]])
		self.legUpper = np.array([[0, 0, -100],\
					  [0, 0, -100],\
					  [0, 0, -100],\
					  [0, 0, -100]])
		self.legLow = np.array([[0, 0, -100],\
					[0, 0, -100],\
					[0, 0, -100],\
					[0, 0, -100]])
		
	def inverseKinematics(self):
		'''Calculate leg joints from Tcp'''
#		self.legPose = 2 * selflegTcp

	def forwardKinematics(self):
		'''Calculate leg Tcp from joints'''
                for i in range(0,4):
			a = np.radians(self.legPose[i][0])
	                b = np.radians(self.legPose[i][1])
	                c = np.radians(self.legPose[i][2])
			transFix = np.matrix([[1, 0, 0, self.legFix[i][0]],\
					      [0, 1, 0, self.legFix[i][1]],\
					      [0, 0, 1, self.legFix[i][2]],\
					      [0, 0, 0, 1]]) 
			rotate1 = np.matrix([[np.cos(a), 0, np.sin(a), 0],\
	                                     [0, 1, 0, 0],\
	                                     [-np.sin(a), 0, np.cos(a), 0],\
					     [0, 0, 0, 1]])
	                rotate2 = np.matrix([[1, 0, 0, 0],\
	                                     [0, np.cos(b), -np.sin(b), 0],\
	                                     [0, np.sin(b), np.cos(b), 0],\
				             [0, 0, 0, 1]]) 
	
	                transUpper = np.matrix([[1, 0, 0, self.legUpper[i][0]],\
						[0, 1, 0, self.legUpper[i][1]],\
						[0, 0, 1, self.legUpper[i][2]],\
						[0, 0, 0, 1]]) 
	                rotate3 = np.matrix([[1, 0, 0, 0],\
	                                     [0, np.cos(c), -np.sin(c), 0],\
	                                     [0, np.sin(c), np.cos(c), 0],\
					     [0, 0, 0, 1]])
					
			transLow = np.matrix([[1, 0, 0, self.legLow[i][0]],\
					      [0, 1, 0, self.legLow[i][1]],\
					      [0, 0, 1, self.legLow[i][2]],\
					      [0, 0, 0, 1]]) 
			transfer = transFix * rotate1 * rotate2 * transUpper * rotate3 * transLow
			p = np.array([0, 0, 0, 1])
                	self.legTcp[i] = np.dot(transfer,p)

	def setLegTcp(self, legTcp):
		'''Set dog's four legs' Tcp position'''
		selft.legTcp = legTcp
		inverseKinematics()			#update dog's four legs' joints

	def setLegPose(self, legPose):
		'''Set dog's four legs' joints pose'''
		self.legPose = legPose
		forwardKinematics()

	def getLegPose(self):
		'''Get dog's four legs' joints pose'''
		return self.legPose

	def getLegTcp(self):
		'''Get dog's four legs' Tcp position'''
		return self.legTcp


#example
d = dogPlatform()
d.forwardKinematics()
print d.getLegPose()
print d.getLegTcp()
