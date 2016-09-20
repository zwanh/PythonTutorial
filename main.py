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
dogShow(dog)












