import numpy as np
import gauss

def polynomialInterpolation(x1, x2, x3, dx0, dx1):
	'''Given three one dimension points and the curve's derivative vectors of start and end points,
calculate a four order polynomial curve(x = n4*t^4 + n3*t^3 + n2*t^2 + n1*t + n0) based on t = [0, 0.5, 1],
[x1, x2, x3] <=> t=[0, 0.5, 1]
[dx0, dx1] <=> t = [0, 1]
return polynomial coefficients c = [n4, n3, n2, n1, n0]
	'''
	b = np.array([x3, x2, dx1, dx0, x1], dtype = np.float)
	A = np.matrix( [[1, 1, 1, 1, 1],\
			[0.5**4, 0.5**3, 0.5**2, 0.5, 1],\
			[4, 3, 2, 1, 0],\
			[0, 0, 0, 1, 0],\
			[0, 0, 0, 0, 1]], dtype = np.float)
	#print('A = ', A)
	#print('b = ', b)
	x = gauss.gauss(A, b)
	print('x = ', x)
	return x

def trajectoryInterpolation(p1, p2, p3, dp0, dp1):
	'''Given three n dimension points and the curve's derivative vectors of start and end points,
calculate a four order polynomial curve
return coefficiency matrix n*5 '''
	n = len(p1)
	ax = polynomialInterpolation(p1[0], p2[0], p3[0], dp0[0], dp1[0])	
	ay = polynomialInterpolation(p1[1], p2[1], p3[1], dp0[1], dp1[1])	
	az = polynomialInterpolation(p1[2], p2[2], p3[2], dp0[2], dp1[2])
	
	return np.matrix([ax,ay,az], dtype = np.float)

def pointOnTrajectory(A, t):
	'''Calculate point on the polynomial curve where t is given'''
	T = np.array([t**4, t**3, t**2, t, 1], dtype = np.float)
	point = np.dot(A, T)
	return point

#example
#p1 = np.array([0, 0, 0], dtype = np.float)
#p2 = np.array([0, 1, 1], dtype = np.float)
#p3 = np.array([0, 2, 0], dtype = np.float)
#pd0 = np.array([0, 1, 0], dtype = np.float)
#pd1 = np.array([0, 1, 0], dtype = np.float)
#
#A = trajectoryInterpolation(p1, p2, p3, dp0, dp1)
#print('A = ', A)
#t = 0.5
#print('point is: ', pointOnTrajectory(A, t))	
