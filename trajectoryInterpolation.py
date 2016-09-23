import numpy as np
from gauss import *

def polynomialInterpolation(x1, x2, x3, dx0, dx1):
	'''Given three one dimension points and the curve's derivative vectors of start and end points,
calculate a four order polynomial curve(x = n4*t^4 + n3*t^3 + n2*t^2 + n1*t + n0) based on t = [0, 0.5, 1],
[x1, x2, x3] <=> t=[0, 0.5, 1]
[dx0, dx1] <=> t = [0, 1]
return polynomial coefficients c = [n4, n3, n2, n1, n0]
	'''
	b = np.array([x3, x2, dx1, dx0, x1], dtype = np.float)
	#np.array() <=> np.matrix()
	A = np.array( [[1, 1, 1, 1, 1],\
			[0.5**4, 0.5**3, 0.5**2, 0.5, 1],\
			[4, 3, 2, 1, 0],\
			[0, 0, 0, 1, 0],\
			[0, 0, 0, 0, 1]], dtype = np.float)
	x = gauss(A, b)
	return x

def trajectoryInterpolation(p1, p2, p3, dp0, dp1):
	'''Given three n dimension points and the curve's derivative vectors of start and end points,
calculate a four order polynomial curve
return coefficiency matrix n*5 '''
	n = len(p1)
	ax = polynomialInterpolation(p1[0], p2[0], p3[0], dp0[0], dp1[0])	
	ay = polynomialInterpolation(p1[1], p2[1], p3[1], dp0[1], dp1[1])	
	az = polynomialInterpolation(p1[2], p2[2], p3[2], dp0[2], dp1[2])
	
	return np.array([ax,ay,az], dtype = np.float)

def pointOnTrajectory(A, t):
	'''Calculate point on the polynomial curve where t is given'''
	T = np.array([t**4, t**3, t**2, t, 1], dtype = np.float)
	point = np.dot(A, T)
	return point

def tcpTrajectoryInterp(tcp, dTcp1, dTcp2):
	tcp1 = tcp
	tcp2 = tcp + dTcp1
	tcp3 = tcp + dTcp2
	A = [np.zeros((5,3)), np.zeros((5,3)), np.zeros((5,3)), np.zeros((5,3))]
	for i in range(0, 4):
		p1 = tcp1[i, 0:3]
		p2 = tcp2[i, 0:3]
		p3 = tcp3[i, 0:3]
		dp0 = p2 - p1
		dp1 = p3 - p2
		A[i] = trajectoryInterpolation(p1, p2, p3, dp0, dp1)
	return A

def tcpOnTrajectory(A,t):
	tcp = np.ones((4,4))
	for i in range(0, 4):
		tcp[i, 0:3] = pointOnTrajectory(A[i], t)
	return tcp

if __name__ == '__main__':
	p1 = np.array([0, 0, 0], dtype = np.float)
	p2 = np.array([0, 1, 1], dtype = np.float)
	p3 = np.array([0, 2, 0], dtype = np.float)
	dp0 = np.array([0, 1, 0], dtype = np.float)
	dp1 = np.array([0, 1, 0], dtype = np.float)
	
	A = trajectoryInterpolation(p1, p2, p3, dp0, dp1)
	print('A = ', A)
	t = 0.5	
	print('point is: ', pointOnTrajectory(A, t))	
