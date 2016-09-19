import numpy as np

def gauss(aa, bb):
	'''calculate linear functions (aa * x = bb) by gauss method 
aa is a Matrix, bb is an array
return x'''
	a = aa.copy()
	b = bb.copy()
	n = len(b)
	for i in range(0, n - 1):
		for j in range(i + 1, n):
			if a[j, i] != 0.0:
				lam = float(a[j, i]) / a[i, i]
				a[j, (i + 1) : n] = a[j, (i + 1) :] - lam * a[i, (i + 1) : n]
				b[j] = b[j] - lam * b[i]
	for k in range(n - 1, -1, -1):
		b[k] = (b[k] - np.dot(a[k, (k + 1) : n], b[(k + 1) : n])) / a[k, k]

	result = b
	return result

#example
#a = np.matrix([[2, 1, -1], [-3, -1, 2], [-2, 1, 2]], dtype = float)
#b = np.array([8, -11, -3], dtype = float)
#x = gauss(a, b)

