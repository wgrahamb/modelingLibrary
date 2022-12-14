import numpy as np

def getAlphaAndBeta(arr):

	alpha = None
	beta = None

	if arr[2] == 0 and arr[0] == 0:
		alpha = 0.0
	else:
		alpha = np.arctan2(arr[2], arr[0])

	speed = np.linalg.norm(arr)
	if speed == 0:
		beta = 0.0
	else:
		beta = np.arcsin(arr[1] / speed)

	return alpha, beta
