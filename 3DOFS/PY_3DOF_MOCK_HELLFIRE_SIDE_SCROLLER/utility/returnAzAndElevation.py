import numpy as np

def returnAzAndElevation(arr):
	az = np.arctan2(arr[1], arr[0])
	el = np.arctan2(arr[2], np.sqrt(arr[0] ** 2 + arr[1] ** 2))
	return az, el

def returnAngle(y, x):

	angle = None

	if y == 0 and x == 0:
		angle = 0.0
	else:
		angle = np.arctan2(y, x)

	return angle

def returnAlphaAndBeta(arr):

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
