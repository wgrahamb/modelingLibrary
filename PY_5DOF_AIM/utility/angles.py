import numpy as np

def returnAzAndElevation(arr):
	az = np.arctan2(arr[1], arr[0])
	el = np.arctan2(arr[2], np.sqrt(arr[0] ** 2 + arr[1] ** 2))
	return az, el

def projection(unitvector, vector):
    return ((unitvector.reshape(-1, 1) * unitvector) @ vector.reshape(-1, 1)).flatten()