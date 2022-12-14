import numpy as np
from numpy import array as npa
import numpy.linalg as la

# CONSTANTS.
WEII3 = 7.292115e-5 # Rotation speed of earth. Radians per second.
REARTH = 6370987.308 # Meters.
SMALL = 9.999999999999999547e-08
DEG_TO_RAD = 0.01745329251994319833

# Input: Azimuth and Elevation in radians.
def BODY_TO_RANGE_AND_ALTITUDE(THETA):
	R = npa(
		[
			[np.cos(THETA), -1.0 * np.sin(THETA)],
			[np.sin(THETA), np.cos(THETA)]
		]
	)
	return R

def FLIGHTPATH_TO_LOCAL_TM(AZIMUTH, ELEVATION):
	FP_TO_L_TM = npa(
		[
			# FORWARD
			[
				np.cos(ELEVATION) * np.cos(AZIMUTH),
				np.cos(ELEVATION) * np.sin(AZIMUTH),
				-np.sin(ELEVATION)
			],

			# LEFT
			[
				-np.sin(AZIMUTH),
				np.cos(AZIMUTH),
				0.0
			],

			# UP
			[
				np.sin(ELEVATION) * np.cos(AZIMUTH),
				np.sin(ELEVATION) * np.sin(AZIMUTH),
				np.cos(ELEVATION)
			]
		]
	)
	return FP_TO_L_TM

# Input: Phi, theta, and psi in radians.
def ORIENTATION_TO_LOCAL_TM(PHI, THETA, PSI):
	O_TO_L_TM = npa(
		[
			[
				np.cos(PSI) * np.cos(THETA),
				np.sin(PSI) * np.cos(THETA),
				-np.sin(THETA)
			],
			[
				np.cos(PSI) * np.sin(THETA) * np.sin(PHI) - np.sin(PSI) * np.cos(PHI),
				np.sin(PSI) * np.sin(THETA) * np.sin(PHI) + np.cos(PSI) * np.cos(PHI),
				np.cos(THETA) * np.sin(PHI)
			],
			[
				np.cos(PSI) * np.sin(THETA) * np.cos(PHI) + np.sin(PSI) * np.sin(PHI),
				np.sin(PSI) * np.sin(THETA) * np.cos(PHI) - np.cos(PSI) * np.sin(PHI),
				np.cos(THETA) * np.cos(PHI)
			]
		]
	)
	return O_TO_L_TM



