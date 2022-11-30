import numpy as np
from numpy import array as npa
from numpy import linalg as la
import pandas as pd

# Input: numpy array of the float type.
def unitvector(a):
    amag = la.norm(a)
    return a / amag

def returnAzAndElevation(arr):
	az = np.arctan2(arr[1], arr[0])
	el = np.arctan2(arr[2], np.sqrt(arr[0] ** 2 + arr[1] ** 2))
	return az, el

def fpaToLocalTm(AZIMUTH, ELEVATION):
	ret = npa(
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
	return ret

class dynamicSeeker:

	def __init__(self, tgtLocalPos, seekerPos, seekerLocalOrient):
		
		relPosU = unitvector(tgtLocalPos - seekerPos)
		seekerToInterceptU = seekerLocalOrient @ relPosU
		seekerToInterceptAz, seekerToInterceptEl = returnAzAndElevation(seekerToInterceptU)

		# SIM CONTROL
		self.go = True
		self.time = 0.0
		self.timeStep = 0.001
		self.integrationStep = 0.001
		self.maxTime = 10
		self.logFile = open("pythonFunctions/secondOrderSeeker/seekerLog.txt", "w")
		self.logFile.write("time boreSightPitch boreSightYaw seekerPitchOffBoreSight seekerYawOffBoreSight targetPitchOffBoreSight targetYawOffBoreSight\n")

		# ASSUME THE SEEKER HAS THE SAME LOCAL ORIENTATION AS THE MISSILE AND HAS BEEN INFORMED OF A TARGET.
		self.boreSight = seekerLocalOrient[0]
		self.targetPitchOffBoreSight = seekerToInterceptEl
		self.targetYawOffBoreSight = seekerToInterceptAz
		self.seekerPos = seekerPos
		self.tgtPos = tgtLocalPos
		self.seekerPitch = 0.0 # RADIANS
		self.seekerYaw = 0.0 # RADIANS
		self.mslLocalOrient = seekerLocalOrient
		self.seekerLocalOrient = seekerLocalOrient # ND
		self.seekerPitchErr = seekerToInterceptEl # ND
		self.seekerYawErr = seekerToInterceptAz # ND
		self.gk = 20 # KALMAN FILTER GAIN >>> ONE PER SECOND
		self.zetak = 0.9 # KALMAN FILTER DAMPING
		self.wnk = 100 # KALMAN FILTER NATURAL FREQUENCY >>> RADIANS PER SECOND
		self.wlr = 0.0 # RADIANS PER SECOND >>> POINTING YAW RATE
		self.wlrd = 0.0 # RADIANS PER SECOND^2 >>> DERIVATIVE OF POINTING YAW RATE
		self.wlr1 = 0.0 # RADIANS PER SECOND >>> YAW SIGHT LINE SPIN RATE
		self.wlr1d = 0.0 # RADIANS PER SECOND^2 >>> DERIVATIVE OF YAW SIGHT LINE SPIN RATE
		self.wlr2 = 0.0 # RADIANS PER SECOND^2 >>> SECOND STATE VARIABLE IN KALMAN FILTER, YAW
		self.wlr2d = 0.0 # RADIANS PER SECOND^3 >>> DERIVATIVE OF SECOND STATE VARIABLE IN KALMAN FILTER, YAW
		self.wlq = 0.0 # RADIANS PER SECOND >>> POINTING PITCH RATE
		self.wlqd = 0.0 # RADIANS PER SECOND^2 >>> DERIVATIVE OF POINTING PITCH RATE
		self.wlq1 = 0.0 # RADIANS PER SECOND >>> PITCH SIGHT LINE SPIN RATE
		self.wlq1d = 0.0 # RADIANS PER SECOND^2 >>> DERIVATIVE OF PITCH SIGHT LINE SPIN RATE
		self.wlq2 = 0.0 # RADIANS PER SECOND^2 >>> SECOND STATE VARIABLE IN KALMAN FILTER, PITCH
		self.wlq2d = 0.0 # RADIANS PER SECOND^3 >>> DERIVATIVE OF SECOND STATE VARIABLE IN KALMAN FILTER, PITCH

	def integrate(self, dy_new, dy, y, step):
		return y + (dy_new + dy) * step / 2

	def updateTarget(self):
		vel = npa([0.0, -1000.0, 0.0])
		self.tgtPos += (vel * self.timeStep)
		self.targetYawOffBoreSight, self.targetPitchOffBoreSight = returnAzAndElevation(self.tgtPos - self.seekerPos)

	def seek(self):
		wsq = self.wnk ** 2 # ND
		gg = self.gk * wsq # ND

		# YAW CHANNEL
		wlr1d_new = self.wlr2 # RADIANS PER SECOND^2
		self.wlr1 = self.integrate(wlr1d_new, self.wlr1d, self.wlr1, self.integrationStep) # RADIANS PER SECOND
		self.wlr1d = wlr1d_new # RADIANS PER SECOND^2
		wlr2d_new = gg * self.seekerYawErr - 2 * self.zetak * self.wnk * self.wlr1d - wsq * self.wlr1 # RADIANS PER SECOND^3
		self.wlr2 = self.integrate(wlr2d_new, self.wlr2d, self.wlr2, self.integrationStep) # RADIANS PER SECOND^2
		self.wlr2d = wlr2d_new # RADIANS PER SECOND^3
		# PITCH CHANNEL
		wlq1d_new = self.wlq2 # RADIANS PER SECOND^2
		self.wlq1 = self.integrate(wlq1d_new, self.wlq1d, self.wlq1, self.integrationStep) # RADIANS PER SECOND
		self.wlq1d = wlq1d_new # RADIANS PER SECOND^2
		wlq2d_new = gg * self.seekerPitchErr - 2 * self.zetak * self.wnk * self.wlq1d - wsq * self.wlq1 # RADIANS PER SECOND^3
		self.wlq2 = self.integrate(wlq2d_new, self.wlq2d, self.wlq2, self.integrationStep) # RADIANS PER SECOND^2
		self.wlq2d = wlq2d_new # RADIANS PER SECOND^3
		
		# YAW CONTROL
		wlrd_new = self.wlr1# RADIANS PER SECOND^2
		self.wlr = self.integrate(wlrd_new, self.wlrd, self.wlr, self.integrationStep) # RADIANS PER SECOND
		self.wlrd = wlrd_new # RADIANS PER SECOND^2
		self.seekerYaw = self.wlr # RADIANS
		# PITCH CONTROL
		wlqd_new = self.wlq1# RADIANS PER SECOND^2
		self.wlq = self.integrate(wlqd_new, self.wlqd, self.wlq, self.integrationStep) # RADIANS PER SECOND
		self.wlqd = wlqd_new # RADIANS PER SECOND^2
		self.seekerPitch = self.wlq # RADIANS

		localRelPos = self.tgtPos - self.seekerPos # METERS
		seekerAttitudeToLocalTM = fpaToLocalTm(self.seekerYaw, -self.seekerPitch) # ND
		self.seekerLocalOrient = seekerAttitudeToLocalTM @ self.mslLocalOrient # ND
		seekerToInterceptRelPos = (self.seekerLocalOrient @ localRelPos) * npa([1.0, 0.5, 0.2]) # METERS >>> ARRAY AT THE END SIMULATES ERROR
		self.seekerYawErr, self.seekerPitchErr = returnAzAndElevation(seekerToInterceptRelPos) # RADIANS

	def logData(self):
		self.logFile.write(f"{self.time:.3f} 0.0 0.0 {self.seekerPitch} {self.seekerYaw} {self.targetPitchOffBoreSight} {self.targetYawOffBoreSight}\n")

	def main(self):
		while self.go:
			self.time += self.timeStep
			self.updateTarget()
			self.seek()
			self.logData()
			print(f"{self.time:.3f} {self.seekerPitchErr:.10f} {self.seekerYawErr:.10f}")
			if self.time > self.maxTime:
				self.go = False



if __name__ == "__main__":
	tgtPos = npa([5000.0, 5000.0, 5000.0])
	seekerPos = np.zeros(3)
	seekerLocalOrient = fpaToLocalTm(0.1, -0.1)
	x = dynamicSeeker(tgtPos, seekerPos, seekerLocalOrient)
	x.main()