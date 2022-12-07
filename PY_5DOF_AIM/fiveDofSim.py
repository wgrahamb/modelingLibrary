
# INCLUDED WITH PYTHON.
from enum import Enum
import time

# PIP INSTALLED LIBRARIES.
import numpy as np
from numpy import array as npa
from numpy import linalg as la
np.set_printoptions(suppress=True, precision=2)

# UTILITY.
from utility.loadPickle import loadpickle as lp
from utility.coordinateTransformations import FLIGHTPATH_TO_LOCAL_TM
from utility.coordinateTransformations import ORIENTATION_TO_LOCAL_TM
from utility.returnAzAndElevation import returnAzAndElevation
from utility.ATM1976 import ATM1976
from utility.trapezoidIntegrate import integrate
from utility.unitVector import unitvector
from utility import loggingFxns as lf

class endChecks(Enum):
	intercept                    = 1
	flying                       = 0
	groundCollision              = -1
	pointOfClosestApproachPassed = -2
	notANumber                   = -3
	maxTimeExceeded              = -4

class fiveDofInterceptor:

	def __init__(
		self,
		targetPos,
		targetVel,
		launchElDeg,
		launchAzDeg,
		launchSpeed,
		launchHgt
	):

		############################################################################
		#
		# AUTHOR - WILSON GRAHAM BEECH
		# REFERENCE - MODELING AND SIMULATION OF AEROSPACE
		# VEHICLE DYNAMICS SECOND EDITON, PETER H. ZIPFEL
		#
		# EAST, NORTH, UP COORDINATE SYSTEM
		#
		# INTERCEPTOR LOCAL ORIENTATION
		# ARRAY 0 >>> LOOKING DOWN THE NOZZLE OF THE INTERCEPTOR
		# ARRAY 1 >>> LOOKING DOWN THE NOZZLE OF THE INTERCEPTOR,
		# THIS POINTS OUT THE LEFT HAND SIDE
		# ARRAY 2 >>> LOOKING DOWN THE NOZZLE OF THE INTERCEPTOR,
		# THIS POINTS OUT THE TOP SIDE OF THE INTERCEPTOR
		#
		#                       POSITIVE NORMAL
		#                           |
		#                           |
		#                           |
		#  POSITIVE SIDE -----------O----------- NEGATIVE SIDE
		#                           |
		#                           |
		#                           |
		#                       NEGATIVE NORMAL
		#
		# NEGATIVE AXIS IS COMING OUT OF THE SCREEN STRAIGHT AT YOU
		# POSITIVE AXIS IS POINTING INTO THE SCREEN DIRECTLY AWAY FROM YOU
		#
		# POSITIVE ALPHA INDICATES NOSE BELOW FREE STREAM VELOCITY
		# POSITIVE BETA INDICATES NOSE LEFT FREE STREAM VELOCITY
		# POSITIVE ROLL INDICATES NORMAL AXIS CLOCKWISELY
		# ROTATED FROM TWELVE O'CLOCK
		#
		# FIN ORIENTATION
		# LOOKING DOWN THE NOZZLE OF THE MISSILE
		#
		#                   FIN 4       FIN 1
		#                           X
		#                   FIN 3       FIN 2
		#
		############################################################################

		# INPUT #
		self.targetPos = targetPos
		self.targetVel = targetVel
		el             = np.radians(launchElDeg) # CONVERT INPUT ELEVATION TO RADS
		az             = np.radians(launchAzDeg) # CONVERT INPUT AZIMUTH TO RADS

		# SIM CONTROL
		self.wallClockStart = time.time()
		self.timeStep       = (1.0 / 100.0) # SECONDS
		self.go             = True
		self.maxTime        = 200 # SECONDS

		# BODY.
		self.tof       = 0.0 # SECONDS
		self.specForce = np.zeros(3) # METERS / S^2
		self.alpha     = 0.0 # RADIANS
		self.beta      = 0.0 # RADIANS

		# FRAME.
		self.ENUtoFLU = FLIGHTPATH_TO_LOCAL_TM(az, -1.0 * el)
		self.posEnu   = npa([0.0, 0.0, launchHgt]) # METERS
		self.velEnu   = self.ENUtoFLU[0] * launchSpeed
		self.accEnu   = np.zeros(3) # METERS / S^2
		

		# INTERCEPTOR CONSTANTS
		self.refArea        = 0.01767 # M^2
		self.nozzleExitArea = 0.00948 # M^2
		self.burnOut        = 3.02 # SECONDS

		# LOOK UP DATA
		self.lookUps = lp("PY_5DOF_AIM/lookUps.pickle")

		 # ATMOSPHERE.
		self.ATMOS = ATM1976()
		self.ATMOS.update(self.posEnu[2], launchSpeed)
		self.RHO   = self.ATMOS.rho # Kilograms per meter cubed.
		self.Q     = self.ATMOS.q # Pascals.
		self.P     = self.ATMOS.p # Pascals.
		self.A     = self.ATMOS.a # Meters per second.
		self.G     = self.ATMOS.g # Meters per second squared.
		self.MACH  = self.ATMOS.mach # Non dimensional.

		# SEEKER AND GUIDANCE
		self.commLimit   = 250
		self.proNavGain  = 4
		self.normCommand = 0.0
		self.sideCommand = 0.0

		# AUTOPILOT #
		self.ta   = 2
		self.tr   = 0.1
		self.gacp = 40

		# PITCH
		self.xi     = 0.0
		self.xid    = 0.0
		self.ratep  = 0.0
		self.ratepd = 0.0
		self.alpd   = 0.0

		# YAW
		self.yi     = 0.0
		self.yid    = 0.0
		self.ratey  = 0.0
		self.rateyd = 0.0
		self.betd   = 0.0

		# END CHECK
		self.lethality    = endChecks.flying
		self.missDistance = 0.0

		# INITIALIZE LOG FILE AND WRITE HEADER
		self.logFile = open("PY_5DOF_AIM/log.txt", "w")
		self.STATE   = self.populateState()
		lf.writeHeader(self.STATE, self.logFile)
		lf.writeData(self.STATE, self.logFile)

	def populateState(self):
		STATE = {
			"tof": self.tof,
			"posE": self.posEnu[0],
			"posN": self.posEnu[1],
			"posU": self.posEnu[2],
			"tgtE": self.targetPos[0],
			"tgtN": self.targetPos[1],
			"tgtU": self.targetPos[2],
			"normComm": self.normCommand,
			"normAch": self.specForce[2],
			"sideComm": self.sideCommand,
			"sideAch": self.specForce[1]
		}
		return STATE

	def fly(self):

		# UPDATE TARGET.
		self.targetPos += (self.timeStep * self.targetVel)

		# ENU TO FLU MATRIX.
		freeStreamAz, freeStreamEl = returnAzAndElevation(self.velEnu)
		freeStreamLocalOrientation = \
			FLIGHTPATH_TO_LOCAL_TM(freeStreamAz, -freeStreamEl)
		attitudeToLocalTransformationMatrix = \
			ORIENTATION_TO_LOCAL_TM(0.0, self.alpha, self.beta)
		self.ENUtoFLU = \
			attitudeToLocalTransformationMatrix @ freeStreamLocalOrientation

		# ATMOSPHERE.
		freeStreamSpeed = la.norm(self.velEnu)
		self.ATMOS.update(self.posEnu[2], freeStreamSpeed)
		self.RHO        = self.ATMOS.rho # Kilograms per meter cubed.
		self.Q          = self.ATMOS.q # Pascals.
		self.P          = self.ATMOS.p # Pascals.
		self.A          = self.ATMOS.a # Meters per second.
		self.G          = self.ATMOS.g # Meters per second squared.
		self.MACH       = self.ATMOS.mach # Non dimensional.
		gravityVec      = npa([0, 0, -1.0 * self.G])

		# KINEMATIC TRUTH SEEKER AND PROPORTIONAL GUIDANCE.
		FLUMslToPipRelPos  = self.ENUtoFLU @ (self.targetPos - self.posEnu)
		FLUMslToPipRelPosU = unitvector(FLUMslToPipRelPos)
		closingVel         = self.ENUtoFLU @ (self.targetVel - self.velEnu)
		closingSpeed       = la.norm(closingVel)
		T1                 = np.cross(FLUMslToPipRelPos, closingVel)
		T2                 = np.dot(FLUMslToPipRelPos, FLUMslToPipRelPos)
		lineOfSightRate    = T1 / T2
		T3                 = -1.0 * self.proNavGain * closingSpeed\
			 * FLUMslToPipRelPosU
		command            = np.cross(T3, lineOfSightRate)
		self.normCommand   = command[2]
		self.sideCommand   = command[1]
		accCommMag         = np.sqrt(self.normCommand ** 2 + self.sideCommand ** 2)
		trigRatio          = np.arctan2(self.normCommand, self.sideCommand)
		if accCommMag > self.commLimit:
			accCommMag = self.commLimit
		self.normCommand   = accCommMag * np.sin(trigRatio)
		self.sideCommand   = accCommMag * np.cos(trigRatio)

		# AEROBALLISTIC ANGLES.
		angleOfAttack    = np.arccos(np.cos(self.alpha) * np.cos(self.beta))
		angleOfAttackDeg = np.degrees(angleOfAttack)
		phiPrime         = np.arctan2(np.tan(self.beta), np.sin(self.alpha))

		# LOOK UPS.
		CD     = None
		thrust = None
		mass   = None
		if self.tof <= self.burnOut:
			mass           = self.lookUps["MASS (KG)"](self.tof)
			CD             = self.lookUps["CD MOTOR ON"]\
				(self.MACH, angleOfAttackDeg)[0]
			thrustSeaLevel = self.lookUps["THRUST (NEWTONS)"](self.tof)
			thrust         = thrustSeaLevel + \
				(101325 - self.P) * self.nozzleExitArea
		else:
			mass   = 38.5
			CD     = self.lookUps["CD MOTOR OFF"]\
				(self.MACH, angleOfAttackDeg)[0]
			thrust = 0.0
		CL = self.lookUps["CL"](self.MACH, angleOfAttackDeg)[0]

		# AERODYNAMICS.
		cosAlpha    = np.cos(self.alpha)
		sinAlpha    = np.sin(self.alpha)
		absAlphaDeg = np.abs(np.degrees(self.alpha))
		absBetaDeg  = np.abs(np.degrees(self.beta))
		CX          = -1 * (CD * cosAlpha - CL * sinAlpha)
		CT          = CD * sinAlpha + CL * cosAlpha
		CZ          = -1 * CT * np.cos(phiPrime)
		CY          = -1 * CT * np.sin(phiPrime)
		CNA         = None
		CYB         = None
		if absAlphaDeg < 10:
			CNA = np.degrees(0.123 + 0.013 * absAlphaDeg)
		else:
			CNA = np.degrees(0.06 * (absAlphaDeg ** 0.625))
		if absBetaDeg < 10:
			CYB = np.degrees(0.123 + 0.013 * absBetaDeg)
		else:
			CYB = np.degrees(0.06 * (absBetaDeg ** 0.625))

		# PITCH AUTOPILOT.
		tip         = freeStreamSpeed * mass / (thrust + self.Q * \
			self.refArea * np.abs(CNA))
		fspz        = self.Q * self.refArea * CZ / mass
		gr          = self.gacp * tip * self.tr / freeStreamSpeed
		gi          = gr / self.ta
		abez        = self.normCommand
		ep          = abez - fspz
		xid_new     = gi * ep
		self.xi     = integrate(xid_new, self.xid, self.xi, self.timeStep)
		self.xid    = xid_new
		ratepc      = -1 * (ep * gr + self.xi)
		ratepd_new  = (ratepc - self.ratep) / self.tr
		self.ratep  = integrate(ratepd_new, self.ratepd, self.ratep, self.timeStep)
		self.ratepd = ratepd_new
		alpd_new    = (tip * self.ratep - self.alpha) / tip
		self.alpha  = integrate(alpd_new, self.alpd, self.alpha, self.timeStep)
		self.alpd   = alpd_new

		# YAW AUTOPILOT.
		tiy         = freeStreamSpeed * mass / (thrust + self.Q * \
			self.refArea * np.abs(CYB))
		fspy        = self.Q * self.refArea * CY / mass
		gr          = self.gacp * tiy * self.tr / freeStreamSpeed
		gi          = gr / self.ta
		abey        = self.sideCommand
		ey          = abey - fspy
		yid_new     = gi * ey
		self.yi     = integrate(yid_new, self.yid, self.yi, self.timeStep)
		self.yid    = yid_new
		rateyc      = ey * gr + self.yi
		rateyd_new  = (rateyc - self.ratey) / self.tr
		self.ratey  = integrate(rateyd_new, self.rateyd, self.ratey, self.timeStep)
		self.rateyd = rateyd_new
		betd_new    = -1 * (tiy * self.ratey + self.beta) / tiy
		self.beta   = integrate(betd_new, self.betd, self.beta, self.timeStep)
		self.betd   = betd_new

		# DERIVATIVE.
		axialAcc       = (thrust + CX * self.Q * self.refArea) / mass
		sideAcc        = (CY * self.Q * self.refArea) / mass
		normalAcc      = (CZ * self.Q * self.refArea) / mass
		self.specForce = npa([axialAcc, sideAcc, normalAcc])
		self.accEnu    = (self.specForce @ self.ENUtoFLU) + gravityVec

		# INTEGRATE STATE USING EULER METHOD.
		deltaPos    = self.timeStep * self.velEnu
		deltaVel    = self.timeStep * self.accEnu
		self.tof    += self.timeStep
		self.posEnu += deltaPos
		self.velEnu += deltaVel

		# END CHECK.
		self.missDistance = la.norm(FLUMslToPipRelPos)
		if self.missDistance < 5.0:
			self.lethality = endChecks.intercept
			self.go        = False
		elif self.posEnu[2] < 0.0:
			self.lethality = endChecks.groundCollision
			self.go        = False
		elif FLUMslToPipRelPos[0] < 0.0:
			self.lethality = endChecks.pointOfClosestApproachPassed
			self.go        = False
		elif np.isnan(np.sum(self.posEnu)):
			self.lethality = endChecks.notANumber
			self.go        = False
		elif self.tof > self.maxTime:
			self.lethality = endChecks.maxTimeExceeded
			self.go        = False

		# LOG DATA
		self.STATE = self.populateState()
		lf.writeData(self.STATE, self.logFile)

	def main(self):
		print("FIVE DOF INTERCEPTOR")
		lastTime = int(0)
		while self.go:
			self.fly()
			if np.floor(self.tof) == lastTime:
				print(f"TOF {self.tof:.1f} ENU {self.posEnu}")
				lastTime += 1
		wallClockEnd = time.time()
		print(f"TOF {self.tof:.4f} ENU {self.posEnu}")
		print(f"SIMULATION RUN TIME : {(wallClockEnd - self.wallClockStart):.2f}")
		print(f"SIMULATION RESULT   : {self.lethality.name}")
		print(f"MISS DISTANCE       : {self.missDistance:.2f}")



if __name__ == "__main__":
	x = fiveDofInterceptor(
		targetPos   = npa([4000.0, 4000.0, 2000.0]),
		targetVel   = npa([0.0, 0.0, 0.0]),
		launchElDeg = 20.0,
		launchAzDeg = 20.0,
		launchSpeed = 55.0,
		launchHgt   = 10.0
	)
	x.main()