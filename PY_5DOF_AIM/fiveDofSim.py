
# INCLUDED WITH PYTHON.
from   enum import Enum
import time
import copy

# PIP INSTALLED LIBRARIES.
import numpy as np
from   numpy import array as npa
from   numpy import linalg as la
np.set_printoptions(suppress=True, precision=2)

# UTILITY.
from utility.loadPickle                import loadpickle as lp
from utility.coordinateTransformations import FLIGHTPATH_TO_LOCAL_TM
from utility.coordinateTransformations import ORIENTATION_TO_LOCAL_TM
from utility.angles                    import returnAzAndElevation
from utility.angles                    import projection
from utility.ATM1976                   import ATM1976
from utility.trapezoidIntegrate        import integrate
from utility.unitVector                import unitvector
from utility                           import loggingFxns as lf

class endChecks(Enum):
	HIT    = 1
	FLIGHT = 0
	GROUND = -1
	POCA   = -2
	NAN    = -3
	TIME   = -4

class fiveDofInterceptor:

	def __init__(
		self,
		targetPos,
		targetVel,
		launchElDeg,
		launchAzDeg,
		launchSpeed,
		launchHgt,
		lineOfAttack
	):

		############################################################################
		#
		# AUTHOR - GRAHAM BEECH
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
		#
		############################################################################

		# INPUT.
		self.targetPos = targetPos # m
		self.targetVel = targetVel # m/s
		el             = np.radians(launchElDeg) # Convert input elevation to rads.
		az             = np.radians(launchAzDeg) # Convert input azimuth to rads.

		# SIM CONTROL.
		self.wallClockStart = time.time() # Real time start.
		self.dt             = (1.0 / 50.0) # seconds
		self.go             = True # Flag for termination.
		self.maxTime        = 200.0 # seconds

		# BODY.
		self.tof       = 0.0 # seconds
		self.specForce = np.zeros(3) # m/s^2
		self.alpha     = 0.0 # rads
		self.beta      = 0.0 # rads

		# INTEGRATION.
		self.PASS = int(0)
		self.P0   = np.zeros(3)
		self.V0   = np.zeros(3)
		self.P1   = np.zeros(3)
		self.V1   = np.zeros(3)
		self.A1   = np.zeros(3)
		self.P2   = np.zeros(3)
		self.V2   = np.zeros(3)
		self.A2   = np.zeros(3)
		self.P3   = np.zeros(3)
		self.V3   = np.zeros(3)
		self.A3   = np.zeros(3)
		self.P4   = np.zeros(3)
		self.V4   = np.zeros(3)
		self.A4   = np.zeros(3)

		# FRAME.
		self.enuToFlu = FLIGHTPATH_TO_LOCAL_TM(az, -1.0 * el) # nd
		self.posEnu   = npa([0.0, 0.0, launchHgt]) # m
		self.velEnu   = self.enuToFlu[0] * launchSpeed # m/s
		self.accEnu   = np.zeros(3) # m/s^2

		# INTERCEPTOR CONSTANTS.
		self.refArea        = 0.01767 # m^2
		self.nozzleExitArea = 0.00948 # m^2
		self.burnOut        = 3.02 # seconds
		self.burnOutMass    = 38.5 # kilograms

		# CONSTANTS.
		self.ambPress = 101325.0 # Ambient pressure in Pascals.

		# LOOK UP TABLES.
		self.lookUps = lp("PY_5DOF_AIM/lookUps.pickle")

		 # ATMOSPHERE.
		self.atm = ATM1976()
		self.atm.update(self.posEnu[2], launchSpeed)
		self.rho   = self.atm.rho # kg/m^3
		self.q     = self.atm.q # pa
		self.p     = self.atm.p # pa
		self.a     = self.atm.a # m/s
		self.g     = self.atm.g # m/s^2
		self.mach  = self.atm.mach # nd

		# GUIDANCE.
		self.midGuideLimit  = 50.0 # m/s^2
		self.lineOfAttack   = lineOfAttack # nd
		self.loaGain        = 1.5 # nd
		self.termGuideLimit = 250.0 # m/s^2
		self.proNavGain     = 4.0 # nd
		self.normComm       = 0.0 # m/s^2
		self.sideComm       = 0.0 # m/s^2

		# AUTOPILOT.
		self.ta   = 2 # Ratio of proportional/integral gain. nd
		self.tr   = 0.1 # Rate loop time constant. seconds
		self.gacp = 40.0 # Root locus gain of acceleration loop. rad/s^2

		# PITCH.
		self.xi     = 0.0 # Integral feedback. rad/s
		self.xid    = 0.0 # Integral feedback derivative. rad/s^2
		self.ratep  = 0.0 # Pitch rate. rad/s
		self.ratepd = 0.0 # Pitch rate dot. rad/s^2
		self.alpd   = 0.0 # Alpha dot. rad/s

		# YAW.
		self.yi     = 0.0 # Integral feedback. rad/s
		self.yid    = 0.0 # Integral feedback derivative. rad/s^2
		self.ratey  = 0.0 # Yaw rate. rad/s
		self.rateyd = 0.0 # Yaw rate dot. rad/s^2
		self.betd   = 0.0 # Beta dot. rad/s

		# END CHECK.
		self.lethality    = endChecks.FLIGHT
		self.missDistance = 0.0 # m

		# INITIALIZE LOG FILE AND WRITE HEADER.
		self.logFile = open("PY_5DOF_AIM/log.txt", "w")
		self.state   = self.populateState()
		lf.writeHeader(self.state, self.logFile)
		lf.writeData(self.state, self.logFile)

	def populateState(self):
		state = {
			"tof": self.tof,
			"posE": self.posEnu[0],
			"posN": self.posEnu[1],
			"posU": self.posEnu[2],
			"tgtE": self.targetPos[0],
			"tgtN": self.targetPos[1],
			"tgtU": self.targetPos[2],
			"normComm": self.normComm,
			"normAch": self.specForce[2],
			"sideComm": self.sideComm,
			"sideAch": self.specForce[1]
		}
		return state

	def fly(self):

		# UPDATE TARGET.
		self.targetPos += (self.dt * self.targetVel) # m

		# ENU TO FLU MATRIX.
		velAz, velEl  = returnAzAndElevation(self.velEnu) # rads
		velFrame      = FLIGHTPATH_TO_LOCAL_TM(velAz, -velEl) # nd
		aeroFrame     = ORIENTATION_TO_LOCAL_TM(0.0, self.alpha, self.beta) # nd
		self.enuToFlu = aeroFrame @ velFrame # nd

		# ATMOSPHERE.
		spd       = la.norm(self.velEnu) # m/s
		self.atm.update(self.posEnu[2], spd)
		self.rho  = self.atm.rho # kg/m^3
		self.q    = self.atm.q # pa
		self.p    = self.atm.p # pa
		self.a    = self.atm.a # m/s
		self.g    = self.atm.g # m/s^2
		self.mach = self.atm.mach # nd
		localGrav = npa([0, 0, -1.0 * self.g]) # m/s^2
		bodyGrav  = self.enuToFlu @ localGrav # m/s^2

		# KINEMATIC TRUTH SEEKER.
		fluMslToTgt   = self.enuToFlu @ (self.targetPos - self.posEnu) # m
		fluMslToTgtU  = unitvector(fluMslToTgt) # nd
		fluMslToTgtM  = la.norm(fluMslToTgt) # m
		closingVel    = self.enuToFlu @ (self.targetVel - self.velEnu) # m/s
		closingSpeed  = la.norm(closingVel) # m/s
		tgo           = fluMslToTgtM / closingSpeed # seconds

		# PROPORTIONAL GUIDANCE.
		if tgo < 4.0:
			T1            = np.cross(fluMslToTgt, closingVel)
			T2            = np.dot(fluMslToTgt, fluMslToTgt)
			omega         = T1 / T2 # rad/s
			T3            = -1.0 * self.proNavGain * closingSpeed * fluMslToTgtU
			comm          = np.cross(T3, omega) # m/s^2
			self.normComm = comm[2] # m/s^2
			self.sideComm = comm[1] # m/s^2
			aMag          = np.sqrt(self.normComm ** 2 + self.sideComm ** 2) # m/s^2
			trigRatio     = np.arctan2(self.normComm, self.sideComm) # nd
			if aMag > self.termGuideLimit:
				aMag = self.termGuideLimit # m/s^2
			self.normComm = aMag * np.sin(trigRatio) # m/s^2
			self.sideComm = aMag * np.cos(trigRatio) # m/s^2

		# LINE OF ATTACK GUIDANCE.
		else:
			losVel        = projection(fluMslToTgtU, closingVel) # m/s
			loaVel        = projection(self.lineOfAttack, closingVel) # m/s
			G = 1 - np.exp(-0.001 * fluMslToTgtM) # nd
			self.normComm = self.loaGain * (losVel[2] + G * loaVel[2]) # m/s^2
			self.sideComm = self.loaGain * (losVel[1] + G * loaVel[1]) # m/s^2
			aMag          = np.sqrt(self.normComm ** 2 + self.sideComm ** 2) # m/s^2
			trigRatio     = np.arctan2(self.normComm, self.sideComm) # nd
			if aMag > self.midGuideLimit:
				aMag = self.midGuideLimit # m/s^2
			self.normComm = aMag * np.sin(trigRatio) # m/s^2
			self.sideComm = aMag * np.cos(trigRatio) # m/s^2

		# AEROBALLISTIC ANGLES.
		angleOfAttack    = np.arccos(np.cos(self.alpha) * np.cos(self.beta)) # rads
		angleOfAttackDeg = np.degrees(angleOfAttack) # deg
		phiPrime         = np.arctan2(np.tan(self.beta), np.sin(self.alpha)) # rads

		# LOOK UPS.
		CD     = None # nd
		thrust = None # newtons
		mass   = None # kg
		if self.tof <= self.burnOut:
			mass      = self.lookUps["MASS (KG)"](self.tof) # kg
			CD        = self.lookUps["CD MOTOR ON"]\
				(self.mach, angleOfAttackDeg)[0] # nd
			vacThrust = self.lookUps["THRUST (NEWTONS)"](self.tof) # newtons
			thrust    = vacThrust + \
				(self.ambPress - self.p) * self.nozzleExitArea # newtons
		else:
			mass   = self.burnOutMass # kg
			CD     = self.lookUps["CD MOTOR OFF"]\
				(self.mach, angleOfAttackDeg)[0] # nd
			thrust = 0.0 # newtons
		CL = self.lookUps["CL"](self.mach, angleOfAttackDeg)[0] # nd

		# AERODYNAMICS.
		cosAlpha    = np.cos(self.alpha) # rads
		sinAlpha    = np.sin(self.alpha) # rads
		absAlphaDeg = np.abs(np.degrees(self.alpha)) # deg
		absBetaDeg  = np.abs(np.degrees(self.beta)) # deg
		CX          = -1 * (CD * cosAlpha - CL * sinAlpha) # nd
		CT          = CD * sinAlpha + CL * cosAlpha # nd
		CZ          = -1 * CT * np.cos(phiPrime) # nd
		CY          = -1 * CT * np.sin(phiPrime) # nd
		
		# AERODYNAMIC DERIVATIVES.
		# CNA and CYB are multiplied by 57.3 to make them dimensionless.
		# (1/deg)*deg == 1
		CNA = None # nd
		CYB = None # nd
		if absAlphaDeg < 10:
			CNA = 57.3 * (0.123 + 0.013 * absAlphaDeg) # nd
		else:
			CNA = 57.3 * (0.06 * (absAlphaDeg ** 0.625)) # nd
		if absBetaDeg < 10:
			CYB = 57.3 * (0.123 + 0.013 * absBetaDeg) # nd
		else:
			CYB = 57.3 * (0.06 * (absBetaDeg ** 0.625)) # nd

		# AUTOPILOT.
		if self.PASS == 0:
			
			# PITCH AUTOPILOT.
			tip         = spd * mass / (thrust + self.q * \
				self.refArea * np.abs(CNA))
			fspz        = (self.q * self.refArea * CZ / mass) + bodyGrav[2]
			gr          = self.gacp * tip * self.tr / spd
			gi          = gr / self.ta
			abez        = self.normComm
			ep          = abez - fspz
			xid_new     = gi * ep
			self.xi     = integrate(xid_new, self.xid, self.xi, self.dt)
			self.xid    = xid_new
			ratepc      = -1 * (ep * gr + self.xi)
			ratepd_new  = (ratepc - self.ratep) / self.tr
			self.ratep  = integrate(ratepd_new, self.ratepd, self.ratep, self.dt)
			self.ratepd = ratepd_new
			alpd_new    = (tip * self.ratep - self.alpha) / tip
			self.alpha  = integrate(alpd_new, self.alpd, self.alpha, self.dt)
			self.alpd   = alpd_new

			# YAW AUTOPILOT.
			tiy         = spd * mass / (thrust + self.q * \
				self.refArea * np.abs(CYB))
			fspy        = self.q * self.refArea * CY / mass
			gr          = self.gacp * tiy * self.tr / spd
			gi          = gr / self.ta
			abey        = self.sideComm
			ey          = abey - fspy
			yid_new     = gi * ey
			self.yi     = integrate(yid_new, self.yid, self.yi, self.dt)
			self.yid    = yid_new
			rateyc      = ey * gr + self.yi
			rateyd_new  = (rateyc - self.ratey) / self.tr
			self.ratey  = integrate(rateyd_new, self.rateyd, self.ratey, self.dt)
			self.rateyd = rateyd_new
			betd_new    = -1 * (tiy * self.ratey + self.beta) / tiy
			self.beta   = integrate(betd_new, self.betd, self.beta, self.dt)
			self.betd   = betd_new

		# DERIVATIVE.
		axialAcc       = (thrust + CX * self.q * self.refArea) / mass # m/s^2
		sideAcc        = (CY * self.q * self.refArea) / mass # m/s^2
		normalAcc      = (CZ * self.q * self.refArea) / mass # m/s^2
		self.specForce = npa([axialAcc, sideAcc, normalAcc]) # m/s^2
		self.accEnu    = (self.specForce @ self.enuToFlu) + localGrav # m/s^2

		# RK4 INTEGRATION.
		if self.PASS == 0:

			# END CHECK.
			self.missDistance = la.norm(fluMslToTgt) # m
			if self.missDistance < 5.0:
				self.lethality = endChecks.HIT
				self.go        = False
			elif self.posEnu[2] < 0.0:
				self.lethality = endChecks.GROUND
				self.go        = False
			elif fluMslToTgt[0] < 0.0:
				self.lethality = endChecks.POCA
				self.go        = False
			elif np.isnan(np.sum(self.posEnu)):
				self.lethality = endChecks.NAN
				self.go        = False
			elif self.tof > self.maxTime:
				self.lethality = endChecks.TIME
				self.go        = False

			# LOG DATA
			self.state = self.populateState()
			lf.writeData(self.state, self.logFile)

			# UPDATE PASS.
			self.PASS += 1

			# BEGIN INTEGRATION.
			self.P0     = copy.deepcopy(self.posEnu)
			self.V0     = copy.deepcopy(self.velEnu)
			self.V1     = copy.deepcopy(self.velEnu)
			self.A1     = copy.deepcopy(self.accEnu)
			self.tof    += (self.dt / 2.0)
			self.posEnu = self.P0 + self.V1 * (self.dt / 2.0)
			self.velEnu = self.V0 + self.A1 * (self.dt / 2.0)

		elif self.PASS == 1:

			# UPDATE PASS.
			self.PASS += 1

			# INTEGRATION.
			self.V2     = copy.deepcopy(self.velEnu)
			self.A2     = copy.deepcopy(self.accEnu)
			self.posEnu = self.P0 + self.V2 * (self.dt / 2.0)
			self.velEnu = self.V0 + self.A2 * (self.dt / 2.0)

		elif self.PASS == 2:

			# UPDATE PASS.
			self.PASS += 1

			# INTEGRATION.
			self.V3     = copy.deepcopy(self.velEnu)
			self.A3     = copy.deepcopy(self.accEnu)
			self.tof    += (self.dt / 2.0)
			self.posEnu = self.P0 + self.V3 * self.dt
			self.velEnu = self.V0 + self.A3 * self.dt

		elif self.PASS == 3:

			# UPDATE PASS.
			self.PASS = 0

			# INTEGRATION.
			self.V4     = copy.deepcopy(self.velEnu)
			self.A4     = copy.deepcopy(self.accEnu)
			self.posEnu = self.P0 + (self.V1 + self.V2 * 2 + self.V3 * 2 + self.V4)\
				 * (self.dt / 6.0)
			self.velEnu = self.V0 + (self.A1 + self.A2 * 2 + self.A3 * 2 + self.A4)\
				 * (self.dt / 6.0)

	def main(self):
		print("FIVE DOF INTERCEPTOR")
		lastTime = int(0)
		while self.go:
			self.fly()
			if np.floor(self.tof) == lastTime:
				print(f"TOF {self.tof:.0f} ENU {self.posEnu}")
				lastTime += 1
		wallClockEnd = time.time()
		print(f"TOF {self.tof:.4f} ENU {self.posEnu}")
		print(f"SIMULATION RUN TIME : {(wallClockEnd - self.wallClockStart):.2f}")
		print(f"SIMULATION RESULT   : {self.lethality.name}")
		print(f"MISS DISTANCE       : {self.missDistance:.2f}")



if __name__ == "__main__":
	x = fiveDofInterceptor(
		targetPos    = npa([4000.0, 4000.0, 3000.0]),
		targetVel    = npa([0.0, 0.0, 0.0]),
		launchElDeg  = 40.0,
		launchAzDeg  = 40.0,
		launchSpeed  = 55.0,
		launchHgt    = 10.0,
		lineOfAttack = npa([0.8, 0.8, 0.3])
	)
	x.main()



















































