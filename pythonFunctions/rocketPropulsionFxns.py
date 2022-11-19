import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use("WebAgg")
import pandas as pd
from numpy import array as npa
from numpy import linalg as la
from dictFxns import printValuesInADictionary
import copy

"""

Functions developed while in MAE 540 Rocket Propulsion at The
University of Alabama in Huntsville. Taught by Dr. Robert
Frederick.

"""

# Constants.
NauticalMilesToFeet = 6076 # Feet.
MilesToFeet = 5280 # Feet.
R_E = 3963 # Miles.
Mu_E = 1.4076e16 # Feet cubed per second squared.
SecondsInADay = 86400 # Seconds.

def linearInterpolation(x, xx, yy):
	lowIndex = None
	highIndex = None
	index = -1
	for number in xx:
		index += 1
		if number > x:
			if highIndex == None:
				highIndex = index
			elif number < xx[highIndex]:
				highIndex = index
		if number < x:
			if lowIndex == None:
				lowIndex = index
			elif number > xx[lowIndex]:
				lowIndex = index
	if lowIndex == None:
		lowIndex = 0
	if highIndex == None:
		highIndex = -1
	# print(x, xx[lowIndex], xx[highIndex], yy[lowIndex], yy[highIndex])
	y = yy[lowIndex] + ((x - xx[lowIndex]) * ((yy[highIndex] - yy[lowIndex]) / (xx[highIndex] - xx[lowIndex])))
	return y

# Rocket Propulsion Textbook 2.11
def Textbook_2_11(ALTITUDE, ORBIT_LATITUDE):

	US_LAT = np.radians(28.5) # Radians
	FR_LAT = np.radians(3) # Radians

	Vc = np.sqrt( Mu_E / ( (R_E * MilesToFeet) + (ALTITUDE * NauticalMilesToFeet) ) )
	print("CIRCULAR VELOCITY", Vc)

	FR_Vpc = 2 * Vc * np.sin((np.abs(FR_LAT - ORBIT_LATITUDE)) / 2)
	print("FRANCE PLANE CHANGE VELOCITY", FR_Vpc)

	FR_Vsurf = (2 * np.pi * (R_E * MilesToFeet) * np.cos(FR_LAT)) / SecondsInADay
	print("FRANCE SURFACE VELOCITY", FR_Vsurf)

	FR_DELTA_V = Vc - FR_Vsurf + FR_Vpc
	print("FRANCE DELTA VELOCITY", FR_DELTA_V)

	US_Vpc = 2 * Vc * np.sin((np.abs(US_LAT - ORBIT_LATITUDE)) / 2)
	print("US PLANE CHANGE VELOCITY", US_Vpc)

	US_Vsurf = (2 * np.pi * (R_E * MilesToFeet) * np.cos(US_LAT)) / SecondsInADay
	print("US SURFACE VELOCITY", US_Vsurf)

	US_DELTA_V = Vc - US_Vsurf + US_Vpc
	print("US DELTA VELOCITY", US_DELTA_V)

	print("\n")

# Rocket Propulsion Textbook 3.3
def Textbook_3_3(
	iPropMassFraction,
	iPayloadMass,
	iDeltaVIdeal,
	iSpecificImpulse
):

	propMassFraction = iPropMassFraction
	payloadMass = iPayloadMass # lbm
	deltaV = iDeltaVIdeal # feet per sec
	ISP = iSpecificImpulse # seconds
	grav = 32.2 # feet per second squared

	MR = np.exp(deltaV / (ISP * grav))
	propMass = payloadMass * (MR - 1) / (MR - ((MR - 1) / propMassFraction))
	inertMass = (propMass - propMass * propMassFraction) / propMassFraction

	print(f"PROP MASS {propMass}; INERT MASS {inertMass}")

	return propMass, inertMass

# Rocket Propulsion Textbook 3.4
def Textbook_3_4(
	finalAltitude, # Meters.
	gravity, # Meters per second squared.
	ISP, # Seconds.
	M0, # Kilograms.
	MF # Kilograms.
):
	
	TEMP1 = finalAltitude - (gravity / 2.0) * (ISP ** 2) * ((np.log(M0 / MF)) ** 2)
	TEMP2 = gravity * ISP
	TEMP3 = TEMP2 * ((MF / (M0 - MF)) * np.log(MF / M0) + 1)
	TEMP4 = TEMP2 * np.log(M0 / MF)
	TEMP5 = TEMP1 / (TEMP3 - TEMP4)
	return TEMP5

class Textbook_4_1:

	# sigma is area ratio
	# gamma is the specific ratio
	# pascals, pascals, kelvin, kg / kg * molK, nd, cm, cm
	def __init__(self, pa, pc, Tc, molarMass, gamma, exitDiam, throatDiam, chamberDiam):
		self.pa = pa
		self.pc = pc
		self.Tc = Tc
		self.molarMass = molarMass
		self.gamma = gamma
		self.exitDiam = exitDiam
		self.throatDiam = throatDiam
		self.chamberDiam = chamberDiam

	@staticmethod
	def areaCircle(diam):
		return (np.pi * diam * diam) / 4.0

	@staticmethod
	def nozzleAreaRatio(mach, gamma):
		t1 = (1 / mach)
		t2 = 2 + (gamma - 1) * mach * mach
		t3 = gamma + 1
		t4 = (gamma + 1) / (2.0 * (gamma - 1))
		t = t1 * ((t2 / t3) ** t4)
		return t

	@staticmethod
	def findMach(low, high, gamma, sigma):
		l = low
		h = high
		index = 0
		while True:
			index += 1
			machGuess = (l + h) / 2.0
			x = Textbook_4_1.nozzleAreaRatio(machGuess, gamma)
			check = x / sigma
			print(f"REPORT\n  INDEX {index}\n  LOW {l}\n  HIGH {h}\n  MACH {machGuess:.2f}\n  CALC {x:.2f}\n  SIGMA {sigma:.2f}\n  CHECK {check:.2f}\n")
			if 0.98 < check < 1.02:
				print(f"Solution found: {machGuess}\n")
				break
			elif check < 0.98:
				l = machGuess
			elif check > 1.02:
				h = machGuess
			if index == 20:
				print(f"Solution does not converge.\n")
				break
		return machGuess

	# uses newton raphson method for function with more than one root
	@staticmethod
	def DrRobertFrederick_findMach(initialMachGuess, gamma, sigma):
		StopCriteria = 0.000001
		EA = StopCriteria * 1.1
		AM2 = copy.deepcopy(initialMachGuess)
		index = 0
		while EA > StopCriteria and index < 100:
			index += 1
			AFUN = (2.0 + (gamma - 1) * AM2 * AM2) / (gamma + 1.0)
			BFUN = (gamma + 1.0) / (2.0 * (gamma - 1.0))
			CFUN = 1.0 / AFUN
			DFUN = 1.0 / (AM2 * AM2)
			DERFUN = (AFUN ** BFUN) * (CFUN - DFUN)
			FUNFUN = (1.0 / AM2) * (AFUN ** BFUN) - sigma
			AMOLD = AM2
			AM2 -= (FUNFUN / DERFUN)
			EA = np.abs((AM2 - AMOLD) / AM2) * 100.0
		return AM2

	@staticmethod
	def isentropicPressureRatio(gamma, mach):
		t1 = 1
		t2 = (gamma - 1) / 2.0
		t3 = mach * mach
		t4 = gamma / (gamma - 1)
		ret = (t1 + t2 * t3) ** t4
		return ret

	@staticmethod
	def vacuumForceCoefficient(gamma, sigma, mach):
		t1 = 2 * gamma * gamma / (gamma - 1)
		t2 = 2 / (gamma + 1)
		t3 = (gamma + 1) / (gamma - 1)
		t4 = 1
		t5 = 1.0 / (Textbook_4_1.isentropicPressureRatio(gamma, mach))
		t6 = (gamma - 1) / gamma
		t7 = (1.0 / (Textbook_4_1.isentropicPressureRatio(gamma, mach))) * sigma
		ret = np.sqrt(t1 * (t2 ** t3) * (t4 - t5 ** t6)) + t7
		return ret

	@staticmethod
	def forceCoefficient(cfv, pa, pc, sigma):
		return cfv - (pa / pc) * sigma

	@staticmethod
	def calcForce(cf, pc, At):
		return cf * pc * At

	@staticmethod
	def charVelocity(Tc, MM, gamma):
		Ru = 8317 # N * m / (kg * kmolK)
		t1 = Ru * Tc / (gamma * MM)
		t2 = 2 / (gamma + 1)
		t3 = (-1.0 * (gamma + 1)) / (2 * (gamma - 1))
		ret = np.sqrt(t1) * (t2 ** t3)
		return ret

	@staticmethod
	def charVelocityImperial(Tc, MM, gamma): # rankine, lbm / lbm-mole, nd
		Ru = 1543
		t1 = (Ru * Tc / (gamma * MM)) * 32.2
		t2 = 2 / (gamma + 1)
		t3 = (-1.0 * (gamma + 1)) / (2 * (gamma - 1))
		ret = np.sqrt(t1) * (t2 ** t3)
		return ret

	@staticmethod
	def calcIsp(cf, cstar, g):
		return (cf * cstar) / g

	@staticmethod
	def exitVelocity(F, cStar, Pc, throatArea):
		return (F * cStar) / (Pc * throatArea)

	@staticmethod
	def speedOfSound(gamma, T, MM):
		Ru = 8317 # N * m / kg * mol * Kelvin
		return np.sqrt((gamma * Ru * T) / MM)

	@staticmethod
	def isentropicTemperatureRatio(gamma, mach):
		t1 = 1
		t2 = (gamma - 1) / 2.0
		t3 = mach * mach
		ret = t1 + t2 * t3
		return ret

	@staticmethod
	def exhaustVelocity(gamma, MM, T1, Te, v1):
		Ru = 8317 # N * m / kg * mol * Kelvin
		t1 = 2 * gamma * Ru
		t2 = MM * (gamma - 1)
		t3 = T1 - Te
		t4 = v1 ** 2
		ret = np.sqrt(((t1 * t3) / t2) + t4)
		return ret

	def main(self):

		pa = self.pa # pascals
		pc = self.pc # pascals
		Tc = self.Tc # kelvin
		molarMass = self.molarMass # kg / kg * molK
		gamma = self.gamma # nd
		exitDiam = self.exitDiam # cm
		throatDiam = self.throatDiam # cm
		chamberDiam = self.chamberDiam # cm

		exitArea = Textbook_4_1.areaCircle(exitDiam) # cm^2
		throatArea = Textbook_4_1.areaCircle(throatDiam) # cm^2
		sigmaExit =  exitArea / throatArea # nd
		exitMach = Textbook_4_1.findMach(0, 20, gamma, sigmaExit) # nd
		pRatio = Textbook_4_1.isentropicPressureRatio(gamma, exitMach) # nd
		cfv = Textbook_4_1.vacuumForceCoefficient(gamma, sigmaExit, exitMach) # nd
		cf = Textbook_4_1.forceCoefficient(cfv, pa, pc, sigmaExit)
		throatAreaM2 = throatArea * (1.0 / 100.0) * (1.0 / 100.0)
		thrust = Textbook_4_1.calcForce(cf, pc, throatAreaM2)
		cStar = Textbook_4_1.charVelocity(Tc, molarMass, gamma)
		Isp = Textbook_4_1.calcIsp(cf, cStar, 9.81)
		PeOverPc = 1.0 / pRatio
		PaOverPc = pa / pc
		flag = None
		check = (PeOverPc / PaOverPc)
		if 0.98 < check < 1.2:
			flag = True
		else:
			flag = False
		exitVelocity = Textbook_4_1.exitVelocity(thrust, cStar, pc, throatAreaM2)
		chamberArea = Textbook_4_1.areaCircle(chamberDiam)
		sigmaChamber = chamberArea / throatArea
		chamberMach = Textbook_4_1.DrRobertFrederick_findMach(0.2, gamma, sigmaChamber)
		a = Textbook_4_1.speedOfSound(gamma, Tc, molarMass)
		v1 = a * chamberMach
		tempRatio = Textbook_4_1.isentropicTemperatureRatio(gamma, exitMach)
		Te = Tc * (1.0 / tempRatio)
		exhaustVelocity = Textbook_4_1.exhaustVelocity(gamma, molarMass, Tc, Te, v1)
		diff = np.abs(exitVelocity - exhaustVelocity)
		percDiff = 100 * (diff / exhaustVelocity)

		data = {
			"exitArea": exitArea,
			"throatArea": throatArea,
			"sigmaExit": sigmaExit,
			"exitMach": exitMach,
			"pRatio": pRatio,
			"cfv": cfv,
			"cf": cf,
			"throatAreaM2": throatAreaM2,
			"thrust": thrust,
			"cStar": cStar,
			"Isp": Isp,
			"perfectlyExpanded": flag,
			"exitVelocity": exitVelocity,
			"chamberArea": chamberArea,
			"sigmaChamber": sigmaChamber,
			"chamberMach": chamberMach,
			"speedOfSound": a,
			"v1": v1,
			"exitTemp": Te,
			"exhaustVelocity": exhaustVelocity,
			"percentDiffStagnantVsNot": percDiff
		}

		return data

def SP04A1():

	# Utility #
	def slopeOfTwoPoints(p1, p2):
		t1 = p2[1] - p1[1]
		t2 = p2[0] - p1[0]
		ret = t1 / t2
		return ret

	def printEqnOfALine(coeff):
		print(f"y = {coeff[0]}x + {coeff[1]}")

	def radius(x, coeff):
		ret = coeff[0] * x + coeff[1]
		return ret

	def area(radius):
		area = np.pi * radius * radius
		return area

	def nozzleAreaRatio(mach, gamma):
		t1 = (1 / mach)
		t2 = 2 + (gamma - 1) * mach * mach
		t3 = gamma + 1
		t = t1 * (t2 / t3) ** ((gamma + 1) / (2.0 * (gamma - 1)))
		return t

	def findMach(gamma, sigma):
		low = 0
		high = 10
		index = 0
		while True:
			index += 1
			machGuess = (low + high) / 2.0
			x = nozzleAreaRatio(machGuess, gamma)
			check = x / sigma
			# print(f"REPORT\n  INDEX {index}\n  LOW {low}\n  HIGH {high}\n  MACH {machGuess:.2f}\n  CHECK {check:.2f}\n")
			if 0.98 < check < 1.02:
				# print(f"Solution found: {machGuess}\n")
				break
			elif check < 0.98:
				low = machGuess
			elif check > 1.02:
				high = machGuess
			if index == 20:
				break
		return machGuess

	# uses newton raphson method for function with two roots
	def RobertFrederick_findMach(initialMachGuess, gamma, sigma):
		StopCriteria = 0.000001
		EA = StopCriteria * 1.1
		AM2 = copy.deepcopy(initialMachGuess)
		index = 0
		while EA > StopCriteria and index < 100:
			index += 1
			AFUN = (2.0 + (gamma - 1) * AM2 * AM2) / (gamma + 1.0)
			BFUN = (gamma + 1.0) / (2.0 * (gamma - 1.0))
			CFUN = 1.0 / AFUN
			DFUN = 1.0 / (AM2 * AM2)
			DERFUN = (AFUN ** BFUN) * (CFUN - DFUN)
			FUNFUN = (1.0 / AM2) * (AFUN ** BFUN) - sigma
			AMOLD = AM2
			AM2 -= (FUNFUN / DERFUN)
			EA = np.abs((AM2 - AMOLD) / AM2) * 100.0
		return AM2

	def isentropicPressureRatio(gamma, mach):
		t1 = 1
		t2 = (gamma - 1) / 2.0
		t3 = mach * mach
		t4 = gamma / (gamma - 1)
		ret = (t1 + t2 * t3) ** t4
		return ret

	def isentropicTemperatureRatio(gamma, mach):
		t1 = 1
		t2 = (gamma - 1) / 2.0
		t3 = mach * mach
		ret = t1 + t2 * t3
		return ret

	# Defined points.
	p1 = [-5, 5]
	p2 = [-3, 5]
	p3 = [0, 2]
	p4 = [3, 6]

	# Slopes and y-intercepts.
	lineOneCoeff = [slopeOfTwoPoints(p1, p2), 5]
	lineTwoCoeff = [slopeOfTwoPoints(p2, p3), 2.0]
	lineThreeCoeff = [slopeOfTwoPoints(p3, p4), 2.0]

	# # Report.
	# print("ROCKET WALL EQNS")
	# printEqnOfALine(lineOneCoeff)
	# printEqnOfALine(lineTwoCoeff)
	# printEqnOfALine(lineThreeCoeff)

	# Constants.
	INC = 0.1 # Inches.
	THROAT_AREA = np.pi * 2 * 2
	PcOne = 1000 # psi
	PcTwo = 500 # psi
	Tc = 5000 # Farenheit
	gamma = 1.3

	# Data.
	X = []
	AREA = []
	RADIUS = []
	MACH = []
	TEMPERATURE = []
	PRESSURE1 = []
	PRESSURE2 = []

	# Function.
	start = p1[0]
	stop = p2[0]
	while start < stop:

		# Calculations.
		rocketRadius = radius(start, lineOneCoeff)
		rocketArea = area(rocketRadius)
		areaRatio = rocketArea / THROAT_AREA
		# rocketMach = RobertFrederick_findMach(0.5, gamma, areaRatio)
		rocketMach = 0.1
		temperature = Tc
		pressureOne = PcOne
		pressureTwo = PcTwo

		# Data.
		X.append(start)
		RADIUS.append(rocketRadius)
		AREA.append(rocketArea)
		MACH.append(rocketMach)
		TEMPERATURE.append(temperature)
		PRESSURE1.append(pressureOne)
		PRESSURE2.append(pressureTwo)

		# Move along the line.
		start += INC

	stop = p3[0]
	while start < stop:

		# Calculations.
		rocketRadius = radius(start, lineTwoCoeff)
		rocketArea = area(rocketRadius)
		areaRatio = rocketArea / THROAT_AREA
		rocketMach = RobertFrederick_findMach(0.07, gamma, areaRatio)
		tempRatio = isentropicTemperatureRatio(gamma, rocketMach)
		temperature = Tc / tempRatio
		pressureRatio = isentropicPressureRatio(gamma, rocketMach)
		pressureOne = PcOne / pressureRatio
		pressureTwo = PcTwo / pressureRatio

		# Data.
		X.append(start)
		RADIUS.append(rocketRadius)
		AREA.append(rocketArea)
		MACH.append(rocketMach)
		TEMPERATURE.append(temperature)
		PRESSURE1.append(pressureOne)
		PRESSURE2.append(pressureTwo)

		# Move along the line.
		start += INC

	stop = p4[0]
	while start < stop:

		# Calculations.
		rocketRadius = radius(start, lineThreeCoeff)
		rocketArea = area(rocketRadius)
		areaRatio = rocketArea / THROAT_AREA
		rocketMach = findMach(gamma, areaRatio)
		tempRatio = isentropicTemperatureRatio(gamma, rocketMach)
		temperature = Tc / tempRatio
		pressureRatio = isentropicPressureRatio(gamma, rocketMach)
		pressureOne = PcOne / pressureRatio
		pressureTwo = PcTwo / pressureRatio

		# Data.
		X.append(start)
		RADIUS.append(rocketRadius)
		AREA.append(rocketArea)
		MACH.append(rocketMach)
		TEMPERATURE.append(temperature)
		PRESSURE1.append(pressureOne)
		PRESSURE2.append(pressureTwo)

		# Move along the line.
		start += INC

	figure = plt.figure()

	windowTwo = figure.add_subplot(221)
	windowTwo.plot(X, RADIUS, color="b", label="ROCKET RADIUS ALONG THE WALL (INCHES)")
	windowTwo.set_xlabel("INCHES")
	windowTwo.set_ylabel("INCHES")
	windowTwo.legend(fontsize="xx-small")

	windowThree = figure.add_subplot(222)
	windowThree.plot(X, MACH, color="b", label="ROCKET MACH ALONG THE WALL")
	windowThree.set_xlabel("INCHES")
	windowThree.set_ylabel("ND")
	windowThree.legend(fontsize="xx-small")

	windowFour = figure.add_subplot(223)
	windowFour.plot(X, TEMPERATURE, color="b", label="TEMPERATURE ALONG THE WALL (F)")
	windowFour.set_xlabel("INCHES")
	windowFour.set_ylabel("F")
	windowFour.legend(fontsize="xx-small")

	windowFive = figure.add_subplot(224)
	windowFive.plot(X, PRESSURE1, color="b", label="PRESSURE ALONG THE WALL AT 1000 PSI CHAMBER PRESSURE (PSI)")
	windowFive.plot(X, PRESSURE2, color="r", label="PRESSURE ALONG THE WALL AT 500 PSI CHAMBER PRESSURE (PSI)")
	windowFive.set_xlabel("INCHES")
	windowFive.set_ylabel("PSI")
	windowFive.legend(fontsize="xx-small")

	plt.show()

def SP04_B():

	gamma = 1.2
	sigmas = np.linspace(1.01, 100, 10000)
	PcOverPa_iters = [2, 4, 10, 25, 75, 150, 500, 1000, 1000000000000]
	# PcOverPa_iters = [2]

	# uses newton raphson method for function with more than one root
	def RobertFrederick_findMach(initialMachGuess, gamma, sigma):
		StopCriteria = 0.000001
		EA = StopCriteria * 1.1
		AM2 = copy.deepcopy(initialMachGuess)
		index = 0
		while EA > StopCriteria and index < 100:
			index += 1
			AFUN = (2.0 + (gamma - 1) * AM2 * AM2) / (gamma + 1.0)
			BFUN = (gamma + 1.0) / (2.0 * (gamma - 1.0))
			CFUN = 1.0 / AFUN
			DFUN = 1.0 / (AM2 * AM2)
			DERFUN = (AFUN ** BFUN) * (CFUN - DFUN)
			FUNFUN = (1.0 / AM2) * (AFUN ** BFUN) - sigma
			AMOLD = AM2
			AM2 -= (FUNFUN / DERFUN)
			EA = np.abs((AM2 - AMOLD) / AM2) * 100.0
		return AM2

	def vacuumForceCoefficient(gamma, sigma, PcOverPe):
		t1 = 2 * gamma * gamma / (gamma - 1)
		t2 = 2 / (gamma + 1)
		t3 = (gamma + 1) / (gamma - 1)
		t4 = 1
		t5 = 1.0 / PcOverPe
		t6 = (gamma - 1) / gamma
		t7 = (1.0 / PcOverPe) * sigma
		ret = np.sqrt(t1 * (t2 ** t3) * (t4 - t5 ** t6)) + t7
		return ret

	def forceCoefficient(cfv, PcOverPa, sigma):
		ret = (cfv - (1.0 / PcOverPa) * sigma)
		return ret

	def nozzleAreaRatio(mach, gamma):
		t1 = (1 / mach)
		t2 = 2 + (gamma - 1) * mach * mach
		t3 = gamma + 1
		t4 = (gamma + 1) / (2.0 * (gamma - 1))
		t = t1 * ((t2 / t3) ** t4)
		return t

	def findMach(low, high, gamma, sigma):
		l = low
		h = high
		index = 0
		while True:
			index += 1
			machGuess = (l + h) / 2.0
			x = nozzleAreaRatio(machGuess, gamma)
			check = x / sigma
			# print(f"REPORT\n  INDEX {index}\n  LOW {l}\n  HIGH {h}\n  MACH {machGuess:.2f}\n  CALC {x:.2f}\n  SIGMA {sigma:.2f}\n  CHECK {check:.2f}\n")
			if 0.99 < check < 1.01:
				# print(f"Solution found: {machGuess}\n")
				break
			elif check < 0.98:
				l = machGuess
			elif check > 1.02:
				h = machGuess
			if index == 20:
				# print(f"Solution does not converge.\n")
				break
		return machGuess

	def isentropicPressureRatio(gamma, mach):
		t1 = 1
		t2 = (gamma - 1) / 2.0
		t3 = mach * mach
		t4 = gamma / (gamma - 1)
		ret = (t1 + t2 * t3) ** t4
		return ret

	def lineOfSeperation(sigma):
		t1 = -1.0 * 0.0445 * (np.log(sigma) ** 2)
		t2 = 0.5324 * np.log(sigma)
		t3 = 0.1843
		ret = t1 + t2 + t3
		return ret

	lineOfMaxCfsX = []
	lineOfMaxCfsY = []
	data = []
	for index, n in enumerate(PcOverPa_iters):
		x = []
		y = []
		flag = 0
		for index, sigma in enumerate(sigmas):
			PcOverPa = n
			Pc = PcOverPa * 14.7
			Me = None
			if n == 2:
				Me = RobertFrederick_findMach(2.0, gamma, sigma)
			else:
				Me = findMach(0, 5, gamma, sigma)
			PcOverPe = isentropicPressureRatio(gamma, Me)
			Pe = (1.0 / PcOverPe) * Pc
			check = np.abs(Pe - 14.7)
			cfv = vacuumForceCoefficient(gamma, sigma, PcOverPe)
			cf = forceCoefficient(cfv, PcOverPa, sigma)
			pd = (check / 14.7) * 100.0
			if pd < 5:
				if flag == 0:
					lineOfMaxCfsX.append(sigma)
					lineOfMaxCfsY.append(cf)
					flag = 1
				else:
					pass
			x.append(sigma)
			y.append(cf)
		if n == 1000000000000:
			dp = [x, y, "INF"]
		else:
			dp = [x, y, f"{n}"]
		data.append(dp)

	x = []
	y = []
	for index, sigma in enumerate(sigmas):
		n = lineOfSeperation(sigma)
		x.append(sigma)
		y.append(n)
	dp = [x, y, "Line of Seperation"]
	data.append(dp)

	fig = plt.figure()
	ax = fig.add_subplot(111)
	ax.set_xscale("log")
	ax.set_xlim([1.05, 100.0])
	ax.set_ylim([0.6, 2.0])
	for index, dataSet in enumerate(data):
		ax.plot(dataSet[0], dataSet[1], label=dataSet[2])
	ax.plot(lineOfMaxCfsX, lineOfMaxCfsY, label="Line of Max Thrust Coefficient")
	ax.legend(fontsize="xx-small")
	plt.show()

def momentumThrust(gamma, sigma, Pc, diamThroat): # nd, nd, psi, inches
	machExit = Textbook_4_1.findMach(0, 20, gamma, sigma)
	pCOverPe = Textbook_4_1.isentropicPressureRatio(gamma, machExit)
	t1 = Pc * np.pi * diamThroat * diamThroat / 4
	t2 = 2 * gamma * gamma / (gamma - 1)
	t3 = (2 / (1 + gamma)) ** ((gamma + 1) / (gamma - 1))
	t4 = 1 - (1 / pCOverPe) ** ((gamma - 1) / gamma)
	t5 = np.sqrt(t2 * t3 * t4)
	t6 = t1 * t5
	return t6

def exhaustVelocity(gamma, Tc, MM, pCOverPe): # nd, F, lbm / lbm-mole, nd
	Ru = 1543 # ft * lbf / lbm-mole * Rankine
	t1 = 32.2 * 2 * gamma * Ru * (Tc + 459.67) / (MM * (gamma - 1))
	t2 = (1 - (1 / pCOverPe) ** ((gamma - 1) / gamma))
	t3 = np.sqrt(t1 * t2)
	return t3

# pascals, m^2, Kelvin, kg / kg-mol, ND
def mDot(Pc, At, Tc, M, gamma):
	Ru = 8317 # N * m / kg * mol * Kelvin
	t1 = Pc * At # pascals * m^2 = N
	t2 = np.sqrt( Ru * Tc / (gamma * M) )
	t3 = 2 / (gamma + 1)
	t4 = (-1.0 * (gamma + 1)) / (2.0 * (gamma - 1))
	ret = t1 / (t2 * (t3 ** t4))
	return ret

# psi, in^2, Farenheit, lbm/lbm-mole, nd
def mDotImperial(Pc, At, Tc, M, gamma):
	Ru = 1543 # ft * lbf / lbm-mole * Rankine
	t1 = Pc * At # psi * in^2 = lbf
	TcR = Tc + 459.7 # Rankine
	t2 = np.sqrt(Ru * TcR * 32.2 / (gamma * M))
	t3 = 2 / (gamma + 1)
	t4 = (-1.0 * (gamma + 1)) / (2.0 * (gamma - 1))
	ret = t1 / (t2 * (t3 ** t4))
	return ret * 32.2

if __name__ == "__main__":

	# ### TEXTBOOK 2.11 ###
	# print("\n")
	# ORBIT_ALT_1 = 85 # Miles
	# ORBIT_LAT_1 = np.radians(0.0)
	# ORBIT_ALT_2 = 90 # Miles
	# ORBIT_LAT_2 = np.radians(35.0)
	# Textbook_2_11(ORBIT_ALT_1, ORBIT_LAT_1)
	# Textbook_2_11(ORBIT_ALT_2, ORBIT_LAT_2)
	
	# ### TEXTBOOK 3.3 ###
	# # Stage 2.
	# propMass, inertMass = Textbook_3_3(
	# 	iPropMassFraction=0.8,
	# 	iPayloadMass=500,
	# 	iDeltaVIdeal=5000,
	# 	iSpecificImpulse=350
	# )
	# # Stage 1.
	# Textbook_3_3(
	# 	iPropMassFraction=0.9,
	# 	iPayloadMass=500 + 200 + propMass + inertMass,
	# 	iDeltaVIdeal=5000,
	# 	iSpecificImpulse=270
	# )

	# ### TEXTBOOK 3.4 ###
	# print("\n")
	# x = Textbook_3_4(
	# 	finalAltitude=20000,
	# 	gravity=9.81,
	# 	ISP=150,
	# 	M0=100,
	# 	MF=50
	# )
	# print(f"TEXTBOOK 3.4 : BURN TIME IS {x} SECONDS.\n")

	# ### TEXTBOOK 4.1 ###
	# psl = 101325 # pascals
	# pc = 5000000 # pascals
	# Tc = 3000 # kelvin
	# molarMass = 15 # kg / kg * molK
	# gamma = 1.2 # nd
	# exitDiam = 26.46 # cm
	# throatDiam = 10 # cm
	# chamberDiam = 15 # cm
	# x = Textbook_4_1(psl, pc, Tc, molarMass, gamma,
	# exitDiam, throatDiam, chamberDiam)
	# y = x.main()
	# printValuesInADictionary(y)

	# ### SP04-A1 ###
	# SP04A1() # has plot

	# ### SP04-B ###
	SP04_B() # has plot





























