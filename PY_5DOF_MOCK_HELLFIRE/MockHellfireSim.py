# Python libraries.
import numpy as np
from numpy import array as npa
import pandas as pd
np.set_printoptions(suppress=True, precision=2)

# Utility.
from utility import coordinateTransformations as ct

# Components.
from classes.SecondOrderActuator import SecondOrderActuator
from classes.MockHellfireControl import MockHellfireControl
from classes.MockHellfireGuidance import MockHellfireGuidance

# Dynamics.
import MockHellfireDynFiveDof as Dyn
from MockHellfireDynFiveDof import endChecks

# MATH CONSTANTS
RAD_TO_DEG = 57.2957795130823
DEG_TO_RAD = 1.0 / RAD_TO_DEG
EPSILON    = 0.0000001

if __name__ == "__main__":

	# Dynamics.
	LLA0 = npa([38.8719, 77.0563, 0.0])
	AZ0  = 0 # Degrees.
	EL0  = 45 # Degrees.
	SPD0 = 10
	ID   = "MOCK_HELLFIRE5DOF"
	DYN  = Dyn.construct_msl(LLA0, AZ0, EL0, SPD0, ID)

	# Components.
	COMPONENTS = {
		"PITCH_ACT": SecondOrderActuator("PITCH_DEFL"),
		"YAW_ACT": SecondOrderActuator("YAW_DEFL")
	}

	# Target.
	TGT_POS = npa([7000.0, 0.0, 500.0])
	TGT_VEL = np.zeros(3)

	# Sim control.
	TIME_INCREMENT = None
	MAXT           = 150

	# Yaw Control Test.
	LAST_YAW_PROP_ERR = 0.0
	YAW_PROP_ERR      = 0.0
	YAW_INT_ERR       = 0.0

	LAST_TIME = int(0)
	while DYN["LETHALITY"] == endChecks.FLIGHT or DYN["LETHALITY"] == endChecks.TIME:

		# Dynamics tof is driver.
		TOF = DYN["STATE"]["TOF"]

		# Get next update time.
		N    = None
		N_ID = None
		for index, key in enumerate(COMPONENTS.keys()):
			if index == 0:
				N    = COMPONENTS[f"{key}"].NEXT_UPDATE_TIME
				N_ID = key
			elif COMPONENTS[f"{key}"].NEXT_UPDATE_TIME < N:
				N    = COMPONENTS[f"{key}"].NEXT_UPDATE_TIME
				N_ID = key

		# Update dynamics.
		TIME_INCREMENT = N - TOF
		if TIME_INCREMENT > EPSILON:
			DYN = Dyn.fly_msl(
				MISSILE_INPUT_DICT=DYN,
				FLY_FOR_THIS_LONG=TIME_INCREMENT,
				PITCH_FIN_DEFL_DEG_INPUT=COMPONENTS["PITCH_ACT"].DEFLECTION,
				YAW_FIN_DEFL_DEG_INPUT=COMPONENTS["YAW_ACT"].DEFLECTION
			)

		# Update components.
		if N_ID == "PITCH_ACT":
			COMPONENTS["PITCH_ACT"].update(0.0)
		elif N_ID == "YAW_ACT":

			YAW_FIN_COMM = 0.0

			# # Yaw Control Test.
			# YAW_RATE_COMM     = np.radians(5.0)
			# LAST_YAW_PROP_ERR = YAW_PROP_ERR
			# YAW_PROP_ERR      = DYN["STATE"]["RRATE"] - YAW_RATE_COMM
			# YAW_INT_ERR       += (YAW_PROP_ERR * COMPONENTS["YAW_ACT"].TIME_STEP)
			# YAW_DER_ERR       = (YAW_PROP_ERR - LAST_YAW_PROP_ERR) / COMPONENTS["YAW_ACT"].TIME_STEP
			# KU                = 0.9
			# TU                = (28.684-28.455)
			# KP                = 0.6 * KU
			# KI                = 1.2 * KU / TU
			# KD                = 3 * KU * TU / 40
			# YAW_FIN_COMM      = np.degrees(
			# 	KP * YAW_PROP_ERR  + \
			# 	KI * YAW_INT_ERR + \
			# 	KD * YAW_DER_ERR
			# )

			COMPONENTS["YAW_ACT"].update(YAW_FIN_COMM)

		# Console report.
		if np.floor(TOF) == LAST_TIME:
			X         = DYN["STATE"]["ENUPOSX"]
			Y         = DYN["STATE"]["ENUPOSY"]
			Z         = DYN["STATE"]["ENUPOSZ"]
			MACH      = DYN["STATE"]["MACH"]
			print(f"TOF {TOF:.0f} ENU {X:.2f} {Y:.2f} {Z:.2f} MACH {MACH:.2f}")
			LAST_TIME += 1

		# Console report.
		if TOF > MAXT:
			X    = DYN["STATE"]["ENUPOSX"]
			Y    = DYN["STATE"]["ENUPOSY"]
			Z    = DYN["STATE"]["ENUPOSZ"]
			MACH = DYN["STATE"]["MACH"]
			print(f"TOF {TOF:.4f} ENU {X:.2f} {Y:.2f} {Z:.2f} MACH {MACH:.2f}")
			break

# f1 = r"PY_5DOF_MOCK_HELLFIRE/data/MOCK_HELLFIRE5DOF.txt"
# viewFile = f1
# df = pd.read_csv(open(f"{viewFile}"), delimiter=" ")

# MAX = max(df.iloc[:]["RRATE"]) * RAD_TO_DEG
# MIN = min(df.iloc[:]["RRATE"]) * RAD_TO_DEG

# print((MAX+MIN)/2)



