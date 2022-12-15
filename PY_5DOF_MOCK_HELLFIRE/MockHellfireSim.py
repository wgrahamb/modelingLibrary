# Python libraries.
import numpy as np
from numpy import array as npa
np.set_printoptions(suppress=True, precision=2)

# Utility.

# Components.
from classes.SecondOrderActuator import SecondOrderActuator

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
	MSL  = Dyn.construct_msl(LLA0, AZ0, EL0, SPD0, ID)

	# Components.
	COMPONENTS = {
		"PITCH_ACT": SecondOrderActuator("PITCH_DEFL"),
		"YAW_ACT": SecondOrderActuator("YAW_DEFL"),
	}

	# Sim control.
	TIME_INCREMENT = None

	# Simple Guidance and Control.
	MANEUVER1         = 3 # Seconds.
	MANEUVER2         = 30 # Seconds.
	PITCH_FIN_COMMAND = None
	YAW_FIN_COMMAND   = None
	PITCHCOMMAND1     = 0 # Degrees.
	YAWCOMMAND1       = 0 # Degrees.
	PITCHCOMMAND2     = 0 # Degrees.
	YAWCOMMAND2       = 0 # Degrees.
	PITCHCOMMAND3     = 0 # Degrees.
	YAWCOMMAND3       = 0 # Degrees.

	LAST_TIME = int(0)
	while MSL["LETHALITY"] == endChecks.FLIGHT or MSL["LETHALITY"] == endChecks.TIME:

		# Dynamics tof is driver.
		TOF = MSL["STATE"]["TOF"]

		# Basic guidance and control.
		if TOF < MANEUVER1:
			PITCH_FIN_COMMAND = PITCHCOMMAND1
			YAW_FIN_COMMAND   = YAWCOMMAND1
		elif TOF < MANEUVER2:
			PITCH_FIN_COMMAND = PITCHCOMMAND2
			YAW_FIN_COMMAND   = YAWCOMMAND2
		else:
			PITCH_FIN_COMMAND = PITCHCOMMAND3
			YAW_FIN_COMMAND   = YAWCOMMAND3

		# Get next update time.
		N    = 0.0
		N_ID = ""
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
			MSL = Dyn.fly_msl(
				MISSILE_INPUT_DICT=MSL,
				FLY_FOR_THIS_LONG=TIME_INCREMENT,
				PITCH_FIN_DEFL_DEG_INPUT=COMPONENTS["PITCH_ACT"].DEFLECTION,
				YAW_FIN_DEFL_DEG_INPUT=COMPONENTS["YAW_ACT"].DEFLECTION
			)

		# Update components. Would like to make this bit more terse.
		if N_ID == "PITCH_ACT":
			COMPONENTS["PITCH_ACT"].update(PITCH_FIN_COMMAND)
		elif N_ID == "YAW_ACT":
			COMPONENTS["YAW_ACT"].update(YAW_FIN_COMMAND)

		# Console report.
		if np.floor(TOF) == LAST_TIME:
			X         = MSL["STATE"]["ENUPOSX"]
			Y         = MSL["STATE"]["ENUPOSY"]
			Z         = MSL["STATE"]["ENUPOSZ"]
			MACH      = MSL["STATE"]["MACH"]
			print(f"TOF {TOF:.0f} ENU {X:.2f} {Y:.2f} {Z:.2f} MACH {MACH:.2f}")
			LAST_TIME += 1

