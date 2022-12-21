# Python libraries.
import numpy as np
from numpy import array as npa
from numpy import linalg as la
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
FT_TO_M    = 0.3048
M_TO_FT    = 1.0 / FT_TO_M
EPSILON    = 0.0000001

if __name__ == "__main__":

	# Dynamics.
	LLA0 = npa([38.8719, 77.0563, 0.0])
	AZ0  = 55
	EL0  = 55
	SPD0 = 10
	ID   = "MOCK_HELLFIRE5DOF"
	DYN  = Dyn.construct_msl(LLA0, AZ0, EL0, SPD0, ID)

	# Components.
	COMPONENTS = {
		"PITCH_ACT": SecondOrderActuator("PITCH_DEFL"),
		"YAW_ACT": SecondOrderActuator("YAW_DEFL"),
		"CONTROL": MockHellfireControl("CONTROL"),
		"GUIDANCE": MockHellfireGuidance("GUIDANCE", npa([0.8, 0.8, -0.1]))
	}

	# Target.
	TGT_POS = npa([7000.0, 0.0, 500.0])
	TGT_VEL = np.zeros(3)

	# Sim control.
	TIME_INCREMENT = None
	MAXT           = 150
	FLAG           = int(0)
	LAST_TIME      = int(0)
	MISS_DIST      = 5.0
	LETHALITY      = None

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
			COMPONENTS["PITCH_ACT"].update(COMPONENTS["CONTROL"].PITCH_FIN_COMM)
		elif N_ID == "YAW_ACT":
			COMPONENTS["YAW_ACT"].update(COMPONENTS["CONTROL"].YAW_FIN_COMM)
		elif N_ID == "CONTROL":
			COMPONENTS["CONTROL"].update(
				DYN["STATE"]["REF_DIAM"],
				DYN["STATE"]["REF_AREA"],
				DYN["STATE"]["Q"],
				DYN["STATE"]["MASS"],
				DYN["STATE"]["SPEED"],
				DYN["STATE"]["TMOI"],
				DYN["STATE"]["RRATE"],
				DYN["STATE"]["QRATE"],
				DYN["STATE"]["CYB"],
				DYN["STATE"]["CYD"],
				DYN["STATE"]["CNB"],
				DYN["STATE"]["CND"],
				DYN["STATE"]["CZA"],
				DYN["STATE"]["CZD"],
				DYN["STATE"]["CMA"],
				DYN["STATE"]["CMD"],
				COMPONENTS["GUIDANCE"].NORM_COMM,
				COMPONENTS["GUIDANCE"].SIDE_COMM
			)
		elif N_ID == "GUIDANCE":
			ENU_TO_FLU = ct.FLIGHTPATH_TO_LOCAL_TM(
				DYN["STATE"]["ENUPSI"],
				-1.0 * DYN["STATE"]["ENUTHT"]
			)
			ENUPOS    = np.zeros(3)
			ENUPOS[0] = DYN["STATE"]["ENUPOSX"]
			ENUPOS[1] = DYN["STATE"]["ENUPOSY"]
			ENUPOS[2] = DYN["STATE"]["ENUPOSZ"]
			ENUVEL    = np.zeros(3)
			ENUVEL[0] = DYN["STATE"]["ENUVELX"]
			ENUVEL[1] = DYN["STATE"]["ENUVELY"]
			ENUVEL[2] = DYN["STATE"]["ENUVELZ"]
			COMPONENTS["GUIDANCE"].update(
				ENU_TO_FLU,
				ENUPOS,
				ENUVEL,
				TGT_POS,
				TGT_VEL
			)

		# Checks and flags.
		if np.floor(TOF) == LAST_TIME:
			FLAG = 1
		if TOF > MAXT:
			FLAG = 2
			LETHALITY = endChecks.TIME
		if COMPONENTS["GUIDANCE"].FLU_REL_POS[0] < 0.0 and TOF > 1:
			FLAG = 2
			LETHALITY = endChecks.POCA
		if la.norm(COMPONENTS["GUIDANCE"].FLU_REL_POS) < MISS_DIST and TOF > 1:
			FLAG = 2
			LETHALITY = endChecks.HIT
		if FLAG != 0:
			X    = DYN["STATE"]["ENUPOSX"]
			Y    = DYN["STATE"]["ENUPOSY"]
			Z    = DYN["STATE"]["ENUPOSZ"]
			MACH = DYN["STATE"]["MACH"]
			if FLAG == 1:
				print(f"TOF {TOF:.0f} ENU {X:.2f} {Y:.2f} {Z:.2f} MACH {MACH:.2f}")
				LAST_TIME += int(1)
				FLAG = 0
			elif FLAG == 2:
				print()
				print(f"REPORT:")
				print(f"STATUS: TOF {TOF:.4f} ENU {X:.2f} {Y:.2f} {Z:.2f} MACH {MACH:.2f}")
				print(f"RESULT: {LETHALITY}")
				print(f"MISS  : {COMPONENTS['GUIDANCE'].FLU_REL_POS}")
				print()
				break





