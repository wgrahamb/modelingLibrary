# Python libraries.
import numpy as np
from numpy import array as npa
from numpy import linalg as la
import time
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

# Some constants.
RAD_TO_DEG = 57.2957795130823
DEG_TO_RAD = 1.0 / RAD_TO_DEG
FT_TO_M    = 0.3048
M_TO_FT    = 1.0 / FT_TO_M
EPSILON    = 0.0000001

if __name__ == "__main__":

	# Dynamics.
	LLA0 = npa([38.8719, 77.0563, 0.0])
	AZ0  = -40
	EL0  = 25
	SPD0 = 10
	ID   = "MOCK_HELLFIRE5DOF"
	DYN  = Dyn.construct_msl(LLA0, AZ0, EL0, SPD0, ID)

	# Components.
	COMPONENTS = {
		"PITCH_ACT": SecondOrderActuator("PITCH_DEFL"),
		"YAW_ACT": SecondOrderActuator("YAW_DEFL"),
		"CONTROL": MockHellfireControl("CONTROL"),
		"GUIDANCE": MockHellfireGuidance(
			"GUIDANCE",
			npa([0.4, 0.5, 0.4])
		)
	}

	# Target.
	TGT_POS = npa([6000.0, 0.0, 500.0])
	TGT_VEL = npa([-100.0, 0.0, 0.0])

	# Sim control.
	REAL_START     = time.time()
	TIME_INCREMENT = None
	MAXT           = 1500
	FLAG           = int(0)
	LAST_TIME      = int(0)
	MISS_DIST      = 5.0
	LETHALITY      = None

	# Loop.
	print()
	while DYN["LETHALITY"] == endChecks.FLIGHT or \
		DYN["LETHALITY"] == endChecks.TIME:

		# Dynamics tof is driver.
		TOF = DYN["STATE"]["TOF"]

		# Get next update time.
		N_DT = None
		N_ID = None
		for index, key in enumerate(COMPONENTS.keys()):
			if index == 0:
				N_DT    = COMPONENTS[f"{key}"].NEXT_UPDATE_TIME
				N_ID = key
			elif COMPONENTS[f"{key}"].NEXT_UPDATE_TIME < N_DT:
				N_DT    = COMPONENTS[f"{key}"].NEXT_UPDATE_TIME
				N_ID = key

		# Check for dynamics and target update.
		TIME_INCREMENT = N_DT - TOF
		if TIME_INCREMENT > EPSILON:

			# Update dynamics.
			DYN = Dyn.fly_msl(
				MISSILE_INPUT_DICT=DYN,
				FLY_FOR_THIS_LONG=TIME_INCREMENT,
				PITCH_FIN_DEFL_DEG_INPUT=COMPONENTS["PITCH_ACT"].DEFLECTION,
				YAW_FIN_DEFL_DEG_INPUT=COMPONENTS["YAW_ACT"].DEFLECTION
			)

			# Update target.
			TGT_POS += (TIME_INCREMENT * TGT_VEL)

		# Update components.
		if N_ID == "PITCH_ACT":

			# Handle input.
			PITCH_FIN_COMM = COMPONENTS["CONTROL"].PITCH_FIN_COMM

			# Update pitching actuator.
			COMPONENTS["PITCH_ACT"].update(PITCH_FIN_COMM)

		elif N_ID == "YAW_ACT":

			# Handle input.
			YAW_FIN_COMM = COMPONENTS["CONTROL"].YAW_FIN_COMM

			# Update yawing actuator.
			COMPONENTS["YAW_ACT"].update(YAW_FIN_COMM)

		elif N_ID == "CONTROL":

			# Handle input.
			REF_DIAM  = DYN["STATE"]["REF_DIAM"]
			REF_AREA  = DYN["STATE"]["REF_AREA"]
			Q         = DYN["STATE"]["Q"]
			MASS      = DYN["STATE"]["MASS"]
			SPD       = DYN["STATE"]["SPEED"]
			TMOI      = DYN["STATE"]["TMOI"]
			RRATE     = DYN["STATE"]["RRATE"]
			QRATE     = DYN["STATE"]["QRATE"]
			CYB       = DYN["STATE"]["CYB"]
			CYD       = DYN["STATE"]["CYD"]
			CNB       = DYN["STATE"]["CNB"]
			CND       = DYN["STATE"]["CND"]
			CZA       = DYN["STATE"]["CZA"]
			CZD       = DYN["STATE"]["CZD"]
			CMA       = DYN["STATE"]["CMA"]
			CMD       = DYN["STATE"]["CMD"]
			NORM_COMM = COMPONENTS["GUIDANCE"].NORM_COMM
			SIDE_COMM = COMPONENTS["GUIDANCE"].SIDE_COMM

			# Update control.
			COMPONENTS["CONTROL"].update(
				REF_DIAM,
				REF_AREA,
				Q,
				MASS,
				SPD,
				TMOI,
				RRATE,
				QRATE,
				CYB,
				CYD,
				CNB,
				CND,
				CZA,
				CZD,
				CMA,
				CMD,
				NORM_COMM,
				SIDE_COMM
			)

		elif N_ID == "GUIDANCE":

			# Handle input.
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

			# Update guidance.
			COMPONENTS["GUIDANCE"].update(
				ENU_TO_FLU,
				ENUPOS,
				ENUVEL,
				TGT_POS,
				TGT_VEL
			)

		# Console report check.
		if np.floor(TOF) == LAST_TIME:
			FLAG = 1

		# End checks.
		if TOF > MAXT:
			FLAG = 2
			LETHALITY = endChecks.TIME
		elif COMPONENTS["GUIDANCE"].FLU_REL_POS[0] < 0.0 and TOF > 1:
			FLAG = 2
			LETHALITY = endChecks.POCA
		elif la.norm(COMPONENTS["GUIDANCE"].FLU_REL_POS) < MISS_DIST and TOF > 1:
			FLAG = 2
			LETHALITY = endChecks.HIT

		# Console report.
		if FLAG != 0:
			X    = DYN["STATE"]["ENUPOSX"]
			Y    = DYN["STATE"]["ENUPOSY"]
			Z    = DYN["STATE"]["ENUPOSZ"]
			ENU  = npa([X, Y, Z])
			MACH = DYN["STATE"]["MACH"]
			if FLAG == 1:
				print(f"TOF {TOF:>{2}.0f} | E {X:>{7}.2f} | N {Y:>{7}.2f} | U {Z:>{7}.2f} | MACH {MACH:>{4}.2f}")
				LAST_TIME += int(1)
				FLAG      = 0
			elif FLAG == 2:
				RUNTIME = time.time() - REAL_START
				MISS    = COMPONENTS['GUIDANCE'].FLU_REL_POS
				print()
				print(f"*** REPORT ***")
				print(f"STATUS   : TOF {TOF:.4f} | ENU {ENU} | MACH {MACH:.2f}")
				print(f"RESULT   : {LETHALITY.name}")
				print(f"MISS     : {la.norm(MISS):.4f} | {MISS}")
				print(f"RUN TIME : {RUNTIME:.2f} SECONDS")
				print()
				break





