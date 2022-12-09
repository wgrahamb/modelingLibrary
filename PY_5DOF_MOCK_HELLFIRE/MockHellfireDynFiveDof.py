
# Python libraries.
import time
import copy
import numpy as np
from numpy import array as npa
from numpy import linalg as la
np.set_printoptions(suppress=True, precision=2)

# Utility.
from utility.returnAzAndElevation import returnAlphaAndBeta
from utility.interpolationGivenTwoVectors import linearInterpolation
import utility.coordinateTransformations as ct
import utility.loggingFxns as lf
import utility.earthTransforms as et

# Classes.
from classes.ATM1976 import ATM1976
from classes.MockHellfireMassAndMotor import MockHellfireMassAndMotor

"""

MOCK HELLFIRE DIMENSIONS:

REFERENCE_DIAMETER 0.18 M
REFERENCE_LENGTH 1.6 M
NOSE_LENGTH 0.249733 M
WING_SPAN 66.1175 MM
WING_TIP_CHORD 91.047 MM
WING_ROOT_CHORD 0.123564 M
TAIL_SPAN 71.3548 MM
TAIL_TIP_CHORD 0.387894 M
TAIL_ROOT_CHORD 0.48084 M
DISTANCE_FROM_BASE_OF_NOSE_TO_WING 0.323925 M
STARTING_CG_FROM_NOSE 0.644605 m
CENTER_OF_DEFLECTION_FROM_NOSE (1.8059 M - 0.249733 M)

UNCORRECTED_CENTER_OF_DEFLECTION_FROM_NOSE 1.8059 M
UNCORRECTED_REFERENCE_LENGTH 1.85026 m

"""

def construct_msl(
	INITIAL_POSITION, # m
	INITIAL_AZIMUTH, # rad
	INITIAL_ELEVATION, # rad
	INITIAL_AIRSPEED, # m/s
	ID # strings
):

	# ATMOSPHERE. ##################################################################
	ATMOS = ATM1976()
	ATMOS.update(INITIAL_POSITION[2], la.norm(INITIAL_AIRSPEED))
	RHO   = ATMOS.rho # kg/m^3
	Q     = ATMOS.q # pa
	P     = ATMOS.p # pa
	A     = ATMOS.a # m/s
	G     = ATMOS.g # m/s^2
	MACH  = ATMOS.mach # nd
	BETA  = None # "Normalized Speed" - Zarchan. nd
	if MACH > 1:
		BETA = np.sqrt(MACH ** 2 - 1)
	else:
		BETA = MACH

	# MASS AND MOTOR PROPERTIES. ###################################################
	MASS_AND_MOTOR = MockHellfireMassAndMotor()
	MASS_AND_MOTOR.update(0.0, P)
	XCG            = MASS_AND_MOTOR.CG_VALUES[0] # m from nose
	MASS           = MASS_AND_MOTOR.MASS # kg
	TMOI           = MASS_AND_MOTOR.TRANSVERSE_MOI # kg*m^2
	THRUST         = MASS_AND_MOTOR.THRUST # newtons

	# STATE. #######################################################################
	INITIAL_AZ = np.radians(INITIAL_AZIMUTH) # rad
	INITIAL_EL = np.radians(INITIAL_ELEVATION) # rad

	# FRAME. #######################################################################
	ENU_TO_FLU = ct.ORIENTATION_TO_LOCAL_TM(0.0, -INITIAL_EL, INITIAL_AZ) # nd
	ENUPOS   = INITIAL_POSITION # m
	ENUVEL   = INITIAL_AIRSPEED * (ENU_TO_FLU[0]) # m/s
	ENUEULER = npa([0.0, INITIAL_EL, INITIAL_AZ]) # rad

	# BODY. ########################################################################
	TOF             = 0.0 # seconds
	SPEED           = la.norm(INITIAL_AIRSPEED * (ENU_TO_FLU[0])) # m/s
	VEL_B           = ENU_TO_FLU @ (INITIAL_AIRSPEED * (ENU_TO_FLU[0])) # m/s
	ALPHA, SIDESLIP = returnAlphaAndBeta(VEL_B) # rad
	SPECIFIC_FORCE  = np.zeros(3) # m/s^2
	BODYRATE        = np.zeros(3) # rad/s

	# DATA. ########################################################################
	MISSILE = {
		"IDENTITY": ID,
		"LOGFILE": open(f"PY_5DOF_MOCK_HELLFIRE/output/{ID}.txt", "w"),
		"LETHALITY": "FLYING",
		"ATMOS": ATMOS,
		"MASS_AND_MOTOR": MASS_AND_MOTOR,
		"STATE": {
			"XCG": XCG, # m
			"MASS": MASS, # kg
			"TMOI": TMOI, # kg*m^2
			"THRUST": THRUST, # newtons

			"RHO": RHO, # kg/m^3
			"Q": Q, # pa
			"P": P, # pa
			"A": A, # m/s
			"G": G, # m/s^2
			"MACH": MACH, # nd
			"BETA": BETA, # nd

			"TOF": TOF, # seconds
			"SPEED": SPEED, # m/s
			"ALPHA": ALPHA, # rad
			"SIDESLIP": SIDESLIP, # rad
			"PRATE": BODYRATE[0], # rad/s
			"QRATE": BODYRATE[1], # rad/s
			"RRATE": BODYRATE[2], # rad/s
			"UDOT_0": SPECIFIC_FORCE[0], # m/s^2
			"VDOT_0": SPECIFIC_FORCE[1], # m/s^2
			"WDOT_0": SPECIFIC_FORCE[2], # m/s^2

			"ENUPOSX": ENUPOS[0], # m
			"ENUPOSY": ENUPOS[1], # m
			"ENUPOSZ": ENUPOS[2], # m
			"ENUVELX": ENUVEL[0], # m/s
			"ENUVELY": ENUVEL[1], # m/s
			"ENUVELZ": ENUVEL[2], # m/s
			"ENUPHI": ENUEULER[0], # rad
			"ENUTHT": ENUEULER[1], # rad
			"ENUPSI": ENUEULER[2], # rad
		}
	}

	lf.writeHeader(MISSILE["STATE"], MISSILE["LOGFILE"])
	lf.writeData(MISSILE["STATE"], MISSILE["LOGFILE"])

	return MISSILE

def fly_msl(
	MISSILE_INPUT_DICT, # state
	FLY_FOR_THIS_LONG, # seconds
	PITCH_FIN_DEFL_DEG_INPUT, # degrees
	YAW_FIN_DEFL_DEG_INPUT # degrees
):

	# HANDLE INPUT. ################################################################
	MSL = copy.copy(MISSILE_INPUT_DICT) # state

	# PROCESS INPUT. ###############################################################
	PITCH_FIN_DEFL_DEG = PITCH_FIN_DEFL_DEG_INPUT # deg
	PITCH_FIN_DEFL_RAD = np.radians(PITCH_FIN_DEFL_DEG) # rad
	YAW_FIN_DEFL_DEG   = YAW_FIN_DEFL_DEG_INPUT # deg
	YAW_FIN_DEFL_RAD   = np.radians(YAW_FIN_DEFL_DEG) # rad
	MAX_TIME           = MSL["STATE"]["TOF"] + FLY_FOR_THIS_LONG # seconds
	TIME_STEP          = FLY_FOR_THIS_LONG # seconds
	DT_LIM             = (1.0 / 100.0) # seconds
	if TIME_STEP > DT_LIM:
		TIME_STEP = DT_LIM

	# ALLOCATE STATE. ##############################################################
	XCG    = MSL["STATE"]["XCG"] # m
	MASS   = MSL["STATE"]["MASS"] # kg
	TMOI   = MSL["STATE"]["TMOI"] # kg*m^2
	THRUST = MSL["STATE"]["THRUST"] # newtons

	RHO  = MSL["STATE"]["RHO"] # kg/m^3
	Q    = MSL["STATE"]["Q"] # pa
	P    = MSL["STATE"]["P"] # pa
	A    = MSL["STATE"]["A"] # m/s
	G    = MSL["STATE"]["G"] # m/s^2
	MACH = MSL["STATE"]["MACH"] # nd
	BETA = MSL["STATE"]["BETA"] # nd

	TOF               = MSL["STATE"]["TOF"] # seconds
	SPEED             = MSL["STATE"]["SPEED"] # m/s
	ALPHA             = MSL["STATE"]["ALPHA"] # rad
	SIDESLIP          = MSL["STATE"]["SIDESLIP"] # rad
	BODYRATE          = np.zeros(3) # rad/s
	BODYRATE[0]       = MSL["STATE"]["PRATE"] # rad/s
	BODYRATE[1]       = MSL["STATE"]["QRATE"] # rad/s
	BODYRATE[2]       = MSL["STATE"]["RRATE"] # rad/s
	SPECIFIC_FORCE    = np.zeros(3) # m/s^2
	SPECIFIC_FORCE[0] = MSL["STATE"]["UDOT_0"] # m/s^2
	SPECIFIC_FORCE[1] = MSL["STATE"]["VDOT_0"] # m/s^2
	SPECIFIC_FORCE[2] = MSL["STATE"]["WDOT_0"] # m/s^2

	ENUPOS      = np.zeros(3) # m
	ENUPOS[0]   = MSL["STATE"]["ENUPOSX"] # m
	ENUPOS[1]   = MSL["STATE"]["ENUPOSY"] # m
	ENUPOS[2]   = MSL["STATE"]["ENUPOSZ"] # m
	ENUVEL      = np.zeros(3) # m/s
	ENUVEL[0]   = MSL["STATE"]["ENUVELX"] # m/s
	ENUVEL[1]   = MSL["STATE"]["ENUVELY"] # m/s
	ENUVEL[2]   = MSL["STATE"]["ENUVELZ"] # m/s
	ENUEULER    = np.zeros(3) # rad
	ENUEULER[0] = MSL["STATE"]["ENUPHI"] # rad
	ENUEULER[1] = MSL["STATE"]["ENUTHT"] # rad
	ENUEULER[2] = MSL["STATE"]["ENUPSI"] # rad

	# INTEGRATION STATE. ###########################################################
	INTEGRATION_PASS = 0
	STATE_P0         = ENUPOS # m
	STATE_V0         = ENUVEL # m/s
	STATE_E0         = ENUEULER # rad/s
	STATE_EDOT0      = BODYRATE # rad/s^2
	V1               = np.zeros(3) # m/s
	A1               = np.zeros(3) # m/s^2
	EDOT1            = np.zeros(3) # rad/s
	EDOTDOT1         = np.zeros(3) # rad/s^2
	V2               = np.zeros(3) # m/s
	A2               = np.zeros(3) # m/s^2
	EDOT2            = np.zeros(3) # rad/s
	EDOTDOT2         = np.zeros(3) # rad/s^2
	V3               = np.zeros(3) # m/s
	A3               = np.zeros(3) # m/s^2
	EDOT3            = np.zeros(3) # rad/s
	EDOTDOT3         = np.zeros(3) # rad/s^2
	V4               = np.zeros(3) # m/s
	A4               = np.zeros(3) # m/s^2
	EDOT4            = np.zeros(3) # rad/s
	EDOTDOT4         = np.zeros(3) # rad/s^2

	# AIRFRAME. ####################################################################
	CD_LOOKUP          = [0.1, 0.6] # nd
	MACH_LOOKUP        = [0.6, 2.5] # nd
	MM_TO_M            = 1.0 / 1000.0
	REF_DIAM           = 0.18 # m
	REF_LNGTH          = 1.6 # m
	REF_AREA           = np.pi * (REF_DIAM ** 2) / 4 # m^2
	WNG_HLF_SPN        = 66.1175 * MM_TO_M / 2.0 # m
	WNG_TIP_CHRD       = 91.047 * MM_TO_M # m
	WNG_ROOT_CHRD      = 0.123564 # m
	WNG_AREA           = 0.5 * WNG_HLF_SPN * (WNG_TIP_CHRD + WNG_ROOT_CHRD) # m^2
	TAIL_HLF_SPN       = 71.3548 * MM_TO_M / 2.0 # m
	TAIL_TIP_CHRD      = 0.387894 # m
	TAIL_ROOT_CHRD     = 0.48084 # m
	TAIL_AREA          = 0.5 * TAIL_HLF_SPN * (TAIL_TIP_CHRD + TAIL_ROOT_CHRD) # m^2
	NOSE_LNGTH         = 0.249733 # m
	X_BASENOSE2WINGTIP = 0.323925 # m
	X_NOSETIP2XCD      = 1.8059 - NOSE_LNGTH # m
	PLANFORM_AREA      = (REF_LNGTH - NOSE_LNGTH) \
		* REF_DIAM + 0.667 * NOSE_LNGTH * REF_DIAM # m^2
	XCP_NOSE           = 0.67 * NOSE_LNGTH # m
	XCP_WNG            = NOSE_LNGTH + X_BASENOSE2WINGTIP \
		+ 0.7 * WNG_ROOT_CHRD - 0.2 * WNG_TIP_CHRD # m
	AN                 = 0.67 * NOSE_LNGTH * REF_DIAM # m^2
	AB                 = (REF_LNGTH - NOSE_LNGTH) * REF_DIAM # m^2
	XCP_BODY           = (0.67 * AN * NOSE_LNGTH + AB \
		* (NOSE_LNGTH + 0.5 * (REF_LNGTH - NOSE_LNGTH))) / (AN + AB) # m

	# FUNCTION FOR RETURNING DATA DICTIONARY. ######################################
	def get_state():
		STATE = {
			"XCG": XCG, # m
			"MASS": MASS, # kg
			"TMOI": TMOI, # kg*m^2
			"THRUST": THRUST, # newtons

			"RHO": RHO, # kg/m^3
			"Q": Q, # pa
			"P": P, # pa
			"A": A, # m/s
			"G": G, # m/s^2
			"MACH": MACH, # nd
			"BETA": BETA, # nd

			"TOF": TOF, # seconds
			"SPEED": SPEED, # m/s
			"ALPHA": ALPHA, # rad
			"SIDESLIP": SIDESLIP, # rad
			"PRATE": BODYRATE[0], # rad/s
			"QRATE": BODYRATE[1], # rad/s
			"RRATE": BODYRATE[2], # rad/s
			"UDOT_0": SPECIFIC_FORCE[0], # m/s^2
			"VDOT_0": SPECIFIC_FORCE[1], # m/s^2
			"WDOT_0": SPECIFIC_FORCE[2], # m/s^2

			"ENUPOSX": ENUPOS[0], # m
			"ENUPOSY": ENUPOS[1], # m
			"ENUPOSZ": ENUPOS[2], # m
			"ENUVELX": ENUVEL[0], # m/s
			"ENUVELY": ENUVEL[1], # m/s
			"ENUVELZ": ENUVEL[2], # m/s
			"ENUPHI": ENUEULER[0], # rad
			"ENUTHT": ENUEULER[1], # rad
			"ENUPSI": ENUEULER[2], # rad
		}
		return STATE

	# LOOP. ########################################################################
	GO = True
	while GO:

		# ATMOSPHERE. ##############################################################
		MSL["ATMOS"].update(ENUPOS[2], SPEED)
		RHO  = MSL["ATMOS"].rho # kg/m^3
		Q    = MSL["ATMOS"].q # pa
		P    = MSL["ATMOS"].p # pa
		A    = MSL["ATMOS"].a # m/s
		G    = MSL["ATMOS"].g # m/s^2
		MACH = MSL["ATMOS"].mach # nd
		BETA = None # "Normalized Speed" - Zarchan. nd
		if MACH > 1:
			BETA = np.sqrt(MACH ** 2 - 1)
		else:
			BETA = MACH

		# MASS AND MOTOR PROPERTIES. ###############################################
		MSL["MASS_AND_MOTOR"].update(TOF, P)
		XCG    = MSL["MASS_AND_MOTOR"].XCG # m
		MASS   = MSL["MASS_AND_MOTOR"].MASS # kg
		TMOI   = MSL["MASS_AND_MOTOR"].TRANSVERSE_MOI # kg*m^2
		THRUST = MSL["MASS_AND_MOTOR"].THRUST # newtons

		# ATTITUDE. ################################################################
		ENU_TO_FLU = ct.ORIENTATION_TO_LOCAL_TM(
			ENUEULER[0], -1.0 * ENUEULER[1], ENUEULER[2]) # nd
		SPEED = la.norm(ENUVEL) # m/s
		VEL_B = ENU_TO_FLU @ ENUVEL # m/s
		TEMP1, TEMP2 = returnAlphaAndBeta(VEL_B) # rad

		# ALPHA AND BETA IN FORWARD, RIGHT, DOWN. ##################################
		ALPHA = -1.0 * TEMP1 # rad
		SIDESLIP = -1.0 * TEMP2 # rad

		# BASIC DRAG MODEL. ########################################################
		CD = None # nd
		if MACH >= MACH_LOOKUP[0]:
			CD = linearInterpolation(MACH, MACH_LOOKUP, CD_LOOKUP)
		else:
			CD = CD_LOOKUP[0]
		DRAG_FORCE = CD * REF_AREA * Q # newtons
		WIND_TO_BODY = ct.FLIGHTPATH_TO_LOCAL_TM(SIDESLIP, ALPHA) # nd
		WIND_DRAG_FORCE = npa([-DRAG_FORCE, 0.0, 0.0]) # newtons
		BODY_DRAG = (WIND_TO_BODY @ WIND_DRAG_FORCE) / MASS # m/s^2

		# AERODYNAMICS. ############################################################
		CY = 2 * SIDESLIP + \
			(1.5 * PLANFORM_AREA * SIDESLIP * SIDESLIP) / REF_AREA + \
			(8 * WNG_AREA * SIDESLIP) / (BETA * REF_AREA) + \
			(8 * TAIL_AREA * (SIDESLIP + YAW_FIN_DEFL_RAD)) / (BETA * REF_AREA) # nd
		CN = 2 * SIDESLIP * ((XCG - XCP_NOSE) / REF_DIAM) + \
			((1.5 * PLANFORM_AREA * SIDESLIP * SIDESLIP) / REF_AREA) * \
			((XCG - XCP_BODY) / REF_DIAM) + \
			((8 * WNG_AREA * SIDESLIP) / (BETA * REF_AREA)) * \
			((XCG - XCP_WNG) / REF_DIAM) + \
			((8 * TAIL_AREA * (SIDESLIP + YAW_FIN_DEFL_RAD)) / \
			(BETA * REF_AREA)) * ((XCG - X_NOSETIP2XCD) / REF_DIAM) # nd

		CZ = 2 * ALPHA + \
			(1.5 * PLANFORM_AREA * ALPHA * ALPHA) / REF_AREA + \
			(8 * WNG_AREA * ALPHA) / (BETA * REF_AREA) + \
			(8 * TAIL_AREA * (ALPHA + PITCH_FIN_DEFL_RAD)) / (BETA * REF_AREA) # nd
		CM = 2 * ALPHA * ((XCG - XCP_NOSE) / REF_DIAM) + \
			((1.5 * PLANFORM_AREA * ALPHA * ALPHA) / REF_AREA) * \
			((XCG - XCP_BODY) / REF_DIAM) + \
			((8 * WNG_AREA * ALPHA) / (BETA * REF_AREA)) * \
			((XCG - XCP_WNG) / REF_DIAM) + \
			((8 * TAIL_AREA * (ALPHA + PITCH_FIN_DEFL_RAD)) / \
			(BETA * REF_AREA)) * ((XCG - X_NOSETIP2XCD) / REF_DIAM) # nd

		# DERIVATIVES. #############################################################
		EDOTDOT_0         = np.zeros(3) # rad/s^2
		EDOTDOT_0[0]      = 0.0 # rad/s^2
		EDOTDOT_0[1]      = (Q * REF_AREA * REF_DIAM * CM) / TMOI # rad/s^2
		EDOTDOT_0[2]      = (Q * REF_AREA * REF_DIAM * CN) / TMOI # rad/s^2
		SPECIFIC_FORCE[0] = THRUST / MASS # m/s^2
		SPECIFIC_FORCE[1] = (Q * REF_AREA * CY) / MASS # m/s^2
		SPECIFIC_FORCE[2] = (Q * REF_AREA * CZ) / MASS # m/s^2
		LOCAL_G           = npa([0.0, 0.0, -1.0 * G]) # m/s^2
		BODY_G            = ENU_TO_FLU @ LOCAL_G # m/s^2
		SPECIFIC_FORCE    += (BODY_G + BODY_DRAG) # m/s^2
		ACC_0             = SPECIFIC_FORCE @ ENU_TO_FLU # m/s^2

		# STATE. ###################################################################
		if INTEGRATION_PASS == 0:

			# LOG DATA. ############################################################
			MSL["STATE"] = get_state()
			lf.writeData(MSL["STATE"], MSL["LOGFILE"] )

			# END CHECK. ###########################################################
			if TOF > MAX_TIME:
				MSL["LETHALITY"] = "MAX_TIME"
				return MSL
			if ENUPOS[2] < 0.0:
				print(f"GROUND - TOF : {TOF:.2f}, ENU : {ENUPOS}, MACH : {MACH:.2f}")
				MSL["LETHALITY"] = "GROUND"
				return MSL
			if np.isnan(np.sum(ENUPOS)):
				print(f"NAN - TOF : {TOF:.2f}, ENU : {ENUPOS}, MACH : {MACH:.2f}")
				MSL["LETHALITY"] = "NAN"
				return MSL

			# BEGIN INTEGRATION PASS. ##############################################
			STATE_P0    = copy.deepcopy(ENUPOS) # m
			STATE_V0    = copy.deepcopy(ENUVEL) # m/s
			STATE_E0    = copy.deepcopy(ENUEULER) # rad/s
			STATE_EDOT0 = copy.deepcopy(BODYRATE) # rad/s^2

			V1       = ENUVEL # m/s
			A1       = ACC_0 # m/s^2
			EDOT1    = BODYRATE # rad/s
			EDOTDOT1 = EDOTDOT_0 # rad/s^2

			ENUPOS   = STATE_P0 + V1 * (TIME_STEP / 2.0) # m
			ENUVEL   = STATE_V0 + A1 * (TIME_STEP / 2.0) # m/s
			ENUEULER = STATE_E0 + EDOT1 * (TIME_STEP / 2.0) # rad
			BODYRATE = STATE_EDOT0 + EDOTDOT1 * (TIME_STEP / 2.0) # rad/s

			TOF += (TIME_STEP / 2.0) # seconds

			INTEGRATION_PASS += 1

		elif INTEGRATION_PASS == 1:

			V2       = ENUVEL # m/s
			A2       = ACC_0 # m/s^2
			EDOT2    = BODYRATE # rad/s
			EDOTDOT2 = EDOTDOT_0 # rad/s^2

			ENUPOS   = STATE_P0 + V2 * (TIME_STEP / 2.0) # m
			ENUVEL   = STATE_V0 + A2 * (TIME_STEP / 2.0) # m/s
			ENUEULER = STATE_E0 + EDOT2 * (TIME_STEP / 2.0) # rad
			BODYRATE = STATE_EDOT0 + EDOTDOT2 * (TIME_STEP / 2.0) # rad/s

			INTEGRATION_PASS += 1

		elif INTEGRATION_PASS == 2:

			V3       = ENUVEL # m/s
			A3       = ACC_0 # m/s^2
			EDOT3    = BODYRATE # rad/s
			EDOTDOT3 = EDOTDOT_0 # rad/s^2

			ENUPOS   = STATE_P0 + V3 * (TIME_STEP) # m
			ENUVEL   = STATE_V0 + A3 * (TIME_STEP) # m/s
			ENUEULER = STATE_E0 + EDOT3 * (TIME_STEP) # rad
			BODYRATE = STATE_EDOT0 + EDOTDOT3 * (TIME_STEP) # rad/s

			TOF += (TIME_STEP / 2.0) # seconds

			INTEGRATION_PASS += 1

		elif INTEGRATION_PASS == 3:

			V4       = ENUVEL # m/s
			A4       = ACC_0 # m/s^2
			EDOT4    = BODYRATE # rad/s
			EDOTDOT4 = EDOTDOT_0 # rad/s^2

			ENUPOS   = STATE_P0 + (TIME_STEP / 6.0) * \
				(V1 + 2 * V2 + 2 * V3 + V4) # m
			ENUVEL   = STATE_V0 + (TIME_STEP / 6.0) * \
				(A1 + 2 * A2 + 2 * A3 + A4) # m/s
			ENUEULER = STATE_E0 + (TIME_STEP / 6.0) * \
				(EDOT1 + 2 * EDOT2 + 2 * EDOT3 + EDOT4) # rad
			BODYRATE = STATE_EDOT0 + (TIME_STEP / 6.0) * \
				(EDOTDOT1 + 2 * EDOTDOT2 + 2 * EDOTDOT3 + EDOTDOT4) # rad/s

			INTEGRATION_PASS = 0



if __name__ == "__main__":

	wallClockStart = time.time()

	POS0 = np.zeros(3)
	AZ0  = 0
	EL0  = 45
	SPD0 = 10
	MSL  = construct_msl(POS0, AZ0, EL0, SPD0, "MOCK_HELLFIRE5DOF")
	MSL  = fly_msl(MSL, 100, -4.0, 0.0)

	wallClockEnd = time.time()
	print(f"RUN TIME : {wallClockEnd - wallClockStart} SECONDS")
