# Python libraries.
import time
import copy
from enum import Enum
import numpy as np
from numpy import array as npa
from numpy import linalg as la
np.set_printoptions(suppress=True, precision=2)

# Utility.
from   utility.getAlphaAndBeta              import getAlphaAndBeta
from   utility.interpolationGivenTwoVectors import linearInterpolation
import utility.coordinateTransformations    as ct
import utility.loggingFxns                  as lf
import utility.earthTransforms              as et

# Classes.
from classes.ATM1976                  import ATM1976
from classes.MockHellfireMassAndMotor import MockHellfireMassAndMotor

# End checks.
class endChecks(Enum):
	HIT    = 1
	FLIGHT = 0
	GROUND = -1
	POCA   = -2
	NAN    = -3
	TIME   = -4

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
	INITIAL_LLA, # geodetic
	INITIAL_AZIMUTH, # deg
	INITIAL_ELEVATION, # deg
	INITIAL_AIRSPEED, # m/s
	ID # string
):

	# ATMOSPHERE. ##################################################################
	ATMOS = ATM1976()
	ATMOS.update(INITIAL_LLA[2], la.norm(INITIAL_AIRSPEED))
	RHO   = ATMOS.rho # kg/m^3
	Q     = ATMOS.q # pa
	P     = ATMOS.p # pa
	A     = ATMOS.a # m/s
	G     = ATMOS.g # m/s^2
	MACH  = ATMOS.mach # nd

	# "Normalized Speed" - Zarchan. nd (KLUDGE)
	if MACH > 1:
		BETA = np.sqrt(MACH ** 2 - 1)
	else:
		BETA = np.sqrt(1.001 ** 2 - 1)

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

	# FRAMES. ######################################################################
	# ENU.
	ENU_TO_FLU = ct.FLIGHTPATH_TO_LOCAL_TM(INITIAL_AZ, -1.0 * INITIAL_EL) # nd
	ENUPOS     = np.zeros(3) # m
	ENUVEL     = INITIAL_AIRSPEED * (ENU_TO_FLU[0]) # m/s
	ENUTHT     = INITIAL_EL # rad
	ENUPSI     = INITIAL_AZ # rad

	GEODETIC0 = npa([np.radians(INITIAL_LLA[0]), np.radians(INITIAL_LLA[1]),
		INITIAL_LLA[2]]) # rad, rad, m
	GEODETIC  = copy.deepcopy(GEODETIC0) # rad, rad, m

	ECEF0 = et.LLA_TO_ECI(GEODETIC0) # m
	ECEF  = copy.deepcopy(ECEF0) # m

	# BODY. ########################################################################
	TOF            = 0.0 # seconds
	SPEED          = la.norm(INITIAL_AIRSPEED * (ENU_TO_FLU[0])) # m/s
	VEL_B          = ENU_TO_FLU @ (INITIAL_AIRSPEED * (ENU_TO_FLU[0])) # m/s
	TEMP1, TEMP2   = getAlphaAndBeta(VEL_B) # rad
	ALPHA          = -1.0 * TEMP1 # rad
	SIDESLIP       = -1.0 * TEMP2 # rad
	SPECIFIC_FORCE = np.zeros(3) # m/s^2
	QRATE          = 0.0 # rad/s
	RRATE          = 0.0 # rad/s

	REF_DIAM  = 0.18 # m
	REF_LNGTH = 1.6 # m
	REF_AREA  = np.pi * (REF_DIAM ** 2) / 4 # m^2

	# DATA. ########################################################################
	MISSILE = {
		"IDENTITY": ID,
		"LOGFILE": f"PY_5DOF_MOCK_HELLFIRE/data/{ID}.txt",
		"LETHALITY": endChecks.FLIGHT,
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

			"CZA": 0.0, # 1/deg
			"CZD": 0.0, # 1/deg
			"CMA": 0.0, # 1/deg
			"CMD": 0.0, # 1/deg
			"CYB": 0.0, # 1/deg
			"CYD": 0.0, # 1/deg
			"CNB": 0.0, # 1/deg
			"CND": 0.0, # 1/deg

			"REF_DIAM": REF_DIAM, # m
			"REF_LENGTH": REF_LNGTH, # m
			"REF_AREA": REF_AREA, # m^2
			"TOF": TOF, # seconds
			"SPEED": SPEED, # m/s
			"ALPHA": ALPHA, # rad
			"SIDESLIP": SIDESLIP, # rad
			"QRATE": QRATE, # rad/s
			"RRATE": RRATE, # rad/s
			"UDOT": SPECIFIC_FORCE[0], # m/s^2
			"VDOT": SPECIFIC_FORCE[1], # m/s^2
			"WDOT": SPECIFIC_FORCE[2], # m/s^2

			"ENUPOSX": ENUPOS[0], # m
			"ENUPOSY": ENUPOS[1], # m
			"ENUPOSZ": ENUPOS[2], # m
			"ENUVELX": ENUVEL[0], # m/s
			"ENUVELY": ENUVEL[1], # m/s
			"ENUVELZ": ENUVEL[2], # m/s
			"ENUTHT": ENUTHT, # rad
			"ENUPSI": ENUPSI, # rad

			"LAT0": GEODETIC0[0], # rad
			"LON0": GEODETIC0[1], # rad
			"ALT0": GEODETIC0[2], # m
			"LAT": GEODETIC[0], # rad
			"LON": GEODETIC[1], # rad
			"ALT": GEODETIC[2], # m

			"ECEF_X0": ECEF0[0], # m
			"ECEF_Y0": ECEF0[1], # m
			"ECEF_Z0": ECEF0[2], # m
			"ECEF_X": ECEF[0], # m
			"ECEF_Y": ECEF[1], # m
			"ECEF_Z": ECEF[2] # m
		}
	}

	LOGFILE = open(MISSILE["LOGFILE"], "w")
	lf.writeHeader(MISSILE["STATE"], LOGFILE)
	lf.writeData(MISSILE["STATE"], LOGFILE)
	LOGFILE.close()

	return MISSILE

def fly_msl(
	MISSILE_INPUT_DICT, # state
	FLY_FOR_THIS_LONG, # seconds
	PITCH_FIN_DEFL_DEG_INPUT, # degrees
	YAW_FIN_DEFL_DEG_INPUT # degrees
):

	# HANDLE INPUT. ################################################################
	MSL     = copy.deepcopy(MISSILE_INPUT_DICT) # state
	LOGFILE = open(MSL["LOGFILE"], "a")

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

	CZA = MSL["STATE"]["CZA"] # 1/deg
	CZD = MSL["STATE"]["CZD"] # 1/deg
	CMA = MSL["STATE"]["CMA"] # 1/deg
	CMD = MSL["STATE"]["CMD"] # 1/deg
	CYB = MSL["STATE"]["CYB"] # 1/deg
	CYD = MSL["STATE"]["CYD"] # 1/deg
	CNB = MSL["STATE"]["CNB"] # 1/deg
	CND = MSL["STATE"]["CND"] # 1/deg

	TOF           = MSL["STATE"]["TOF"] # seconds
	SPEED         = MSL["STATE"]["SPEED"] # m/s
	ALPHA         = MSL["STATE"]["ALPHA"] # rad
	SIDESLIP      = MSL["STATE"]["SIDESLIP"] # rad
	QRATE         = MSL["STATE"]["QRATE"] # rad/s
	RRATE         = MSL["STATE"]["RRATE"] # rad/s
	SPEC_FORCE    = np.zeros(3) # m/s^2
	SPEC_FORCE[0] = MSL["STATE"]["UDOT"] # m/s^2
	SPEC_FORCE[1] = MSL["STATE"]["VDOT"] # m/s^2
	SPEC_FORCE[2] = MSL["STATE"]["WDOT"] # m/s^2

	ENUPOS    = np.zeros(3) # m
	ENUPOS[0] = MSL["STATE"]["ENUPOSX"] # m
	ENUPOS[1] = MSL["STATE"]["ENUPOSY"] # m
	ENUPOS[2] = MSL["STATE"]["ENUPOSZ"] # m
	ENUVEL    = np.zeros(3) # m/s
	ENUVEL[0] = MSL["STATE"]["ENUVELX"] # m/s
	ENUVEL[1] = MSL["STATE"]["ENUVELY"] # m/s
	ENUVEL[2] = MSL["STATE"]["ENUVELZ"] # m/s
	ENUTHT    = MSL["STATE"]["ENUTHT"] # rad
	ENUPSI    = MSL["STATE"]["ENUPSI"] # rad

	GEODETIC0    = np.zeros(3) # rad, rad, m
	GEODETIC0[0] = MSL["STATE"]["LAT0"] # rad, rad, m
	GEODETIC0[1] = MSL["STATE"]["LON0"] # rad, rad, m
	GEODETIC0[2] = MSL["STATE"]["ALT0"] # rad, rad, m
	GEODETIC     = np.zeros(3) # rad, rad, m
	GEODETIC[0]  = MSL["STATE"]["LAT"] # rad, rad, m
	GEODETIC[1]  = MSL["STATE"]["LON"] # rad, rad, m
	GEODETIC[2]  = MSL["STATE"]["ALT"] # rad, rad, m

	ECEF0    = np.zeros(3) # m
	ECEF0[0] = MSL["STATE"]["ECEF_X0"] # m
	ECEF0[1] = MSL["STATE"]["ECEF_Y0"] # m
	ECEF0[2] = MSL["STATE"]["ECEF_Z0"] # m
	ECEF     = np.zeros(3) # m
	ECEF[0]  = MSL["STATE"]["ECEF_X"] # m
	ECEF[1]  = MSL["STATE"]["ECEF_Y"] # m
	ECEF[2]  = MSL["STATE"]["ECEF_Z"] # m

	# INTEGRATION STATE. ###########################################################
	INTEGRATION_PASS = 0
	STATE_P0   = ENUPOS # m
	STATE_V0   = ENUVEL # m/s
	STATE_THT0 = ENUTHT # rad
	STATE_PSI0 = ENUPSI # rad
	STATE_Q0   = QRATE # rad/s
	STATE_R0   = RRATE # rad/s

	V1  = np.zeros(3) # m/s
	A1  = np.zeros(3) # m/s^2
	Q1  = 0.0 # rad/s
	R1  = 0.0 # rad/s
	QD1 = 0.0 # rad/s^2
	RD1 = 0.0 # rad/s^2

	V2  = np.zeros(3) # m/s
	A2  = np.zeros(3) # m/s^2
	Q2  = 0.0 # rad/s
	R2  = 0.0 # rad/s
	QD2 = 0.0 # rad/s^2
	RD2 = 0.0 # rad/s^2

	V3  = np.zeros(3) # m/s
	A3  = np.zeros(3) # m/s^2
	Q3  = 0.0 # rad/s
	R3  = 0.0 # rad/s
	QD3 = 0.0 # rad/s^2
	RD3 = 0.0 # rad/s^2

	V4  = np.zeros(3) # m/s
	A4  = np.zeros(3) # m/s^2
	Q4  = 0.0 # rad/s
	R4  = 0.0 # rad/s
	QD4 = 0.0 # rad/s^2
	RD4 = 0.0 # rad/s^2

	# AIRFRAME. ####################################################################
	CD_LOOKUP          = [0.1, 0.75] # nd
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
	XHL                = 1.8059 - NOSE_LNGTH # m
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

			"CZA": CZA, # 1/deg
			"CZD": CZD, # 1/deg
			"CMA": CMA, # 1/deg
			"CMD": CMD, # 1/deg
			"CYB": CYB, # 1/deg
			"CYD": CYD, # 1/deg
			"CNB": CNB, # 1/deg
			"CND": CND, # 1/deg

			"REF_DIAM": REF_DIAM, # m
			"REF_LENGTH": REF_LNGTH, # m
			"REF_AREA": REF_AREA, # m^2
			"TOF": TOF, # seconds
			"SPEED": SPEED, # m/s
			"ALPHA": ALPHA, # rad
			"SIDESLIP": SIDESLIP, # rad
			"QRATE": QRATE, # rad/s
			"RRATE": RRATE, # rad/s
			"UDOT": SPEC_FORCE[0], # m/s^2
			"VDOT": SPEC_FORCE[1], # m/s^2
			"WDOT": SPEC_FORCE[2], # m/s^2

			"ENUPOSX": ENUPOS[0], # m
			"ENUPOSY": ENUPOS[1], # m
			"ENUPOSZ": ENUPOS[2], # m
			"ENUVELX": ENUVEL[0], # m/s
			"ENUVELY": ENUVEL[1], # m/s
			"ENUVELZ": ENUVEL[2], # m/s
			"ENUTHT": ENUTHT, # rad
			"ENUPSI": ENUPSI, # rad

			"LAT0": GEODETIC0[0], # rad
			"LON0": GEODETIC0[1], # rad
			"ALT0": GEODETIC0[2], # m
			"LAT": GEODETIC[0], # rad
			"LON": GEODETIC[1], # rad
			"ALT": GEODETIC[2], # m

			"ECEF_X0": ECEF0[0], # m
			"ECEF_Y0": ECEF0[1], # m
			"ECEF_Z0": ECEF0[2], # m
			"ECEF_X": ECEF[0], # m
			"ECEF_Y": ECEF[1], # m
			"ECEF_Z": ECEF[2] # m
		}
		return STATE

	# LOOP. ########################################################################
	GO = True
	while GO:

		# ATMOSPHERE. ##############################################################
		MSL["ATMOS"].update(GEODETIC[2], SPEED)
		RHO  = MSL["ATMOS"].rho # kg/m^3
		Q    = MSL["ATMOS"].q # pa
		P    = MSL["ATMOS"].p # pa
		A    = MSL["ATMOS"].a # m/s
		G    = MSL["ATMOS"].g # m/s^2
		MACH = MSL["ATMOS"].mach # nd

		# "Normalized Speed" - Zarchan. nd (KLUDGE)
		if MACH > 1:
			BETA = np.sqrt(MACH ** 2 - 1)
		else:
			BETA = np.sqrt(1.001 ** 2 - 1)

		# MASS AND MOTOR PROPERTIES. ###############################################
		MSL["MASS_AND_MOTOR"].update(TOF, P)
		XCG    = MSL["MASS_AND_MOTOR"].XCG # m
		MASS   = MSL["MASS_AND_MOTOR"].MASS # kg
		TMOI   = MSL["MASS_AND_MOTOR"].TRANSVERSE_MOI # kg*m^2
		THRUST = MSL["MASS_AND_MOTOR"].THRUST # newtons

		# ATTITUDE. ################################################################
		ENU_TO_FLU   = ct.FLIGHTPATH_TO_LOCAL_TM(ENUPSI, -1.0 * ENUTHT) # nd
		SPEED        = la.norm(ENUVEL) # m/s
		VEL_B        = ENU_TO_FLU @ ENUVEL # m/s
		TEMP1, TEMP2 = getAlphaAndBeta(VEL_B) # rad

		# ALPHA AND BETA IN FORWARD, RIGHT, DOWN. ##################################
		ALPHA    = -1.0 * TEMP1 # rad
		SIDESLIP = -1.0 * TEMP2 # rad

		# SET APART FOR COMPONENTIZATION. ##########################################
		# BASIC DRAG MODEL. ########################################################
		CD = None # nd
		if MACH >= MACH_LOOKUP[0]:
			CD = linearInterpolation(MACH, MACH_LOOKUP, CD_LOOKUP)
		else:
			CD = CD_LOOKUP[0]
		DRAG_FORCE      = CD * REF_AREA * Q # newtons
		WIND_TO_BODY    = ct.FLIGHTPATH_TO_LOCAL_TM(SIDESLIP, ALPHA) # nd
		WIND_DRAG_FORCE = npa([-DRAG_FORCE, 0.0, 0.0]) # newtons
		BODY_DRAG       = (WIND_TO_BODY @ WIND_DRAG_FORCE) / MASS # m/s^2

		# AERODYNAMICS. ############################################################
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
			(BETA * REF_AREA)) * ((XCG - XHL) / REF_DIAM) # nd

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
			(BETA * REF_AREA)) * ((XCG - XHL) / REF_DIAM) # nd

		# AERODYNAMIC DERIVATIVES. #################################################
		CZA = 2 + \
			(1.5 * PLANFORM_AREA * ALPHA / REF_AREA) + \
			(8 * WNG_AREA / (BETA * REF_AREA)) + \
			(8 * TAIL_AREA / (BETA * REF_AREA)) # 1/deg
		CZD = (8 * TAIL_AREA / (BETA * REF_AREA)) # 1/deg
		CMA = (2 * (XCG - XCP_NOSE) / REF_DIAM) + \
			(1.5 * PLANFORM_AREA * ALPHA / REF_AREA) * \
			((XCG - XCP_BODY) / REF_DIAM) + \
			(8 * WNG_AREA / (BETA * REF_AREA)) * \
			((XCG - XCP_WNG) / REF_DIAM) + \
			(8 * TAIL_AREA / (BETA * REF_AREA)) * \
			((XCG - XHL) / REF_DIAM) # 1/deg
		CMD = (8 * TAIL_AREA / (BETA * REF_AREA)) * \
			((XCG - XHL) / REF_DIAM) # 1/deg

		CYB = 2 + \
			(1.5 * PLANFORM_AREA * SIDESLIP / REF_AREA) + \
			(8 * WNG_AREA / (BETA * REF_AREA)) + \
			(8 * TAIL_AREA / (BETA * REF_AREA)) # 1/deg
		CYD = (8 * TAIL_AREA / (BETA * REF_AREA)) # 1/deg
		CNB = (2 * (XCG - XCP_NOSE) / REF_DIAM) + \
			(1.5 * PLANFORM_AREA * SIDESLIP / REF_AREA) * \
			((XCG - XCP_BODY) / REF_DIAM) + \
			(8 * WNG_AREA / (BETA * REF_AREA)) * \
			((XCG - XCP_WNG) / REF_DIAM) + \
			(8 * TAIL_AREA / (BETA * REF_AREA)) * \
			((XCG - XHL) / REF_DIAM) # 1/deg
		CND = (8 * TAIL_AREA / (BETA * REF_AREA)) * \
			((XCG - XHL) / REF_DIAM) # 1/deg
		############################################################################
		# END SET APART. ###########################################################

		# DERIVATIVES. #############################################################
		QDOT = (Q * REF_AREA * REF_DIAM * CM) / TMOI # rad/s^2
		RDOT = (Q * REF_AREA * REF_DIAM * CN) / TMOI # rad/s^2

		SPEC_FORCE[0] = THRUST / MASS # m/s^2
		SPEC_FORCE[1] = (Q * REF_AREA * CY) / MASS # m/s^2
		SPEC_FORCE[2] = (Q * REF_AREA * CZ) / MASS # m/s^2

		LOCAL_G       = npa([0.0, 0.0, -1.0 * G]) # m/s^2
		BODY_G        = ENU_TO_FLU @ LOCAL_G # m/s^2
		SPEC_FORCE    += (BODY_G + BODY_DRAG) # m/s^2

		ACC           = SPEC_FORCE @ ENU_TO_FLU # m/s^2

		# GEOMETRY #################################################################
		T1 = np.cos(GEODETIC0[0]) * ENUPOS[2] - np.sin(GEODETIC0[0]) * ENUPOS[1]
		T2 = np.sin(GEODETIC0[0]) * ENUPOS[2] + np.cos(GEODETIC0[0]) * ENUPOS[1]
		T3 = np.cos(GEODETIC0[1]) * T1 - np.sin(GEODETIC0[1]) * ENUPOS[0]
		T4 = np.sin(GEODETIC0[1]) * T1 + np.cos(GEODETIC0[1]) * ENUPOS[0]
		ECEF = ECEF0 + npa([T3, T4, T2]) # m

		GEODETIC = et.ECI_TO_LLA(ECEF) # rad, rad, m

		# STATE. ###################################################################
		if INTEGRATION_PASS == 0:

			# LOG DATA. ############################################################
			MSL["STATE"] = get_state()
			lf.writeData(MSL["STATE"], LOGFILE)

			# END CHECK. ###########################################################
			if TOF > MAX_TIME:
				MSL["LETHALITY"] = endChecks.TIME
				LOGFILE.close()
				return MSL
			if ENUPOS[2] < 0.0:
				print(f"GROUND - TOF : {TOF:.2f}, ENU : {ENUPOS}, MACH : {MACH:.2f}")
				MSL["LETHALITY"] = endChecks.GROUND
				LOGFILE.close()
				return MSL
			if np.isnan(np.sum(ENUPOS)):
				print(f"NAN - TOF : {TOF:.2f}, ENU : {ENUPOS}, MACH : {MACH:.2f}")
				MSL["LETHALITY"] = endChecks.NAN
				LOGFILE.close()
				return MSL

			# BEGIN INTEGRATION PASS. ##############################################
			STATE_P0 = copy.deepcopy(ENUPOS) # m
			STATE_V0 = copy.deepcopy(ENUVEL) # m/s
			STATE_THT0 = copy.deepcopy(ENUTHT) # rad
			STATE_PSI0 = copy.deepcopy(ENUPSI) # rad
			STATE_Q0 = copy.deepcopy(QRATE) # rad/s
			STATE_R0 = copy.deepcopy(RRATE) # rad/s

			V1  = ENUVEL # m/s
			A1  = ACC # m/s^2
			Q1  = QRATE # rad/s
			R1  = RRATE # rad/s
			QD1 = QDOT # rad/s^2
			RD1 = RDOT # rad/s^2

			ENUPOS = STATE_P0 + V1 * (TIME_STEP / 2.0) # m
			ENUVEL = STATE_V0 + A1 * (TIME_STEP / 2.0) # m/s
			ENUTHT = STATE_THT0 + Q1 * (TIME_STEP / 2.0) # rad
			ENUPSI = STATE_PSI0 + R1 * (TIME_STEP / 2.0) # rad
			QRATE  = STATE_Q0 + QD1 * (TIME_STEP / 2.0) # rad/s
			RRATE  = STATE_R0 + RD1 * (TIME_STEP / 2.0) # rad/s

			TOF += (TIME_STEP / 2.0) # seconds

			INTEGRATION_PASS += 1

		elif INTEGRATION_PASS == 1:

			V2  = ENUVEL # m/s
			A2  = ACC # m/s^2
			Q2  = QRATE # rad/s
			R2  = RRATE # rad/s
			QD2 = QDOT # rad/s^2
			RD2 = RDOT # rad/s^2

			ENUPOS = STATE_P0 + V2 * (TIME_STEP / 2.0) # m
			ENUVEL = STATE_V0 + A2 * (TIME_STEP / 2.0) # m/s
			ENUTHT = STATE_THT0 + Q2 * (TIME_STEP / 2.0) # rad
			ENUPSI = STATE_PSI0 + R2 * (TIME_STEP / 2.0) # rad
			QRATE  = STATE_Q0 + QD2 * (TIME_STEP / 2.0) # rad/s^2
			RRATE  = STATE_R0 + RD2 * (TIME_STEP / 2.0) # rad/s^2

			INTEGRATION_PASS += 1

		elif INTEGRATION_PASS == 2:

			V3  = ENUVEL # m/s
			A3  = ACC # m/s^2
			Q3  = QRATE # rad/s
			R3  = RRATE # rad/s
			QD3 = QDOT # rad/s^2
			RD3 = RDOT # rad/s^2

			ENUPOS = STATE_P0 + V3 * (TIME_STEP) # m
			ENUVEL = STATE_V0 + A3 * (TIME_STEP) # m/s
			ENUTHT = STATE_THT0 + Q3 * (TIME_STEP) # rad
			ENUPSI = STATE_PSI0 + R3 * (TIME_STEP) # rad
			QRATE  = STATE_Q0 + QD3 * (TIME_STEP) # rad/s^2
			RRATE  = STATE_R0 + RD3 * (TIME_STEP) # rad/s^2

			TOF += (TIME_STEP / 2.0) # seconds

			INTEGRATION_PASS += 1

		elif INTEGRATION_PASS == 3:

			V4  = ENUVEL # m/s
			A4  = ACC # m/s^2
			Q4  = QRATE # rad/s
			R4  = RRATE # rad/s
			QD4 = QDOT # rad/s^2
			RD4 = RDOT # rad/s^2

			ENUPOS   = STATE_P0 + (TIME_STEP / 6.0) * \
				(V1 + 2 * V2 + 2 * V3 + V4) # m
			ENUVEL   = STATE_V0 + (TIME_STEP / 6.0) * \
				(A1 + 2 * A2 + 2 * A3 + A4) # m/s
			ENUTHT = STATE_THT0 + (TIME_STEP / 6.0) * \
				(Q1 + 2 * Q2 + 2 * Q3 + Q4) # rad
			ENUPSI = STATE_PSI0 + (TIME_STEP / 6.0) * \
				(R1 + 2 * R2 + 2 * R3 + R4) # rad/s
			QRATE = STATE_Q0 + (TIME_STEP / 6.0) * \
				(QD1 + 2 * QD2 + 2 * QD3 + QD4)
			RRATE = STATE_R0 + (TIME_STEP / 6.0) * \
				(RD1 + 2 * RD2 + 2 * RD3 + RD4)

			INTEGRATION_PASS = 0



if __name__ == "__main__":

	wallClockStart = time.time()

	LLA0 = npa([38.8719, 77.0563, 0.0])
	AZ0  = 0
	EL0  = 45
	SPD0 = 10
	ID   = "MOCK_HELLFIRE5DOF"
	MSL  = construct_msl(LLA0, AZ0, EL0, SPD0, ID)
	MSL  = fly_msl(MSL, 100, -4.0, 0.0)

	wallClockEnd = time.time()
	print(f"RUN TIME : {wallClockEnd - wallClockStart} SECONDS")
