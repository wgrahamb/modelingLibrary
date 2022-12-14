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
from   utility.earthTransforms              import *

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

	# FRAMES. ######################################################################
	# LLA.
	GEODETIC0 = npa(
		[
			np.radians(INITIAL_LLA[0]), # rad
			np.radians(INITIAL_LLA[1]), # rad
			INITIAL_LLA[2] # meters
		]
	)
	GEODETIC = copy.deepcopy(GEODETIC0)

	# ENU.
	ENU_TO_FLU = ct.ORIENTATION_TO_LOCAL_TM(0.0, -INITIAL_EL, INITIAL_AZ) # nd
	ENUPOS     = np.zeros(3) # m
	ENUVEL     = INITIAL_AIRSPEED * (ENU_TO_FLU[0]) # m/s
	ENUEULER   = npa([0.0, INITIAL_EL, INITIAL_AZ]) # rad

	# ECEF.
	ECEFPOS     = et.LLA_TO_ECI(GEODETIC, 0.0) # m
	ECEFPOS0    = copy.deepcopy(ECEFPOS) # m
	ECEF_TO_ENU = ct.ORIENTATION_TO_LOCAL_TM(
		0.0,
		(np.pi / 2.0) + GEODETIC[0], # rad
		GEODETIC[1] # rad
	) # nd
	ECEF_TO_FLU = ENU_TO_FLU @ ECEF_TO_ENU # nd
	ECEFVEL     = (ENU_TO_FLU @ (INITIAL_AIRSPEED * (ENU_TO_FLU[0]))) @ \
		ECEF_TO_FLU # m/s

	# ECI.
	ECIPOS                 = et.LLA_TO_ECI(GEODETIC, 0.0) # m
	ECI_TO_ECEF            = et.ECI_TO_ECEF_TM(0.0) # nd
	TEMP                   = ECI_TO_ECEF.transpose() @ ECEFVEL # m/s
	OMEGA                  = npa([0.0, 0.0, WEII3]) # rad/s
	ECIVEL_DUE_TO_ROTATION = np.cross(OMEGA, ECIPOS) # nd
	ECIVEL                 = TEMP + ECIVEL_DUE_TO_ROTATION # m/s
	ECI_TO_FLU             = ECEF_TO_FLU @ ECI_TO_ECEF # nd

	# BODY. ########################################################################
	TOF            = 0.0 # seconds
	SPEED          = la.norm(INITIAL_AIRSPEED * (ENU_TO_FLU[0])) # m/s
	VEL_B          = ENU_TO_FLU @ (INITIAL_AIRSPEED * (ENU_TO_FLU[0])) # m/s
	TEMP1, TEMP2   = getAlphaAndBeta(VEL_B) # rad
	ALPHA          = -1.0 * TEMP1 # rad
	SIDESLIP       = -1.0 * TEMP2 # rad
	SPECIFIC_FORCE = np.zeros(3) # m/s^2
	BODYRATE       = np.zeros(3) # rad/s

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

			"TOF": TOF, # seconds
			"SPEED": SPEED, # m/s
			"ALPHA": ALPHA, # rad
			"SIDESLIP": SIDESLIP, # rad
			"PRATE": BODYRATE[0], # rad/s
			"QRATE": BODYRATE[1], # rad/s
			"RRATE": BODYRATE[2], # rad/s
			"UDOT": SPECIFIC_FORCE[0], # m/s^2
			"VDOT": SPECIFIC_FORCE[1], # m/s^2
			"WDOT": SPECIFIC_FORCE[2], # m/s^2

			"LAT0": GEODETIC0[0], # rad
			"LON0": GEODETIC0[1], # rad
			"ALT0": GEODETIC0[2], # meters
			"LAT": GEODETIC[0], # rad
			"LON": GEODETIC[1], # rad
			"ALT": GEODETIC[2], # meters

			"ENU_TO_FLU_XX": ENU_TO_FLU[0, 0], # nd
			"ENU_TO_FLU_XY": ENU_TO_FLU[0, 1], # nd
			"ENU_TO_FLU_XZ": ENU_TO_FLU[0, 2], # nd
			"ENU_TO_FLU_YX": ENU_TO_FLU[1, 0], # nd
			"ENU_TO_FLU_YY": ENU_TO_FLU[1, 1], # nd
			"ENU_TO_FLU_YZ": ENU_TO_FLU[1, 2], # nd
			"ENU_TO_FLU_ZX": ENU_TO_FLU[2, 0], # nd
			"ENU_TO_FLU_ZY": ENU_TO_FLU[2, 1], # nd
			"ENU_TO_FLU_ZZ": ENU_TO_FLU[2, 2], # nd
			"ENUPOSX": ENUPOS[0], # m
			"ENUPOSY": ENUPOS[1], # m
			"ENUPOSZ": ENUPOS[2], # m
			"ENUVELX": ENUVEL[0], # m/s
			"ENUVELY": ENUVEL[1], # m/s
			"ENUVELZ": ENUVEL[2], # m/s
			"ENUPHI": ENUEULER[0], # rad
			"ENUTHT": ENUEULER[1], # rad
			"ENUPSI": ENUEULER[2], # rad

			"ECEFPOSY": ECEFPOS[0], # m
			"ECEFPOSX": ECEFPOS[1], # m
			"ECEFPOSZ": ECEFPOS[2], # m
			"ECEFPOSY0": ECEFPOS0[0], # m
			"ECEFPOSX0": ECEFPOS0[1], # m
			"ECEFPOSZ0": ECEFPOS0[2], # m
			"ECEF_TO_ENU_XX": ECEF_TO_ENU[0, 0], # nd
			"ECEF_TO_ENU_XY": ECEF_TO_ENU[0, 1], # nd
			"ECEF_TO_ENU_XZ": ECEF_TO_ENU[0, 2], # nd
			"ECEF_TO_ENU_YX": ECEF_TO_ENU[1, 0], # nd
			"ECEF_TO_ENU_YY": ECEF_TO_ENU[1, 1], # nd
			"ECEF_TO_ENU_YZ": ECEF_TO_ENU[1, 2], # nd
			"ECEF_TO_ENU_ZX": ECEF_TO_ENU[2, 0], # nd
			"ECEF_TO_ENU_ZY": ECEF_TO_ENU[2, 1], # nd
			"ECEF_TO_ENU_ZZ": ECEF_TO_ENU[2, 2], # nd
			"ECEF_TO_FLU_XX": ECEF_TO_FLU[0, 0], # nd
			"ECEF_TO_FLU_XY": ECEF_TO_FLU[0, 1], # nd
			"ECEF_TO_FLU_XZ": ECEF_TO_FLU[0, 2], # nd
			"ECEF_TO_FLU_YX": ECEF_TO_FLU[1, 0], # nd
			"ECEF_TO_FLU_YY": ECEF_TO_FLU[1, 1], # nd
			"ECEF_TO_FLU_YZ": ECEF_TO_FLU[1, 2], # nd
			"ECEF_TO_FLU_ZX": ECEF_TO_FLU[2, 0], # nd
			"ECEF_TO_FLU_ZY": ECEF_TO_FLU[2, 1], # nd
			"ECEF_TO_FLU_ZZ": ECEF_TO_FLU[2, 2], # nd
			"ECEFVELX": ECEFVEL[0], # m/s
			"ECEFVELY": ECEFVEL[1], # m/s
			"ECEFVELZ": ECEFVEL[2], # m/s

			"ECIPOSX": ECIPOS[0], # m
			"ECIPOSY": ECIPOS[1], # m
			"ECIPOSZ": ECIPOS[2], # m
			"ECI_TO_ECEF_XX": ECI_TO_ECEF[0, 0], # nd
			"ECI_TO_ECEF_XY": ECI_TO_ECEF[0, 1], # nd
			"ECI_TO_ECEF_XZ": ECI_TO_ECEF[0, 2], # nd
			"ECI_TO_ECEF_YX": ECI_TO_ECEF[1, 0], # nd
			"ECI_TO_ECEF_YY": ECI_TO_ECEF[1, 1], # nd
			"ECI_TO_ECEF_YZ": ECI_TO_ECEF[1, 2], # nd
			"ECI_TO_ECEF_ZX": ECI_TO_ECEF[2, 0], # nd
			"ECI_TO_ECEF_ZY": ECI_TO_ECEF[2, 1], # nd
			"ECI_TO_ECEF_ZZ": ECI_TO_ECEF[2, 2], # nd
			"ECIVELX": ECIVEL[0], # m/s
			"ECIVELY": ECIVEL[1], # m/s
			"ECIVELZ": ECIVEL[2], # m/s
			"ECI_TO_FLU_XX": ECI_TO_FLU[0, 0], # nd
			"ECI_TO_FLU_XY": ECI_TO_FLU[0, 1], # nd
			"ECI_TO_FLU_XZ": ECI_TO_FLU[0, 2], # nd
			"ECI_TO_FLU_YX": ECI_TO_FLU[1, 0], # nd
			"ECI_TO_FLU_YY": ECI_TO_FLU[1, 1], # nd
			"ECI_TO_FLU_YZ": ECI_TO_FLU[1, 2], # nd
			"ECI_TO_FLU_ZX": ECI_TO_FLU[2, 0], # nd
			"ECI_TO_FLU_ZY": ECI_TO_FLU[2, 1], # nd
			"ECI_TO_FLU_ZZ": ECI_TO_FLU[2, 2], # nd
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

	TOF           = MSL["STATE"]["TOF"] # seconds
	SPEED         = MSL["STATE"]["SPEED"] # m/s
	ALPHA         = MSL["STATE"]["ALPHA"] # rad
	SIDESLIP      = MSL["STATE"]["SIDESLIP"] # rad
	BODYRATE      = np.zeros(3) # rad/s
	BODYRATE[0]   = MSL["STATE"]["PRATE"] # rad/s
	BODYRATE[1]   = MSL["STATE"]["QRATE"] # rad/s
	BODYRATE[2]   = MSL["STATE"]["RRATE"] # rad/s
	SPEC_FORCE    = np.zeros(3) # m/s^2
	SPEC_FORCE[0] = MSL["STATE"]["UDOT"] # m/s^2
	SPEC_FORCE[1] = MSL["STATE"]["VDOT"] # m/s^2
	SPEC_FORCE[2] = MSL["STATE"]["WDOT"] # m/s^2

	GEODETIC0    = np.zeros(3) # lla
	GEODETIC0[0] = MSL["STATE"]["LAT0"] # rad
	GEODETIC0[1] = MSL["STATE"]["LON0"] # rad
	GEODETIC0[2] = MSL["STATE"]["ALT0"] # m
	GEODETIC     = np.zeros(3) # lla
	GEODETIC[0]  = MSL["STATE"]["LAT"] # rad
	GEODETIC[1]  = MSL["STATE"]["LON"] # rad
	GEODETIC[2]  = MSL["STATE"]["ALT"] # m

	ENU_TO_FLU       = np.zeros((3,3)) # nd
	ENU_TO_FLU[0, 0] = MSL["STATE"]["ENU_TO_FLU_XX"] # nd
	ENU_TO_FLU[0, 1] = MSL["STATE"]["ENU_TO_FLU_XY"] # nd
	ENU_TO_FLU[0, 2] = MSL["STATE"]["ENU_TO_FLU_XZ"] # nd
	ENU_TO_FLU[1, 0] = MSL["STATE"]["ENU_TO_FLU_YX"] # nd
	ENU_TO_FLU[1, 1] = MSL["STATE"]["ENU_TO_FLU_YY"] # nd
	ENU_TO_FLU[1, 2] = MSL["STATE"]["ENU_TO_FLU_YZ"] # nd
	ENU_TO_FLU[2, 0] = MSL["STATE"]["ENU_TO_FLU_ZX"] # nd
	ENU_TO_FLU[2, 1] = MSL["STATE"]["ENU_TO_FLU_ZY"] # nd
	ENU_TO_FLU[2, 2] = MSL["STATE"]["ENU_TO_FLU_ZZ"] # nd
	ENUPOS           = np.zeros(3) # m
	ENUPOS[0]        = MSL["STATE"]["ENUPOSX"] # m
	ENUPOS[1]        = MSL["STATE"]["ENUPOSY"] # m
	ENUPOS[2]        = MSL["STATE"]["ENUPOSZ"] # m
	ENUVEL           = np.zeros(3) # m/s
	ENUVEL[0]        = MSL["STATE"]["ENUVELX"] # m/s
	ENUVEL[1]        = MSL["STATE"]["ENUVELY"] # m/s
	ENUVEL[2]        = MSL["STATE"]["ENUVELZ"] # m/s
	ENUEULER         = np.zeros(3) # rad
	ENUEULER[0]      = MSL["STATE"]["ENUPHI"] # rad
	ENUEULER[1]      = MSL["STATE"]["ENUTHT"] # rad
	ENUEULER[2]      = MSL["STATE"]["ENUPSI"] # rad

	ECEFPOS           = np.zeros(3) # m
	ECEFPOS[0]        = MSL["STATE"]["ECEFPOSX"] # m
	ECEFPOS[1]        = MSL["STATE"]["ECEFPOSY"] # m
	ECEFPOS[2]        = MSL["STATE"]["ECEFPOSZ"] # m
	ECEFPOS0          = np.zeros(3) # m
	ECEFPOS0[0]       = MSL["STATE"]["ECEFPOSX0"] # m
	ECEFPOS0[1]       = MSL["STATE"]["ECEFPOSY0"] # m
	ECEFPOS0[2]       = MSL["STATE"]["ECEFPOSZ0"] # m
	ECEF_TO_ENU       = np.zeros((3,3)) # nd
	ECEF_TO_ENU[0, 0] = MSL["STATE"]["ECEF_TO_ENU_XX"] # nd
	ECEF_TO_ENU[0, 1] = MSL["STATE"]["ECEF_TO_ENU_XY"] # nd
	ECEF_TO_ENU[0, 2] = MSL["STATE"]["ECEF_TO_ENU_XZ"] # nd
	ECEF_TO_ENU[1, 0] = MSL["STATE"]["ECEF_TO_ENU_YX"] # nd
	ECEF_TO_ENU[1, 1] = MSL["STATE"]["ECEF_TO_ENU_YY"] # nd
	ECEF_TO_ENU[1, 2] = MSL["STATE"]["ECEF_TO_ENU_YZ"] # nd
	ECEF_TO_ENU[2, 0] = MSL["STATE"]["ECEF_TO_ENU_ZX"] # nd
	ECEF_TO_ENU[2, 1] = MSL["STATE"]["ECEF_TO_ENU_ZY"] # nd
	ECEF_TO_ENU[2, 2] = MSL["STATE"]["ECEF_TO_ENU_ZZ"] # nd
	ECEF_TO_FLU       = np.zeros((3,3)) # nd
	ECEF_TO_FLU[0, 0] = MSL["STATE"]["ECEF_TO_FLU_XX"] # nd
	ECEF_TO_FLU[0, 1] = MSL["STATE"]["ECEF_TO_FLU_XY"] # nd
	ECEF_TO_FLU[0, 2] = MSL["STATE"]["ECEF_TO_FLU_XZ"] # nd
	ECEF_TO_FLU[1, 0] = MSL["STATE"]["ECEF_TO_FLU_YX"] # nd
	ECEF_TO_FLU[1, 1] = MSL["STATE"]["ECEF_TO_FLU_YY"] # nd
	ECEF_TO_FLU[1, 2] = MSL["STATE"]["ECEF_TO_FLU_YZ"] # nd
	ECEF_TO_FLU[2, 0] = MSL["STATE"]["ECEF_TO_FLU_ZX"] # nd
	ECEF_TO_FLU[2, 1] = MSL["STATE"]["ECEF_TO_FLU_ZY"] # nd
	ECEF_TO_FLU[2, 2] = MSL["STATE"]["ECEF_TO_FLU_ZZ"] # nd
	ECEFVEL           = np.zeros(3) # m/s
	ECEFVEL[0]        = MSL["STATE"]["ECEFVELX"] # m/s
	ECEFVEL[1]        = MSL["STATE"]["ECEFVELY"] # m/s
	ECEFVEL[2]        = MSL["STATE"]["ECEFVELZ"] # m/s

	ECIPOS            = np.zeros(3) # m
	ECIPOS[0]         = MSL["STATE"]["ECIPOSX"] # m
	ECIPOS[1]         = MSL["STATE"]["ECIPOSY"] # m
	ECIPOS[2]         = MSL["STATE"]["ECIPOSZ"] # m
	ECI_TO_ECEF       = np.zeros((3, 3)) # nd
	ECI_TO_ECEF[0, 0] = MSL["STATE"]["ECI_TO_ECEF_XX"] # nd
	ECI_TO_ECEF[0, 1] = MSL["STATE"]["ECI_TO_ECEF_XY"] # nd
	ECI_TO_ECEF[0, 2] = MSL["STATE"]["ECI_TO_ECEF_XZ"] # nd
	ECI_TO_ECEF[1, 0] = MSL["STATE"]["ECI_TO_ECEF_YX"] # nd
	ECI_TO_ECEF[1, 1] = MSL["STATE"]["ECI_TO_ECEF_YY"] # nd
	ECI_TO_ECEF[1, 2] = MSL["STATE"]["ECI_TO_ECEF_YZ"] # nd
	ECI_TO_ECEF[2, 0] = MSL["STATE"]["ECI_TO_ECEF_ZX"] # nd
	ECI_TO_ECEF[2, 1] = MSL["STATE"]["ECI_TO_ECEF_ZY"] # nd
	ECI_TO_ECEF[2, 2] = MSL["STATE"]["ECI_TO_ECEF_ZZ"] # nd
	ECIVEL            = np.zeros(3) # m/s
	ECIVEL[0]         = MSL["STATE"]["ECIVELX"] # m/s
	ECIVEL[1]         = MSL["STATE"]["ECIVELY"] # m/s
	ECIVEL[2]         = MSL["STATE"]["ECIVELZ"] # m/s
	ECI_TO_FLU        = np.zeros((3, 3)) # nd
	ECI_TO_FLU[0, 0]  = MSL["STATE"]["ECI_TO_FLU_XX"] # nd
	ECI_TO_FLU[0, 1]  = MSL["STATE"]["ECI_TO_FLU_XY"] # nd
	ECI_TO_FLU[0, 2]  = MSL["STATE"]["ECI_TO_FLU_XZ"] # nd
	ECI_TO_FLU[1, 0]  = MSL["STATE"]["ECI_TO_FLU_YX"] # nd
	ECI_TO_FLU[1, 1]  = MSL["STATE"]["ECI_TO_FLU_YY"] # nd
	ECI_TO_FLU[1, 2]  = MSL["STATE"]["ECI_TO_FLU_YZ"] # nd
	ECI_TO_FLU[2, 0]  = MSL["STATE"]["ECI_TO_FLU_ZX"] # nd
	ECI_TO_FLU[2, 1]  = MSL["STATE"]["ECI_TO_FLU_ZY"] # nd
	ECI_TO_FLU[2, 2]  = MSL["STATE"]["ECI_TO_FLU_ZZ"] # nd

	# INTEGRATION STATE. ###########################################################
	INTEGRATION_PASS = 0
	STATE_P0 = ENUPOS # m
	STATE_V0 = ENUVEL # m/s
	STATE_E0 = ENUEULER # rad/s
	STATE_W0 = BODYRATE # rad/s^2
	V1       = np.zeros(3) # m/s
	A1       = np.zeros(3) # m/s^2
	W1       = np.zeros(3) # rad/s
	WD1      = np.zeros(3) # rad/s^2
	V2       = np.zeros(3) # m/s
	A2       = np.zeros(3) # m/s^2
	W2       = np.zeros(3) # rad/s
	WD2      = np.zeros(3) # rad/s^2
	V3       = np.zeros(3) # m/s
	A3       = np.zeros(3) # m/s^2
	W3       = np.zeros(3) # rad/s
	WD3      = np.zeros(3) # rad/s^2
	V4       = np.zeros(3) # m/s
	A4       = np.zeros(3) # m/s^2
	W4       = np.zeros(3) # rad/s
	WD4      = np.zeros(3) # rad/s^2

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
			"UDOT": SPEC_FORCE[0], # m/s^2
			"VDOT": SPEC_FORCE[1], # m/s^2
			"WDOT": SPEC_FORCE[2], # m/s^2

			"LAT0": GEODETIC0[0], # rad
			"LON0": GEODETIC0[1], # rad
			"ALT0": GEODETIC0[2], # meters
			"LAT": GEODETIC[0], # rad
			"LON": GEODETIC[1], # rad
			"ALT": GEODETIC[2], # meters

			"ENU_TO_FLU_XX": ENU_TO_FLU[0, 0], # nd
			"ENU_TO_FLU_XY": ENU_TO_FLU[0, 1], # nd
			"ENU_TO_FLU_XZ": ENU_TO_FLU[0, 2], # nd
			"ENU_TO_FLU_YX": ENU_TO_FLU[1, 0], # nd
			"ENU_TO_FLU_YY": ENU_TO_FLU[1, 1], # nd
			"ENU_TO_FLU_YZ": ENU_TO_FLU[1, 2], # nd
			"ENU_TO_FLU_ZX": ENU_TO_FLU[2, 0], # nd
			"ENU_TO_FLU_ZY": ENU_TO_FLU[2, 1], # nd
			"ENU_TO_FLU_ZZ": ENU_TO_FLU[2, 2], # nd
			"ENUPOSX": ENUPOS[0], # m
			"ENUPOSY": ENUPOS[1], # m
			"ENUPOSZ": ENUPOS[2], # m
			"ENUVELX": ENUVEL[0], # m/s
			"ENUVELY": ENUVEL[1], # m/s
			"ENUVELZ": ENUVEL[2], # m/s
			"ENUPHI": ENUEULER[0], # rad
			"ENUTHT": ENUEULER[1], # rad
			"ENUPSI": ENUEULER[2], # rad

			"ECEFPOSY": ECEFPOS[0], # m
			"ECEFPOSX": ECEFPOS[1], # m
			"ECEFPOSZ": ECEFPOS[2], # m
			"ECEFPOSY0": ECEFPOS0[0], # m
			"ECEFPOSX0": ECEFPOS0[1], # m
			"ECEFPOSZ0": ECEFPOS0[2], # m
			"ECEF_TO_ENU_XX": ECEF_TO_ENU[0, 0], # nd
			"ECEF_TO_ENU_XY": ECEF_TO_ENU[0, 1], # nd
			"ECEF_TO_ENU_XZ": ECEF_TO_ENU[0, 2], # nd
			"ECEF_TO_ENU_YX": ECEF_TO_ENU[1, 0], # nd
			"ECEF_TO_ENU_YY": ECEF_TO_ENU[1, 1], # nd
			"ECEF_TO_ENU_YZ": ECEF_TO_ENU[1, 2], # nd
			"ECEF_TO_ENU_ZX": ECEF_TO_ENU[2, 0], # nd
			"ECEF_TO_ENU_ZY": ECEF_TO_ENU[2, 1], # nd
			"ECEF_TO_ENU_ZZ": ECEF_TO_ENU[2, 2], # nd
			"ECEF_TO_FLU_XX": ECEF_TO_FLU[0, 0], # nd
			"ECEF_TO_FLU_XY": ECEF_TO_FLU[0, 1], # nd
			"ECEF_TO_FLU_XZ": ECEF_TO_FLU[0, 2], # nd
			"ECEF_TO_FLU_YX": ECEF_TO_FLU[1, 0], # nd
			"ECEF_TO_FLU_YY": ECEF_TO_FLU[1, 1], # nd
			"ECEF_TO_FLU_YZ": ECEF_TO_FLU[1, 2], # nd
			"ECEF_TO_FLU_ZX": ECEF_TO_FLU[2, 0], # nd
			"ECEF_TO_FLU_ZY": ECEF_TO_FLU[2, 1], # nd
			"ECEF_TO_FLU_ZZ": ECEF_TO_FLU[2, 2], # nd
			"ECEFVELX": ECEFVEL[0], # m/s
			"ECEFVELY": ECEFVEL[1], # m/s
			"ECEFVELZ": ECEFVEL[2], # m/s

			"ECIPOSX": ECIPOS[0], # m
			"ECIPOSY": ECIPOS[1], # m
			"ECIPOSZ": ECIPOS[2], # m
			"ECI_TO_ECEF_XX": ECI_TO_ECEF[0, 0], # nd
			"ECI_TO_ECEF_XY": ECI_TO_ECEF[0, 1], # nd
			"ECI_TO_ECEF_XZ": ECI_TO_ECEF[0, 2], # nd
			"ECI_TO_ECEF_YX": ECI_TO_ECEF[1, 0], # nd
			"ECI_TO_ECEF_YY": ECI_TO_ECEF[1, 1], # nd
			"ECI_TO_ECEF_YZ": ECI_TO_ECEF[1, 2], # nd
			"ECI_TO_ECEF_ZX": ECI_TO_ECEF[2, 0], # nd
			"ECI_TO_ECEF_ZY": ECI_TO_ECEF[2, 1], # nd
			"ECI_TO_ECEF_ZZ": ECI_TO_ECEF[2, 2], # nd
			"ECIVELX": ECIVEL[0], # m/s
			"ECIVELY": ECIVEL[1], # m/s
			"ECIVELZ": ECIVEL[2], # m/s
			"ECI_TO_FLU_XX": ECI_TO_FLU[0, 0], # nd
			"ECI_TO_FLU_XY": ECI_TO_FLU[0, 1], # nd
			"ECI_TO_FLU_XZ": ECI_TO_FLU[0, 2], # nd
			"ECI_TO_FLU_YX": ECI_TO_FLU[1, 0], # nd
			"ECI_TO_FLU_YY": ECI_TO_FLU[1, 1], # nd
			"ECI_TO_FLU_YZ": ECI_TO_FLU[1, 2], # nd
			"ECI_TO_FLU_ZX": ECI_TO_FLU[2, 0], # nd
			"ECI_TO_FLU_ZY": ECI_TO_FLU[2, 1], # nd
			"ECI_TO_FLU_ZZ": ECI_TO_FLU[2, 2], # nd
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
		ENU_TO_FLU   = ct.ORIENTATION_TO_LOCAL_TM(
			ENUEULER[0], -1.0 * ENUEULER[1], ENUEULER[2]) # nd
		SPEED        = la.norm(ENUVEL) # m/s
		VEL_B        = ENU_TO_FLU @ ENUVEL # m/s
		TEMP1, TEMP2 = getAlphaAndBeta(VEL_B) # rad

		# ALPHA AND BETA IN FORWARD, RIGHT, DOWN. ##################################
		ALPHA    = -1.0 * TEMP1 # rad
		SIDESLIP = -1.0 * TEMP2 # rad

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
		RATEDOT       = np.zeros(3) # rad/s^2
		RATEDOT[0]    = 0.0 # rad/s^2
		RATEDOT[1]    = (Q * REF_AREA * REF_DIAM * CM) / TMOI # rad/s^2
		RATEDOT[2]    = (Q * REF_AREA * REF_DIAM * CN) / TMOI # rad/s^2

		SPEC_FORCE[0] = THRUST / MASS # m/s^2
		SPEC_FORCE[1] = (Q * REF_AREA * CY) / MASS # m/s^2
		SPEC_FORCE[2] = (Q * REF_AREA * CZ) / MASS # m/s^2

		LOCAL_G       = npa([0.0, 0.0, -1.0 * G]) # m/s^2
		BODY_G        = ENU_TO_FLU @ LOCAL_G # m/s^2
		SPEC_FORCE    += (BODY_G + BODY_DRAG) # m/s^2

		ACC           = SPEC_FORCE @ ENU_TO_FLU # m/s^2

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
			STATE_E0 = copy.deepcopy(ENUEULER) # rad/s
			STATE_W0 = copy.deepcopy(BODYRATE) # rad/s^2

			V1  = ENUVEL # m/s
			A1  = ACC # m/s^2
			W1  = BODYRATE # rad/s
			WD1 = RATEDOT # rad/s^2

			ENUPOS   = STATE_P0 + V1 * (TIME_STEP / 2.0) # m
			ENUVEL   = STATE_V0 + A1 * (TIME_STEP / 2.0) # m/s
			ENUEULER = STATE_E0 + W1 * (TIME_STEP / 2.0) # rad
			BODYRATE = STATE_W0 + WD1 * (TIME_STEP / 2.0) # rad/s

			TOF += (TIME_STEP / 2.0) # seconds

			INTEGRATION_PASS += 1

		elif INTEGRATION_PASS == 1:

			V2  = ENUVEL # m/s
			A2  = ACC # m/s^2
			W2  = BODYRATE # rad/s
			WD2 = RATEDOT # rad/s^2

			ENUPOS   = STATE_P0 + V2 * (TIME_STEP / 2.0) # m
			ENUVEL   = STATE_V0 + A2 * (TIME_STEP / 2.0) # m/s
			ENUEULER = STATE_E0 + W2 * (TIME_STEP / 2.0) # rad
			BODYRATE = STATE_W0 + WD2 * (TIME_STEP / 2.0) # rad/s

			INTEGRATION_PASS += 1

		elif INTEGRATION_PASS == 2:

			V3  = ENUVEL # m/s
			A3  = ACC # m/s^2
			W3  = BODYRATE # rad/s
			WD3 = RATEDOT # rad/s^2

			ENUPOS   = STATE_P0 + V3 * (TIME_STEP) # m
			ENUVEL   = STATE_V0 + A3 * (TIME_STEP) # m/s
			ENUEULER = STATE_E0 + W3 * (TIME_STEP) # rad
			BODYRATE = STATE_W0 + WD3 * (TIME_STEP) # rad/s

			TOF += (TIME_STEP / 2.0) # seconds

			INTEGRATION_PASS += 1

		elif INTEGRATION_PASS == 3:

			V4  = ENUVEL # m/s
			A4  = ACC # m/s^2
			W4  = BODYRATE # rad/s
			WD4 = RATEDOT # rad/s^2

			ENUPOS   = STATE_P0 + (TIME_STEP / 6.0) * \
				(V1 + 2 * V2 + 2 * V3 + V4) # m
			ENUVEL   = STATE_V0 + (TIME_STEP / 6.0) * \
				(A1 + 2 * A2 + 2 * A3 + A4) # m/s
			ENUEULER = STATE_E0 + (TIME_STEP / 6.0) * \
				(W1 + 2 * W2 + 2 * W3 + W4) # rad
			BODYRATE = STATE_W0 + (TIME_STEP / 6.0) * \
				(WD1 + 2 * WD2 + 2 * WD3 + WD4) # rad/s

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
