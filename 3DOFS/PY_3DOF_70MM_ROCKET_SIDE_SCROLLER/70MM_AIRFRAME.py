import numpy as np
from numpy import array as npa
from numpy import linalg as la
import utility.loggingFxns as lf
import utility.coordinateTransformations as ct
from utility.ATM_IMPERIAL import ATM_IMPERIAL
np.set_printoptions(precision=2, suppress=True)

# INPUTS.
ALT = 10000 # FEET
SPD = 1200 # FEET PER SEC
DEFL_D = 0 # DEGREES
DEFL = np.radians(DEFL_D) # RADIANS

# AIRFRAME.
REF_DIAM = 0.23 # FEET
NOSE_LNGTH = 1.11 # FEET
REF_LNGTH = 4.59 # FEET
WNG_HLF_SPN = 0 # FEET
WNG_TIP_CHRD = 0 # FEET
WNG_ROOT_CHRD = 0 # FEET
TAIL_HLF_SPN = 0.04 # FEET
TAIL_TIP_CHRD = 0.27 # FEET
TAIL_ROOT_CHRD = 0.27 # FEET
BASE_OF_NOSE_TO_WNG = 0 # FEET
XCG = REF_LNGTH - 2.5 # FEET, FROM NOSE.
XCD = REF_LNGTH - (TAIL_TIP_CHRD / 2.0) # FEET, FROM NOSE.

# BODY.
WNG_AREA = 0.5 * WNG_HLF_SPN * (WNG_TIP_CHRD + WNG_ROOT_CHRD)
TAIL_AREA = 0.5 * TAIL_HLF_SPN * (TAIL_TIP_CHRD + TAIL_ROOT_CHRD)
REF_AREA = np.pi * (REF_DIAM ** 2) / 4
NOSE_AREA = NOSE_LNGTH * REF_DIAM
PLANFORM_AREA = (REF_LNGTH - NOSE_LNGTH) * REF_DIAM + \
	0.667 * NOSE_LNGTH * REF_DIAM
XCP_NOSE = 0.67 * NOSE_LNGTH
XCP_WNG = NOSE_LNGTH + BASE_OF_NOSE_TO_WNG + \
	0.7 * WNG_ROOT_CHRD - 0.2 * WNG_TIP_CHRD
AN = 0.67 * NOSE_LNGTH * REF_DIAM
AB = (REF_LNGTH - NOSE_LNGTH) * REF_DIAM
XCP_BODY = (0.67 * AN * NOSE_LNGTH + \
	AB * (NOSE_LNGTH + 0.5 * (REF_LNGTH - NOSE_LNGTH))) / (AN + AB)

# ATMOSPHERE.
ATM = ATM_IMPERIAL()
ATM.update(ALT, SPD)
RHO = ATM.rho
G = ATM.g
Q = ATM.q
A = ATM.a
MACH = ATM.mach
BETA = np.sqrt(MACH ** 2 - 1) # ND

# MASS AND MOTOR. (NO MOTOR FOR NOW.)
MASS = 22.95 # LBM
TMOI = (MASS * (3 * ((0.5 * REF_DIAM) ** 2) \
	+ REF_LNGTH ** 2)) / (12 * G) # LBF - FT - S^2

# DERIVATIVES.
SPECIFIC_FORCE = np.zeros(2)
ACC = np.zeros(2)
ADOT = 0.0
QDOT = 0.0
WDOT = 0.0

# STATE. 
POS = npa([0.0, ALT])
VEL = npa([SPD, 0.0])
THT = 0.0
RATE = 0.0
ALPHA = 0.0

# SIM CONTROL.
TOF = 0.0
DT = 0.01
MAXT = 3

# DATA
def populateState():
	STATE = {
		"TOF": TOF,
		"RNG": POS[0],
		"ALT": POS[1],
		"THETA": THT,
		"RATE": RATE,
		"ALPHA": ALPHA,
		"ALPHADOT": ADOT,
		"RATEDOT": QDOT,
		"WDOT": WDOT
	}
	return STATE

STATE = populateState()
LOGFILE = open("3DOFS/PY_3DOF_70MM_ROCKET_SIDE_SCROLLER/data/log.txt", "w")
lf.writeHeader(STATE, LOGFILE)
lf.writeData(STATE, LOGFILE)

LASTT = None
while TOF <= MAXT:

	# CN AND CM.
	CN = 2 * ALPHA + \
		(1.5 * PLANFORM_AREA * ALPHA * ALPHA) / REF_AREA + \
		(8 * WNG_AREA * ALPHA) / (BETA * REF_AREA) + \
		(8 * TAIL_AREA * (ALPHA + DEFL)) / (BETA * REF_AREA)
	CM = 2 * ALPHA * ((XCG - XCP_NOSE) / REF_DIAM) + \
		((1.5 * PLANFORM_AREA * ALPHA * ALPHA) / REF_AREA) * \
		((XCG - XCP_BODY) / REF_DIAM) + \
		((8 * WNG_AREA * ALPHA) / (BETA * REF_AREA)) * \
		((XCG - XCP_WNG) / REF_DIAM) + \
		((8 * TAIL_AREA * (ALPHA + DEFL)) / (BETA * REF_AREA)) * \
		((XCG - XCD) / REF_DIAM)

	# DERIVATIVES.
	ADOT = RATE - (SPECIFIC_FORCE[1] / SPD) # RADS PER S
	QDOT = (Q * REF_AREA * REF_DIAM * CM) / TMOI # RADS PER S^2
	WDOT = ((Q * REF_AREA * CN) * (G / MASS)) # FT PER S^2

	SPECIFIC_FORCE = npa([0.0, WDOT])
	LOCALG = npa([0.0, -1.0 * G])
	LOCAL_TO_BODY_TM = ct.BODY_TO_RANGE_AND_ALTITUDE(-1.0 * THT)
	BODYG = LOCAL_TO_BODY_TM @ LOCALG
	SPECIFIC_FORCE += BODYG
	ACC = (SPECIFIC_FORCE @ LOCAL_TO_BODY_TM)

	# STATE.
	TOF += DT
	POS += VEL * DT
	VEL += ACC * DT
	THT += RATE * DT
	RATE += QDOT * DT
	ALPHA += ADOT * DT

	# DATA.
	STATE = populateState()
	lf.writeData(STATE, LOGFILE)

	# REPORT.
	if round(TOF, 3).is_integer() and LASTT != TOF:
		print(f"{TOF:.0f} POS {POS}")
