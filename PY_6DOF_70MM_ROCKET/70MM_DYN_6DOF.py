import numpy as np
from numpy import array as npa
from numpy import linalg as la
import matplotlib
import matplotlib.pyplot as plt
import pandas as pd
import copy

import utility.loggingFxns                  as lf
import utility.angles                       as ang
import utility.coordinateTransformations    as ct
from   utility.ATM_IMPERIAL                 import ATM_IMPERIAL
from   utility.interpolationGivenTwoVectors import linearInterpolation
import utility.matPlotLibColors             as mc

from TABLES import *

matplotlib.use('WebAgg')
np.set_printoptions(precision=2, suppress=True)

# INPUTS.
ALT  = 10000 # FEET
SPD  = 300 # FEET PER SEC
EL   = 1.0 # DEG
AZ   = 0.0 # DEG
MAXT = 15 # SECONDS
DT   = 1.0 / 600.0 # SECONDS

# MISSILE CONSTANTS.
REF_DIAM  = 0.23 # FEET
REF_LNGTH = 4.59 # FEET
REF_AREA  = np.pi * (REF_DIAM ** 2) / 4 # FT^2
BURNOUT   = 1.112 # SECONDS

# AERODYNAMICS.
CMQ = linearInterpolation(0.0, MACHS_1, CMQS) # PER RAD
CNA = linearInterpolation(0.0, MACHS_1, CNAS) # PER RAD
XCP = linearInterpolation(0.0, MACHS_1, XCPS) / 12.0 # FEET FROM NOSE
CD  = linearInterpolation(0.0, MACHS_2, CD_ON) # ND
CL  = linearInterpolation(0.0, MACHS_3, CLS) # ND
CLP = linearInterpolation(0.0, MACHS_3, CLPS) # PER RAD

# ATMOSPHERE.
ATM  = ATM_IMPERIAL()
ATM.update(ALT, SPD)
RHO  = ATM.rho # SL PER FT^3
G    = ATM.g # FT PER S^2
Q    = ATM.q # PSF
A    = ATM.a # FT PER S
MACH = ATM.mach # ND

# MASS AND MOTOR.
XCG    = REF_LNGTH - (linearInterpolation(0.0, T1S, XCGS) / 12.0) # FT
TMOI   = (linearInterpolation(0.0, T1S, TMOIS)) / (144.0 * 32.2) # LBF - FT - S^2
AMOI   = (linearInterpolation(0.0, T1S, AMOIS)) / (144.0 * 32.2) # LBF - FT - S^2
MASS   = linearInterpolation(0.0, T1S, WEIGHTS) # LBM
THRUST = linearInterpolation(0.0, T2S, THRUSTS) # LBF
TORQUE = linearInterpolation(0.0, T2S, TORQUES) / 12.0 # LBF - FT

# ATTITUDE.
ENU2FLU = ct.ORIENTATION_TO_LOCAL_TM(0.0, \
	-1.0 * np.radians(EL), np.radians(AZ)) # ND

# DERIVATIVES.
SPECIFIC_FORCE = np.zeros(3) # FT PER S^2
ACC            = np.zeros(3) # FT PER S^2
PHIDOT         = 0.0 # RADS PER SEC
THTDOT         = 0.0 # RADS PER SEC
PSIDOT         = 0.0 # RADS PER SEC
PDOT           = 0.0 # RADS PER S^2
QDOT           = 0.0 # RADS PER S^2
RDOT           = 0.0 # RADS PER S^2
ADOT           = 0.0 # RADS PER S
BDOT           = 0.0 # RADS PER S

# STATE.
TOF       = 0.0 # SECONDS
POS       = npa([0.0, 0.0, ALT]) # FT
VEL       = npa([SPD, 0.0, 0.0]) @ ENU2FLU # FT PER S
PHI       = 0.0 # RADS
THT       = np.radians(EL) # RADS
PSI       = np.radians(AZ) # RADS
ROLLRATE  = 0.0 # RADS PER S
PITCHRATE = 0.0 # RADS PER S
YAWRATE   = 0.0 # RADS PER S
ALPHA     = 0.0 # RADS
BETA      = 0.0 # RADS

INT_PASS   = 0 # RK2 INTEGRATION
POS0       = None # FT
VEL0       = None # FT PER S
PHI0       = None # RADS
THT0       = None # RADS
PSI0       = None # RADS
ROLLRATE0  = None # RADS PER S
PITCHRATE0 = None # RADS PER S
YAWRATE0   = None # RADS PER S
ALPHA0     = None # RADS
BETA0      = None # RADS

# DATA
def populateState():
	STATE = {
		"TOF": TOF,
		"RNG": POS[0],
		"CROSSRNG": POS[1],
		"ALT": POS[2],
		"PHI": PHI,
		"THETA": THT,
		"PSI": PSI,
		"ROLLRATE": ROLLRATE,
		"PITCHRATE": PITCHRATE,
		"YAWRATE": YAWRATE,
		"ALPHA": ALPHA,
		"BETA": BETA,
		"ALPHADOT": ADOT,
		"BETADOT": BDOT,
		"PDOT": PDOT,
		"QDOT": QDOT,
		"RDOT": RDOT,
		"UDOT": SPECIFIC_FORCE[0],
		"VDOT": SPECIFIC_FORCE[1],
		"WDOT": SPECIFIC_FORCE[2]
	}
	return STATE

# LOG
STATE   = populateState()
LOGFILE = open("PY_6DOF_70MM_ROCKET/data/log.txt", "w")
lf.writeHeader(STATE, LOGFILE)
lf.writeData(STATE, LOGFILE)

LASTT = None # SECONDS
while True:

	# ATTITUDE.
	ENU2FLU  = ct.ORIENTATION_TO_LOCAL_TM(PHI, -1.0 * THT, PSI) # ND
	VELB     = ENU2FLU @ VEL
	AERO_ANG = ang.returnAeroAngles(VELB)

	# MASS AND MOTOR.
	XCG    = REF_LNGTH - (linearInterpolation(TOF, T1S, XCGS) / 12.0) # FT FROM NOSE
	TMOI   = (linearInterpolation(TOF, T1S, TMOIS)) / (144.0 * 32.2) # LBF-FT-S^2
	AMOI   = (linearInterpolation(TOF, T1S, AMOIS)) / (144.0 * 32.2) # LBF-FT-S^2
	MASS   = linearInterpolation(TOF, T1S, WEIGHTS) # LBM
	THRUST = linearInterpolation(TOF, T2S, THRUSTS) # LBF
	TORQUE = linearInterpolation(TOF, T2S, TORQUES) / 12.0 # LBF - FT

	ACC_THRUST    = (THRUST / MASS) * 32.2 # FT PER S^2
	ANGACC_TORQUE = (2 * TORQUE / (MASS * (REF_DIAM / 2) * \
		(REF_DIAM / 2))) * 32.2 # RADS PER S^2

	# ATMOSPHERE.
	SPD       = la.norm(VEL)
	ATM.update(POS[2], SPD)
	RHO       = ATM.rho # SL PER FT^3
	G         = ATM.g # FT PER S^2
	Q         = ATM.q # PSF
	A         = ATM.a # FT PER S
	MACH      = ATM.mach # ND
	LOCALG    = npa([0.0, 0.0, -1.0 * G]) # FT PER S^2
	BODY_GRAV = ENU2FLU @ LOCALG # FT PER S^2

	# AERODYNAMICS. #
	# PITCH AND YAW.
	# DIVIDED CMQ BY 2.0, ALLOWS FOR BALLISTIC FLIGHT.
	CMQ = linearInterpolation(MACH, MACHS_1, CMQS) # PER RAD
	CNA = linearInterpolation(MACH, MACHS_1, CNAS) # PER RAD
	XCP = linearInterpolation(MACH, MACHS_1, XCPS) / 12.0 # FT FROM NOSE
	CZ  = CNA * ALPHA # ND
	CM  = CZ * (XCG - XCP) / REF_DIAM + \
		(REF_DIAM / (2 * SPD)) * (CMQ / 2) * PITCHRATE # ND
	CY  = CNA * BETA # ND
	CN  = CY * (XCG - XCP) / REF_DIAM + \
		(REF_DIAM / (2 * SPD)) * (CMQ / 2) * YAWRATE # ND

	# DRAG.
	CD = None # ND
	if TOF < BURNOUT:
		CD = linearInterpolation(MACH, MACHS_2, CD_ON) # ND
	else:
		CD = linearInterpolation(MACH, MACHS_2, CD_OFF) # ND
	DRAG_FORCE      = CD * REF_AREA * Q # LBF
	WIND_TO_BODY    = ct.FLIGHTPATH_TO_LOCAL_TM(BETA, ALPHA) # ND
	WIND_DRAG_FORCE = npa([-DRAG_FORCE, 0.0, 0.0]) # LBF
	BODY_DRAG       = ((WIND_TO_BODY @ WIND_DRAG_FORCE) / MASS) * 32.2 # FT PER S^2

	# ROLL.
	CL  = linearInterpolation(MACH, MACHS_3, CLS) # ND
	CLP = linearInterpolation(MACH, MACHS_3, CLPS) # PER RAD
	CLL = CL + CLP * ROLLRATE * REF_DIAM / (2 * SPD) # ND

	# DERIVATIVES.
	WDOT           = ((Q * REF_AREA * CZ) * (32.2 / MASS)) # FT PER S^2
	VDOT           = ((Q * REF_AREA * CY) * (32.2 / MASS)) # FT PER S^2
	SPECIFIC_FORCE = npa([ACC_THRUST, VDOT, WDOT]) # FT PER S^2
	SPECIFIC_FORCE += BODY_GRAV # ADD GRAVITY
	SPECIFIC_FORCE += BODY_DRAG # ADD DRAG
	ACC            = (SPECIFIC_FORCE @ ENU2FLU) # FT PER S^2
	PHIDOT         = ROLLRATE + (PITCHRATE * np.sin(PHI) + YAWRATE * np.cos(PHI)) \
		* np.tan(THT) # RADS PER SEC
	THTDOT         = PITCHRATE * np.cos(PHI) - YAWRATE * np.sin(PHI) # RADS PER SEC
	PSIDOT         = (PITCHRATE * np.sin(PHI) + YAWRATE * np.cos(PHI)) \
		/ np.cos(THT) # RADS PER SEC
	PDOT           = ((Q * REF_AREA * REF_DIAM * CLL) / AMOI) \
		+ ANGACC_TORQUE # RADS PER S^2
	QDOT           = (Q * REF_AREA * REF_DIAM * CM) / TMOI # RADS PER S^2
	RDOT           = (Q * REF_AREA * REF_DIAM * CN) / TMOI # RADS PER S^2
	ADOT           = PITCHRATE - (SPECIFIC_FORCE[2] / SPD) # RADS PER S
	BDOT           = YAWRATE - (SPECIFIC_FORCE[1] / SPD) # RADS PER S

	# STATE.
	if INT_PASS == 0:

		# DATA.
		STATE = populateState()
		lf.writeData(STATE, LOGFILE)

		# REPORT.
		if round(TOF, 3).is_integer() and LASTT != TOF:
			print(f"{TOF:.0f} POS {POS} MACH {MACH:.2f}")
			LASTT = TOF

		# END CHECK.
		if TOF > MAXT:
			print(f"{TOF:.3f} POS {POS} MACH {MACH:.2f}")
			print("TIME")
			break
		if POS[2] < 0.0:
			print(f"{TOF:.3f} POS {POS} MACH {MACH:.2f}")
			print("GROUND")
			break
		if np.isnan(POS[1]):
			print(f"{TOF:.3f} POS {POS} MACH {MACH:.2f}")
			print("NAN")
			break

		# BEGIN INTEGRATION PASS.
		INT_PASS += 1
		TOF      += (DT / 2.0)

		POS0       = copy.deepcopy(POS)
		VEL0       = copy.deepcopy(VEL)
		PHI0       = copy.deepcopy(PHI)
		THT0       = copy.deepcopy(THT)
		PSI0       = copy.deepcopy(PSI)
		ROLLRATE0  = copy.deepcopy(ROLLRATE)
		PITCHRATE0 = copy.deepcopy(PITCHRATE)
		YAWRATE0   = copy.deepcopy(YAWRATE)
		ALPHA0     = copy.deepcopy(ALPHA)
		BETA0      = copy.deepcopy(BETA)

		POS       += VEL * (DT / 2.0)
		VEL       += ACC * (DT / 2.0)
		PHI       += PHIDOT * (DT / 2.0)
		THT       += THTDOT * (DT / 2.0)
		PSI       += PSIDOT * (DT / 2.0)
		ROLLRATE  += PDOT * (DT / 2.0)
		PITCHRATE += QDOT * (DT / 2.0)
		YAWRATE   += RDOT * (DT / 2.0)
		ALPHA     += ADOT * (DT / 2.0)
		BETA      += BDOT * (DT / 2.0)

		if PHI > (2 * np.pi):
			TEMP = PHI - (2 * np.pi)
			PHI  = 0.0 + TEMP
		elif PHI < 0.0:
			TEMP = 360.0 - PHI
			PHI  = TEMP

	else:

		INT_PASS = 0
		TOF      += (DT / 2.0)

		POS       = POS0 + VEL * DT
		VEL       = VEL0 + ACC * DT
		PHI       = PHI0 + PHIDOT * DT
		THT       = THT0 + THTDOT * DT
		PSI       = PSI0 + PSIDOT * DT
		ROLLRATE  = ROLLRATE0 + PDOT * DT
		PITCHRATE = PITCHRATE0 + QDOT * DT
		YAWRATE   = YAWRATE0 + RDOT * DT
		ALPHA     = ALPHA0 + ADOT * DT
		BETA      = BETA0 + BDOT * DT

		POS0       = None
		VEL0       = None
		PHI0       = None
		THT0       = None
		PSI0       = None
		ROLLRATE0  = None
		PITCHRATE0 = None
		YAWRATE0   = None
		ALPHA0     = None
		BETA0      = None

		if PHI > (2 * np.pi):
			TEMP = PHI - (2 * np.pi)
			PHI  = 0.0 + TEMP
		elif PHI < 0.0:
			TEMP = (2 * np.pi) - PHI
			PHI  = TEMP

# SET UP MATPLOTLIB FIG.
DATFILE = open("PY_6DOF_70MM_ROCKET/data/log.txt", "r")
DF      = pd.read_csv(DATFILE, delim_whitespace=True)
FIG     = plt.figure()
START   = 0
STOP    = -1
COLORS  = mc.matPlotLibColors()

# TRAJECTORY.
TRAJ = FIG.add_subplot(221, projection="3d")
TRAJ.set_title("TRAJECTORY")
TRAJ.set_xlabel("EAST.")
TRAJ.set_ylabel("NORTH.")
TRAJ.set_zlabel("UP.")
XMIN = min(list(DF.iloc[START:STOP]["RNG"]))
XMAX = max(list(DF.iloc[START:STOP]["RNG"]))
YMIN = min(list(DF.iloc[START:STOP]["CROSSRNG"]))
YMAX = max(list(DF.iloc[START:STOP]["CROSSRNG"]))
ZMIN = min(list(DF.iloc[START:STOP]["ALT"]))
ZMAX = max(list(DF.iloc[START:STOP]["ALT"]))
TRAJ.set_box_aspect(
	(
		np.ptp([XMIN - 1000, XMAX + 1000]), 
		np.ptp([YMIN - 5000, YMAX + 5000]), 
		np.ptp([ZMIN, ZMAX + 1000]),
	)
)
TRAJ.set_xlim([XMIN - 1000, XMAX + 1000])
TRAJ.set_ylim([YMIN - 5000, YMAX + 5000])
TRAJ.set_zlim([ZMIN, ZMAX + 1000])
TRAJ.plot(
	DF.iloc[START:STOP]["RNG"],
	DF.iloc[START:STOP]["CROSSRNG"],
	DF.iloc[START:STOP]["ALT"],
	color="b"
)

# PITCH RATE.
PITCHRATE = FIG.add_subplot(222)
PITCHRATE.set_xlabel("TOF")
PITCHRATE.set_ylabel("Q, RADS PER SEC.")
PITCHRATE.plot(
	DF.iloc[START:STOP]["TOF"],
	DF.iloc[START:STOP]["PITCHRATE"],
	color=COLORS.pop(0)
)

# ROLL RATE.
ROLLRATE = FIG.add_subplot(223)
ROLLRATE.set_xlabel("TOF")
ROLLRATE.set_ylabel("P, HERTZ.")
ROLLRATE.plot(
	DF.iloc[START:STOP]["TOF"],
	DF.iloc[START:STOP]["ROLLRATE"] / 6.28,
	color=COLORS.pop(0)
)

# YAW RATE.
YAWRATE = FIG.add_subplot(224)
YAWRATE.set_xlabel("TOF")
YAWRATE.set_ylabel("R, RADS PER SEC.")
YAWRATE.plot(
	DF.iloc[START:STOP]["TOF"],
	DF.iloc[START:STOP]["YAWRATE"],
	color=COLORS.pop(0)
)

# SHOW.
plt.show()














