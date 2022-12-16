import numpy as np
from numpy import linalg as la
from numpy import array as npa
np.set_printoptions(suppress=True, precision=4)
import pymap3d

# CONSTANTS.
WEII3       = 7.292115e-5 # Rotation speed of earth. Radians per second.
REARTH      = 6370987.308 # Meters.
SMALL       = 9.999999999999999547e-08
DEG_TO_RAD  = 0.01745329251994319833
RAD_TO_DEG  = 1.0 / DEG_TO_RAD
SMAJOR_AXIS = 6378137.0 # Meters.
FLATTENING  = 1.0 / 298.257223563
GM          = 398600440000000.0
C20         = -1.0 * 0.0004841668499999999772

# //////////////////////////////////////////////////////////////////////////////
# - Returns the T.M. of earth wrt inertial coordinates 
# - Return output
#     TEI = T.M. of Earthy wrt inertial coordinates 
# - Argument input
#     time = time since start of simulation - s
# - 10628 Created by Peter H Zipfel
# - Ported to python by Graham Beech.
# - Should not be used unless integrating state in an inertial earth frame.
# //////////////////////////////////////////////////////////////////////////////
def ECI_TO_ECEF_TM(TIME): # Seconds from launch.
	GW_CLONG = 0.0 # Greenwich celestial longitude at start of flight. Radians.
	TEI = np.eye(3) # TM
	XI = WEII3 * TIME + GW_CLONG
	SXI = np.sin(XI)
	CXI = np.cos(XI)
	TEI[0, 0] = CXI
	TEI[0, 1] = SXI
	TEI[1, 0] = -1.0 * SXI
	TEI[1, 1] = CXI
	return TEI

# ////////////////////////////////////////////////////////////////////////////
# - Calculates geodetic longitude, latitude, and altitude from inertial
# displacement vector
# - using the WGS 84 reference ellipsoid
# - Reference: Britting,K.R."Inertial Navigation Systems Analysis", Wiley. 1971
# - Parameter output
#        lon = geodetic longitude - rad
#        lat = geodetic latitude - rad
#        alt = altitude above ellipsoid - m
# - Parameter input
#        SBII(3x1) = Inertial position - m
# - 030414 Created from FORTRAN by Peter H Zipfel
# - Ported to python by Graham Beech.
# - If no time is input, then ECI can be treated as ECEF.
# /////////////////////////////////////////////////////////////////////////////
def ECI_TO_LLA(
	ECIPOS, 
	TIME: float = None
):

	if TIME == None:
		TIME = 0.0

	LLAREF = np.zeros(3)
	GW_CLONG = 0.0 # Greenwich celestial longitude at start of flight. Radians.
	COUNT = 0
	LAT0 = 0.0
	ALAMDA = 0.0
	DBI = la.norm(ECIPOS)
	LATG = np.arcsin(ECIPOS[2] / DBI)
	LLAREF[0] = LATG
	GO = True
	while GO:
		LAT0 = LLAREF[0]
		R0 = SMAJOR_AXIS * \
			(
			1.0 - \
			(FLATTENING * (1.0 - np.cos(2.0 * LAT0)) / 2.0) + \
			(5.0 * (FLATTENING ** 2) * (1.0 - np.cos(4 * LAT0)) / 16.0)
			)
		LLAREF[2] = DBI - R0
		DD = FLATTENING * np.sin(2 * LAT0) * \
			(1.0 - FLATTENING / 2.0 - LLAREF[2] / R0)
		LLAREF[0] = LATG + DD
		COUNT += 1
		if COUNT > 100:
			print("GEODETIC LATITUDE DOES NOT CONVERGE")
			return
		if np.abs(LLAREF[0] - LAT0) < SMALL:
			break
	SBII1 = ECIPOS[0]
	SBII2 = ECIPOS[1]
	DUM4 = np.arcsin(SBII2 / np.sqrt(SBII1 ** 2 + SBII2 ** 2))
	if SBII1 >= 0.0 and SBII2 >= 0.0:
		ALAMDA = DUM4
	if SBII1 < 0.0 and SBII2 >= 0.0:
		ALAMDA = (180.0 * DEG_TO_RAD) - DUM4
	if SBII1 < 0.0 and SBII2 < 0.0:
		ALAMDA = (180.0 * DEG_TO_RAD) - DUM4
	if SBII1 > 0.0 and SBII2 < 0.0:
		ALAMDA = (360.0 * DEG_TO_RAD) + DUM4
	LLAREF[1] = ALAMDA - WEII3 * TIME - GW_CLONG
	if LLAREF[1] > (180.0 * DEG_TO_RAD):
		TEMP = -1.0 * ((360.0 * DEG_TO_RAD) - LLAREF[1])
		LLAREF[1] = TEMP
	return LLAREF

# /////////////////////////////////////////////////////////////////////////////
# - Returns the inertial displacement vector from longitude, latitude and altitude
# - using the WGS 84 reference ellipsoid
# - Reference: Britting,K.R."Inertial Navigation Systems Analysis"
# pp.45-49, Wiley, 1971
# - Return output
#        SBII(3x1) = Inertial vehicle position - m
# - Parameter input
#        lon = geodetic longitude - rad
#        lat = geodetic latitude - rad
#        alt = altitude above ellipsoid - m
#        time = simulation time - sec 
# - 030411 Created from FORTRAN by Peter H Zipfel
# - Ported to python by Graham Beech.
# - If no time is input, then ECI can be treated as ECEF.
# /////////////////////////////////////////////////////////////////////////////
def LLA_TO_ECI(
	LLA,
	TIME: float = None
):

	if TIME == None:
		TIME = 0.0

	GW_CLONG = 0.0 # Greenwich celestial longitude at start of flight. Radians.
	SBII = np.zeros(3)
	SBID = np.zeros(3)
	R0 = SMAJOR_AXIS * \
		(
			1.0 - \
			(FLATTENING * (1.0 - np.cos(2.0 * LLA[0])) / 2.0) + \
			(5.0 * (FLATTENING ** 2) * (1.0 - np.cos(4 * LLA[0])) / 16.0)
		)
	DD = FLATTENING * np.sin(2 * LLA[0]) * (1.0 - FLATTENING / 2.0 - LLA[2] / R0)
	DBI = R0 + LLA[2]
	SBID[0] = -1.0 * DBI * np.sin(DD)
	SBID[1] = 0.0
	SBID[2] = -1.0 * DBI * np.cos(DD)
	LON_CEL = GW_CLONG + WEII3 * TIME + LLA[1]
	SLAT = np.sin(LLA[0])
	CLAT = np.cos(LLA[0])
	SLON = np.sin(LON_CEL)
	CLON = np.cos(LON_CEL)
	SBII[0] = -1.0 * SLAT * CLON * SBID[0] - CLAT * CLON * SBID[2]
	SBII[1] = -1.0 * SLAT * SLON * SBID[0] - CLAT * SLON * SBID[2]
	SBII[2] = CLAT * SBID[0] - SLAT * SBID[2]
	return SBII

# //////////////////////////////////////////////////////////////////////////////
# - Returns the T.M. of geographic (geocentric) wrt inertial
# - using the WGS 84 reference ellipsoid
# - Reference: Britting,K.R."Inertial Navigation Systems Analysis",
# pp.45-49, Wiley, 1971
# - Return output
#        TGI(3x3) = T.M.of geographic wrt inertial coord - ND
# - Parameter input
#        lon = geodetic longitude - rad
#        lat = geodetic latitude - rad
#        alt = altitude above ellipsoid - m
# - 030414 Created from FORTRAN by Peter H Zipfel
# - Ported to python by Graham Beech.
# - If no time is input, then ECI can be treated as ECEF.
# /////////////////////////////////////////////////////////////////////////////
def GEOC_LLA_TO_ECI_TM(
	GEOD_LLA,
	TIME: float = None
):

	if TIME == None:
		TIME = 0.0

	GW_CLONG = 0.0 # Greenwich celestial longitude at start of flight. Radians.

	TDI = np.zeros((3, 3))
	TGD = np.zeros((3, 3))
	TGI = np.zeros((3, 3))

	LON_CEL = GW_CLONG + WEII3 * TIME + GEOD_LLA[1]

	TDI13 = np.cos(GEOD_LLA[0])
	TDI33 = -1.0 * np.sin(GEOD_LLA[0])
	TDI22 = np.cos(LON_CEL)
	TDI21 = -1.0 * np.sin(LON_CEL)

	TDI[0, 2] = TDI13
	TDI[2, 2] = TDI33
	TDI[1, 1] = TDI22
	TDI[1, 0] = TDI21
	TDI[0, 0] = TDI33 * TDI22
	TDI[0, 1] = -1.0 * TDI33 * TDI21
	TDI[2, 0] = -1.0 * TDI13 * TDI22
	TDI[2, 1] = TDI13 * TDI21

	R0 = SMAJOR_AXIS * \
		(
			1.0 - \
			(FLATTENING * (1.0 - np.cos(2.0 * GEOD_LLA[0])) / 2.0) + \
			(5.0 * (FLATTENING ** 2) * (1.0 - np.cos(4 * GEOD_LLA[0])) / 16.0)
		)
	DD = FLATTENING * np.sin(2 * GEOD_LLA[0]) * \
		(1.0 - FLATTENING / 2.0 - GEOD_LLA[2] / R0)

	COSDD = np.cos(DD)
	SINDD = np.sin(DD)

	TGD[0, 0] = COSDD
	TGD[2, 2] = COSDD
	TGD[1, 1] = 1.0
	TGD[2, 0] = SINDD
	TGD[0, 2] = -1.0 * SINDD

	TGI = TGD @ TDI

	return TGI

# /////////////////////////////////////////////////////////////////////////////
# - Earth gravitational acceleration, using the WGS 84 ellipsoid
# - Ref: Chatfield, A.B.,"Fundamentals of High Accuracy Inertial
# - Navigation",p.10, Prog.Astro and Aeronautics, Vol 174, AIAA, 1997.
# - Return output
#        GRAVG(3x1) = gravitational acceleration in geocentric coord - m/s^2
# - Parameter input
#        SBII = inertial displacement vector - m
#        time = simulation time - sec 
# - 030417 Created from FORTRAN by Peter H Zipfel
# - Ported to python by Graham Beech.
# - If no time is input, then ECI can be treated as ECEF.
# /////////////////////////////////////////////////////////////////////////////
def GEOCENTRIC_GRAV(
	ECIPOS,
	TIME: float = None
):

	if TIME == None:
		TIME = 0.0

	ECIGRAV = np.zeros(3)
	LLAREF = ECI_TO_LLA(ECIPOS, TIME)
	DBI = la.norm(ECIPOS)
	DUM1 = GM / (DBI ** 2)
	DUM2 = 3.0 * np.sqrt(5.0)
	DUM3 = (SMAJOR_AXIS / DBI) ** 2
	ECIGRAV[0] = -1.0 * DUM1 * DUM2 * C20 * \
		DUM3 * np.sin(LLAREF[0]) * np.cos(LLAREF[0])
	ECIGRAV[1] = 0.0
	ECIGRAV[2] = DUM1 * (1.0 + (DUM2 / 2.0) * C20 * \
		DUM3 * (3 * (np.sin(LLAREF[0] ** 2)) - 1.0))
	return ECIGRAV

# GB
def EULER_FROM_DCM(TM):
	EULER = np.zeros(3)
	R31 = TM[0, 2]
	THETA = np.arcsin(R31)
	R32 = TM[1, 2]
	R33 = TM[2, 2]
	PHI = np.arctan2((R32 / np.cos(THETA)), (R33 / np.cos(THETA)))
	R21 = TM[0, 1]
	R11 = TM[0, 0]
	PSI = np.arctan2((R21 / np.cos(THETA)), (R11 / np.cos(THETA)))
	EULER[0] = PHI
	EULER[1] = THETA
	EULER[2] = PSI
	return EULER

# GB
def ECEF_DISPLACEMENT_TO_ENU(RELPOS, LAT0, LON0):
	TEMP = np.cos(LON0) * RELPOS[0] + np.sin(LON0) * RELPOS[1]
	E = -1.0 * np.sin(LON0) * RELPOS[0] + np.cos(LON0) * RELPOS[1]
	N = np.cos(LAT0) * TEMP + np.sin(LAT0) * RELPOS[2]
	U = -1.0 * np.sin(LAT0) * TEMP + np.cos(LAT0) * RELPOS[2]
	ENU = npa([E, N, U])
	return ENU

# Returns earth centered earth fixed coordinates given geocentric coordinates. GB.
def GEOC_LLA_TO_ECEF(GEOC_LLA):

	ECEF      = np.zeros(3)
	RADIUS    = -1.0 * (GEOC_LLA[2] + REARTH)

	TGE       = np.zeros((3, 3))
	CLON      = np.cos(GEOC_LLA[1])
	SLON      = np.sin(GEOC_LLA[1])
	CLAT      = np.cos(GEOC_LLA[0])
	SLAT      = np.sin(GEOC_LLA[0])
	TGE[0, 0] = -1.0 * SLAT * CLON
	TGE[0, 1] = -1.0 * SLAT * SLON
	TGE[0, 2] = CLAT
	TGE[1, 0] = -1.0 * SLON
	TGE[1, 1] = CLON
	TGE[1, 2] = 0.0
	TGE[2, 0] = -1.0 * CLAT * CLON
	TGE[2, 1] = -1.0 * CLAT * SLON
	TGE[2, 2] = -1.0 * SLAT

	ECEF      = TGE.transpose() @ npa([0.0, 0.0, RADIUS])

	return ECEF

# //////////////////////////////////////////////////////////////////////////////
# - Returns lon, lat, alt from inertial displacement vector
# - 000620 Created by Vy Nguyen
# - Ported to python by Graham Beech.
# - Returns geocentric coordinates given earth centered earth fixed coordinates.
# //////////////////////////////////////////////////////////////////////////////
def ECEF_TO_GEOC_LLA(ECEF):

	DBI = la.norm(ECEF)
	LAT = np.arcsin(ECEF[2]/DBI)
	ALT = DBI-REARTH
	DUM4 = np.arcsin(ECEF[1] / np.sqrt(ECEF[0] ** 2 + ECEF[1] ** 2))

	ALAM = None
	if ECEF[0] >= 0 and ECEF[1] >= 0:
		ALAM = DUM4
	if ECEF[0] < 0 and ECEF[1] >= 0:
		ALAM = np.radians(180.0)-DUM4
	if ECEF[0] < 0 and ECEF[1] < 0:
		ALAM = np.radians(180.0)-DUM4
	if ECEF[0] >= 0 and ECEF[1] < 0:
		ALAM = np.radians(360.0)+DUM4

	LON = ALAM
	if LON > np.radians(180.0):
		TEMP = -1.0 * (np.radians(360.0)-LON)
		LON = TEMP

	RET = npa([LAT, LON, ALT])
	
	return RET

if __name__ == "__main__":

	# LLA example.
	lla        = npa([np.radians(38.8719), np.radians(77.0563), 0.0])
	pyMapEcef  = pymap3d.geodetic2ecef(np.degrees(lla[0]),
		np.degrees(lla[1]), lla[2])
	zipfelEcef = LLA_TO_ECI(lla)
	print(f"PYMAP3D X: {pyMapEcef[0]:.4f}; ZIPFEL X: {zipfelEcef[0]:.4f}")
	print(f"    DIFF: {(np.abs(pyMapEcef[0]-zipfelEcef[0])):.4f}")
	print(f"PYMAP3D Y: {pyMapEcef[1]:.4f}; ZIPFEL Y: {zipfelEcef[1]:.4f}")
	print(f"    DIFF: {(np.abs(pyMapEcef[1]-zipfelEcef[1])):.4f}")
	print(f"PYMAP3D Z: {pyMapEcef[2]:.4f}; ZIPFEL Z: {zipfelEcef[2]:.4f}")
	print(f"    DIFF: {(np.abs(pyMapEcef[2]-zipfelEcef[2])):.4f}")

	# ECEF example. Differences should be zero.
	pyMapLla = pymap3d.ecef2geodetic(pyMapEcef[0], pyMapEcef[1], pyMapEcef[2])
	zipfelLla = ECI_TO_LLA(zipfelEcef)
	zipfelLla[1] *= RAD_TO_DEG
	zipfelLla[0] *= RAD_TO_DEG
	print()
	print(f"PYMAP3D LAT: {pyMapLla[0]:.4f}; ZIPFEL LAT: {zipfelLla[0]:.4f}")
	print(f"    DIFF: {(np.abs(pyMapLla[0]-zipfelLla[0])):.4f}")
	print(f"PYMAP3D LON: {pyMapLla[1]:.4f}; ZIPFEL LON: {zipfelLla[1]:.4f}")
	print(f"    DIFF: {(np.abs(pyMapLla[1]-zipfelLla[1])):.4f}")
	print(f"PYMAP3D ALT: {pyMapLla[2]:.4f}; ZIPFEL ALT: {zipfelLla[2]:.4f}")
	print(f"    DIFF: {(np.abs(pyMapLla[2]-zipfelLla[2])):.4f}")































