// Standard.
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <math.h>
#include <vector>
#include <map>
#include <algorithm>
#include <memory>

// Namespace.
using namespace std;

// Components.
#include "secondOrderActuator.h"

#ifndef MISSILEMODEL_H
#define MISSILEMODEL_H

/* Missile constants. */
const double REFERENCE_AREA             = 0.01824; // missile reference area in m^2
const double REFERENCE_DIAMETER         = 0.1524; // missile reference length in meters
const double THRUST_EXIT_AREA           = 0.0125; // nozzle exit area in m^2
const double ROCKET_BURN_OUT_TIME       = 2.421; // motor shutoff in seconds
const double SEEKER_KF_G                = 10.0; // seeker Kalman filter gain in 1/s
const double SEEKER_KF_ZETA             = 0.9; // seeker Kalman filter damping
const double SEEKER_KF_WN               = 60.0; // seeker Kalman filter natural frequency in rads/s
const double PROPORTIONAL_GUIDANCE_GAIN = 2.75; // proportional navigation gain
const double MAXIMUM_ACCELERATION       = 450.0; // maximum acceleration allowed in m/s^2
const double ROLL_ANGLE_COMMAND         = 0.0; // commanded roll angle in radians or degrees
const double ALPHA_PRIME_MAX            = 40.0; // maximum angle of attack allowed in radians
const double SEA_LEVEL_PRESSURE         = 101325; // nominal pressure in pascals
const double LAUNCH_XCG_FROM_NOSE       = 1.5357; // center of mass at missile launch in meters
const double ROLL_ANGLE_GAIN            = 1.0; 
const double ROLL_PROP_GAIN             = 0.011;
const double ROLL_DER_GAIN              = 0.000034125;

/* This struct fully represents a missile. */
struct Missile
{

	/* Variables */

	// Target.
	double waypoint[3]; // guidance east, north, up target in meters (INIT)

	// Status.
	bool isBallistic    = false; // flag to indicate whether or not the missile is flying ballistically or guided
	bool isLaunched     = false; // flag to indicate whether or not the missile is free to launch
	double timeStep     = (1.0 / 600.0); // missile integration time step
	double halfTimeStep = timeStep * 0.5; // missile half integration time step for runge kutta methods

	// Body.
	double fluVel[3];    // missile velocity in the body frame in m/s (INIT)
	double spcfForce[3]; // missile acceleration in the body frame in m/s^2 (INIT)
	double spd;          // missile speed in m/s (INIT)
	double rate[3]      = {0.0, 0.0, 0.0}; // missile angular velocity in rads/s
	double rateDot[3]   = {0.0, 0.0, 0.0}; // missile angular acceleration in rads/s^2
	double tof          = 0.0; // missile time of flight in seconds
	double rng          = 0.0; // total missile range in meters
	double alphaRadians = 0.0; // vertical angle of attack in rads
	double betaRadians  = 0.0; // horizontal angle of attack in rads
	double alphaDegrees = 0.0; // vertical angle of attack in degrees
	double betaDegrees  = 0.0; // horizontal angle of attack in degrees

	// Local Frame.
	double enuPos[3];      // missile east, north, up position in meters (INIT)
	double enuVel[3];      // missile east, north, up velocity in m/s (INIT)
	double enuAcc[3];      // missile east, north, up acceleration in m/s^2 (INIT)
	double enuToFlu[3][3]; // missile east, north, up to forward, left, up transformation matrix (INIT)
	double enuAttitude[3]; // missile east, north, up attitude in radians (INIT)
	double enuAttitudeDot[3] = {0.0, 0.0, 0.0}; // missile east, north, up attitude rate in rads/s

	// Atmosphere.
	double grav       = 0.0; // gravity magnitude in m/s^2
	double fluGrav[3] = {0.0, 0.0, 0.0}; // gravity in the body frame in m/s^2
	double p          = 0.0; // air pressure in pascals
	double q          = 0.0; // dynamic pressure in pascals
	double mach       = 0.0; // mach speed

	// Seeker.
	double skrTht;            // seeker pitch angle relative to bore in radians (INIT)
	double skrPsi;            // seeker yaw angle relative to bore in radians (INIT)
	double skrEnuToFlu[3][3]; // seeker east, north, up to forward, left, up transformation matrix (INIT)
	double skrThtErr;         // seeker boresight vertical offset from target in radians (INIT)
	double skrPsiErr;         // seeker boresight horizontal offset from target in radians (INIT)
	double skrWlr;            // pointing yaw rate in rads/s (INIT)
	double skrWlq;            // Pointing pitch rate. Radians per second. (INIT)
	double skrWlrDot  = 0.0; // derivative of pointing yaw rate in rads/s^2
	double skrWlr1    = 0.0; // yaw sight line spin rate in rads/s
	double skrWlr1Dot = 0.0; // derivative of yaw sight line spin rate in rads/s^2
	double skrWlr2    = 0.0; // second state variable in yawing kalman filter in rads/s^2
	double skrWlr2Dot = 0.0; // derivative of second state variable in yawing kalman filter in rads/s^3
	double skrWlqDot  = 0.0; // derivative of pointing pitch rate in rads/s^2
	double skrWlq1    = 0.0; // pitch sight line spin rate in rads/s
	double skrWlq1Dot = 0.0; // derivative of pitch sight line spin rate in rads/s^2
	double skrWlq2    = 0.0; // second state variable in pitching kalman filter in rads/s^2
	double skrWlq2Dot = 0.0; // derivative of second state variable in pitching kalman filter in rads/s^3

	// Guidance.
	bool isHoming           = false; // flag to indicate whether or not the missile has entered the homing phase
	double timeToGo         = 0.0; // time to reach waypoint in seconds
	double mslToWaypoint[3] = {0.0, 0.0, 0.0}; // waypoint position relative to missile body in meters
	double normComm         = 0.0; // body normal guidance command in m/s^2
	double sideComm         = 0.0; // body lateral guidance command in m/s^2
	double commLimit        = MAXIMUM_ACCELERATION; // guidance command limit in m/s^2

	// Control
	double yawIntErr        = 0.0; // Something.
	double lastYawPropErr   = 0.0; // Radians per second.
	double yawPropErr       = 0.0; // Radians per second.
	double yawFinComm       = 0.0; // Radians.
	double pitchIntErr      = 0.0; // Something.
	double lastPitchPropErr = 0.0; // Radians per second.
	double pitchPropErr     = 0.0; // Radians per second.
	double pitchFinComm     = 0.0; // Radians.
	double lastRollPropErr  = 0.0; // Radians per second.
	double rollPropErr      = 0.0; // Radians per second.
	double rollFinComm      = 0.0; // Radians.

	// Actuators.
	shared_ptr<secondOrderActuator> actOne   =
		make_shared<secondOrderActuator>(); // pointer to actuator one
	shared_ptr<secondOrderActuator> actTwo   =
		make_shared<secondOrderActuator>(); // pointer to actuator two
	shared_ptr<secondOrderActuator> actThree =
		make_shared<secondOrderActuator>(); // pointer to actuator three
	shared_ptr<secondOrderActuator> actFour  =
		make_shared<secondOrderActuator>(); // pointer to actuator four
	double finOneDefl   = 0.0; // fin one deflection in radians
	double finTwoDefl   = 0.0; // fin two deflection in radians
	double finThreeDefl = 0.0; // fin three deflection in radians
	double finFourDefl  = 0.0; // fin four deflection in radians
	double pitchFinDefl = 0.0; // encoded pitching deflection in radians
	double yawFinDefl   = 0.0; // encoded yawing deflection in radians
	double rollFinDefl  = 0.0; // encoded rolling deflection in radians

	// Aerodynamic angles and conversions.
	double aoaRad           = 0.0; // total angle of attack in radians
	double aoaDeg           = 0.0; // total angle of attack in degrees
	double sinPhiPrime      = 0.0; // sin of phi prime
	double cosPhiPrime      = 0.0; // cos of phi prime
	double pitchFinDeflAero = 0.0; // pitching fin deflection in the aero ballistic frame in degrees
	double yawFinDeflAero   = 0.0; // yawing fin deflection in the aero ballistic frame in degrees
	double rollFinDeflDeg   = 0.0; // encoded rolling deflection in degrees
	double totFinDeflDeg    = 0.0; // total fin deflection in degrees
	double pitchRateAero    = 0.0; // pitching body rate in aero ballistic frame in deg/s
	double yawRateAero      = 0.0; // yawing body rate in aero ballistic frame in deg/s
	double rollRateDeg      = 0.0; // rolling rate in deg/s
	double sin4PhiPrime     = 0.0; // == sin(4 * phiPrime)
	double sqrtSin2PhiPrime = 0.0; // == sin(sqrt(2 * phiPrime))

	// Table look ups.
	map<string, int> tableNameIndexPairs; // map to track tables by name and their index in the vector below
	vector<vector<vector<double>>> tables; // stores the aerodynamic, propulsion, and mass property tables

	// Aerodynamics.
	double CA0    = 0.0; // axial force coefficient
	double CAA    = 0.0; // axial force derivative of alpha prime in 1/deg
	double CAD    = 0.0; // axial force derivative of control fin deflection in 1/deg^2
	double CA_OFF = 0.0; // power off correction term for axial force coefficient
	double CYP    = 0.0; // side force coefficient correction term for when phi is non zero
	double CYDR   = 0.0; // side force derivative of elevator in 1/deg
	double CN0    = 0.0; // normal force coefficient
	double CNP    = 0.0; // correction to normal force coefficient term for when phi is non zero
	double CNDQ   = 0.0; // normal force derivative of elevator in 1/deg
	double CLLAP  = 0.0; // roll moment derivative for aoa^2 for when phi is non zero in 1/deg^2
	double CLLP   = 0.0; // roll moment damping derivative in degrees
	double CLLDP  = 0.0; // roll moment derivative of aileron in 1/deg
	double CLM0   = 0.0; // pitching moment coefficient at launch center of gravity
	double CLMP   = 0.0; // correction to pitching moment coefficient for when phi is non zero
	double CLMQ   = 0.0; // pitching moment damping derivative in 1/deg
	double CLMDQ  = 0.0; // pitching moment derivative of elevator in 1/deg
	double CLNP   = 0.0; // yaw moment coefficient correction for when phi is non zero

	// Mass properties.
	double mass = 0.0; // missile mass in kilograms
	double tmoi = 0.0; // transverse moment of inertia in kg-m^2
	double amoi = 0.0; // axial moment of inertia in kg-m^2
	double xcg  = 0.0; // axial center of gravity in meters

	// Propulsion.
	double vacThrust = 0.0; // vacuum motor thrust in Newtons
	double thrust    = 0.0; // corrected motor thrust in Newtons

	// Aerodynamic coefficients.
	double CX = 0.0; // axial force coefficient
	double CY = 0.0; // lateral force coefficient
	double CZ = 0.0; // normal force coefficient
	double CL = 0.0; // rolling moment coefficient
	double CM = 0.0; // pitching moment coefficient
	double CN = 0.0; // yawing moment coefficient

	// Aerodynamic derivatives.
	double SM  = 0.0; // static margin

	// Performance and termination check.
	double missDistance = 0.0; // magnitude of the miss in meters
	string lethality;

	// Integration states.
	// P  = ENUPosition
	// V  = ENUVelocity
	// A  = ENUAcceleration
	// E  = ENUEulerAngles
	// ED = ENUEulerDot
	// W  = BodyRate
	// WD = BodyRateDot.
	int INTEGRATION_METHOD = 2;
	int INTEGRATION_PASS = 0;

	double P0[3] = {0.0, 0.0, 0.0};
	double V0[3] = {0.0, 0.0, 0.0};
	double E0[3] = {0.0, 0.0, 0.0};
	double W0[3] = {0.0, 0.0, 0.0};

	double P1[3] = {0.0, 0.0, 0.0};
	double V1[3] = {0.0, 0.0, 0.0};
	double A1[3] = {0.0, 0.0, 0.0};
	double E1[3] = {0.0, 0.0, 0.0};
	double ED1[3] = {0.0, 0.0, 0.0};
	double W1[3] = {0.0, 0.0, 0.0};
	double WD1[3] = {0.0, 0.0, 0.0};

	double P2[3] = {0.0, 0.0, 0.0};
	double V2[3] = {0.0, 0.0, 0.0};
	double A2[3] = {0.0, 0.0, 0.0};
	double E2[3] = {0.0, 0.0, 0.0};
	double ED2[3] = {0.0, 0.0, 0.0};
	double W2[3] = {0.0, 0.0, 0.0};
	double WD2[3] = {0.0, 0.0, 0.0};

	double P3[3] = {0.0, 0.0, 0.0};
	double V3[3] = {0.0, 0.0, 0.0};
	double A3[3] = {0.0, 0.0, 0.0};
	double E3[3] = {0.0, 0.0, 0.0};
	double ED3[3] = {0.0, 0.0, 0.0};
	double W3[3] = {0.0, 0.0, 0.0};
	double WD3[3] = {0.0, 0.0, 0.0};

	double P4[3] = {0.0, 0.0, 0.0};
	double V4[3] = {0.0, 0.0, 0.0};
	double A4[3] = {0.0, 0.0, 0.0};
	double E4[3] = {0.0, 0.0, 0.0};
	double ED4[3] = {0.0, 0.0, 0.0};
	double W4[3] = {0.0, 0.0, 0.0};
	double WD4[3] = {0.0, 0.0, 0.0};

};

// Public Functions.
void formatTables (Missile &missile, string dataFile);
Missile clone(const Missile &missile);
void emplace(Missile &missile, double phi, double theta, double psi, double ENUPosition[3]);
void seekerOn(Missile &missile);
void sixDofFly(Missile &missile, string flyOutID, bool writeData, bool consoleReport, double maxTime);
void threeDofFly(Missile &missile, string flyOutID, bool writeData, bool consoleReport, double maxTime);

#endif