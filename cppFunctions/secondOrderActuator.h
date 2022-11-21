#pragma once
#include "util.h"

class secondOrderActuator
{

public:

	// Methods.
	secondOrderActuator();
	double update(double command, double dt);

private:

	// Time.
	double t; // total time in seconds

	// Class variables.
	double deflLim; // fin deflection limit in degrees
	double deflRateLimit; // fin rate limit in deg/s
	double wn; // natural frequency in deg/s
	double zeta; // damping

	// State.
	double defl; // deflection in degrees
	double deflDer; // derivative of deflection in deg/s
	double deflDot; // fin rate in deg/s
	double deflDotDer; // derivative of fin rate in deg/s^2

};

