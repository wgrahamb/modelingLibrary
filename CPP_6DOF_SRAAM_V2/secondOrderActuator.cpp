#include "secondOrderActuator.h"

secondOrderActuator::secondOrderActuator()
{
	t             = 0.0;
	deflLim       = 28.0;
	deflRateLimit = 225;
	wn            = 100;
	zeta          = 0.9;
	defl          = 0.0;
	deflDer       = 0.0;
	deflDot       = 0.0;
	deflDotDer    = 0.0;
}

double secondOrderActuator::update(double command, double dt)
{
	double temp;
	double deflDerNew;
	double edx;
	double deflDotDerNew;

	deflDerNew    = deflDot;
	temp          = signum(deflDerNew);
	if (fabs(deflDerNew) > deflRateLimit) deflDerNew = deflRateLimit * temp;
	defl          = trapezoidIntegrate(deflDerNew, deflDer, defl, dt);
	temp          = signum(defl);
	if (fabs(defl) > deflLim) defl = deflLim * temp;
	deflDer       = deflDerNew;
	edx           = command - defl;
	deflDotDerNew =  wn * wn * edx - 2 * zeta * wn * deflDer;
	deflDot       = trapezoidIntegrate(deflDotDerNew, deflDotDer, deflDot, dt);
	deflDotDer    = deflDotDerNew;
	t             += dt;

	return defl;
}