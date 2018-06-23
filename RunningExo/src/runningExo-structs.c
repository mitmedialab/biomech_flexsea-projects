/*
 * runningExo-parameters.c
 *
 *  Created on: Jun 19, 2018
 *      Author: Albert Wu
 */
#include "runningExo-parameters.h"
#include "runningExo-structs.h"

float updateDerivative(derivativeAverager *der,float newValue)
{
	der->lastIndex++;
	der->lastIndex = (der->lastIndex<AVERAGE_FILTER_SAMPLES ? der->lastIndex : 0);
	der->sum-=der->prevDerivatives[der->lastIndex];
	der->sum+=newValue;
	der->prevDerivatives[der->lastIndex]=newValue;
	der->average = (der->prevDerivatives[der->lastIndex])/(AVERAGE_FILTER_SAMPLES*1.);
	return der->average;
}

float getDerivative(struct derivativeAverager *der)
{
	return der->average;
}
