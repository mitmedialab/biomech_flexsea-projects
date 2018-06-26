/*
 * runningExo-parameters.c
 *
 *  Created on: Jun 19, 2018
 *      Author: Albert Wu
 */
#include "runningExo-parameters.h"
#include "runningExo-structs.h"
int mod (int a, int b) {
	int ret = a % b;
	if(ret < 0)
		ret+=b;
	return ret;
}

float updateDerivativeAverage(derivativeAverager *der,float newValue)
{
	der->lastIndex++;
	der->lastIndex =  mod(der->lastIndex,AVERAGE_FILTER_SAMPLES);
	der->sum-=der->prevDerivatives[der->lastIndex];
	der->sum+=newValue;
	der->prevDerivatives[der->lastIndex]=newValue;
	der->average = (der->sum)/(AVERAGE_FILTER_SAMPLES*1.);
	return der->average;
}

float getDerivativeAverage(struct derivativeAverager *der)
{
	return der->average;
}

float updateDerivativeFilter(derivativeFilter *der, float newValue)
{
	//Update values
	der->lastIndex++;
	der->lastIndex = (der->lastIndex<LPF_SAMPLE_COUNT ? der->lastIndex : 0);
	der->prevDerivatives[der->lastIndex]=newValue;
	//Convolution
	der->result = 0.;
	for(int i = 0; i < LPF_SAMPLE_COUNT; i++)
	{
		int wrappedIndexConvolutor = mod((i+der->lastIndex),LPF_SAMPLE_COUNT);
		int wrappedIndexConvoluted = mod((0-i+der->lastIndex),LPF_SAMPLE_COUNT);
		der->result+=der->prevDerivatives[wrappedIndexConvoluted]*LPF_COEFFICIENTS[wrappedIndexConvolutor];
	}
	return der->result;
}

float getDerivativeFilter(derivativeFilter* der, float newValue)
{
	return der->result;
}
