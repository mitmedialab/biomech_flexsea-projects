/*
 * spline_functions.c
 *
 *  Created on:
 *      Author:
 */

//****************************************************************************
// Include(s)
//****************************************************************************
#include "spline_functions.h"

//****************************************************************************
// Method(s)
//****************************************************************************

/*
 *  Initializes the cubic spline parameters
 *  Param: cSpline(CubicSpline) -
 *  Param: actx(Act_s) -
 *  Param: gainParams(GainParams) -
 *  Param: gainParams(float) -
 */
void initializeCubicSplineParams(CubicSpline *cSpline, Act_s *actx, GainParams gainParams, float resFactor){
	cSpline->timeState = 0;
	cSpline->resFactor = resFactor; // delta X (time)
	cSpline->thetaSetFsm = gainParams.thetaDes; // Initial joint angle - thetaSetFsm = delta Y (joint angle)
	cSpline->xi1 = 0.0; // Initial X (time) coordinate
	cSpline->xInt1 = (cSpline->resFactor/2.0)*.4;
	cSpline->xf1 = cSpline->resFactor/2.0;
	cSpline->yi1 = actx->jointAngleDegrees; // Initial Y (joint angle) coordinate
	cSpline->yf1 = cSpline->yi1 + ((cSpline->thetaSetFsm - cSpline->yi1)/2.0);
	cSpline->yInt1 = cSpline->yi1 - ((cSpline->yi1 - cSpline->yf1) * .15);
	cSpline->xi2 = cSpline->resFactor/2.0;
	cSpline->xInt2 = (cSpline->resFactor-(cSpline->resFactor/2.0))*.6+(cSpline->resFactor/2.0);
	cSpline->xf2 = cSpline->resFactor; // Final X (time) coordinate
	cSpline->yi2 = cSpline->yi1 + ((cSpline->thetaSetFsm - cSpline->yi1)/2.0);
	cSpline->yf2 = cSpline->thetaSetFsm; // Final Y (joint angle) coordinate
	cSpline->yInt2 = cSpline->yf2 + ((cSpline->yi2 - cSpline->yf2) * .15);
	solveTridiagonalMatrix(cSpline);
}

/*
 *  Solves the matrix and finds the coefficients for the functions
 *  Param: cSpline(CubicSpline) -
 */
void solveTridiagonalMatrix(CubicSpline *cSpline){
	float B[3], A[2], C[2], r[3];
	float e[3], f[3], g[2];
	float x[3];
	float y[3];
	int n = 3; // f vector length
	float factor;
	float k[3];
	float a1, a2, b1, b2;
	x[0] = cSpline->xi1;
	x[1] = cSpline->xInt1;
	x[2] = cSpline->xf1;
	y[0] = cSpline->yi1;
	y[1] = cSpline->yInt1;
	y[2] = cSpline->yf1;

	B[0] = 2.0 / (x[1] - x[0]);
	B[1] = 2.0 * ((1/(x[1]-x[0])) + (1/(x[2]-x[1])));
	B[2] = 2.0 / (x[2]-x[1]);
	A[0] = 1.0 / (x[1]-x[0]);
	A[1] = 1.0 / (x[2]-x[1]);
	C[0] = 1.0 / (x[1]-x[0]);
	C[1] = 1.0 / (x[2]-x[1]);
	r[0] = 3.0 * ((y[1]-y[0])/(pow(x[1]-x[0],2)));
	r[1] = 3.0 * (((y[1]-y[0])/(pow(x[1]-x[0],2))) + ((y[2]-y[1])/(pow(x[2]-x[1],2))));
	r[2] = 3.0 * ((y[2]-y[1])/(pow(x[2]-x[1],2)));

	e[0] = 0;
	e[1] = A[0];
	e[2] = A[1];
	for(int i=0; i<3; i++){
		f[i] = B[i];
	}
	g[0] = C[0];
	g[1] = C[1];
	// Forward elimination
	for(int i = 1; i < n; i++){
		factor = e[i] / f[i-1];
		f[i] = f[i] - (factor * g[i-1]);
		r[i] = r[i] - (factor * r[i-1]);
	}
	// Back substitution
	k[n-1] = r[n-1] / f[n-1];
	for(int i = 1; i >= 0; i--){
		k[i] = (r[i] - (g[i] * k[i+1])) / f[i];
	}
	// ai and bi computation
	a1 = k[0]*(x[1]-x[0]) - (y[1]-y[0]);
	a2 = k[1]*(x[2]-x[1]) - (y[2]-y[1]);
	b1 = -1.0*k[1]*(x[1]-x[0]) + (y[1]-y[0]);
	b2 = -1.0*k[2]*(x[2]-x[1]) + (y[2]-y[1]);
	cSpline->a11 = a1;
	cSpline->a21 = a2;
	cSpline->b11 = b1;
	cSpline->b21 = b2;
	// -----S curve complementary trajectory-----
	x[0] = cSpline->xi2;
	x[1] = cSpline->xInt2;
	x[2] = cSpline->xf2;
	y[0] = cSpline->yi2;
	y[1] = cSpline->yInt2;
	y[2] = cSpline->yf2;

	B[0] = 2.0 / (x[1] - x[0]);
	B[1] = 2.0 * ((1/(x[1]-x[0])) + (1/(x[2]-x[1])));
	B[2] = 2.0 / (x[2]-x[1]);
	A[0] = 1.0 / (x[1]-x[0]);
	A[1] = 1.0 / (x[2]-x[1]);
	C[0] = 1.0 / (x[1]-x[0]);
	C[1] = 1.0 / (x[2]-x[1]);
	r[0] = 3.0 * ((y[1]-y[0])/(pow(x[1]-x[0],2)));
	r[1] = 3.0 * (((y[1]-y[0])/(pow(x[1]-x[0],2))) + ((y[2]-y[1])/(pow(x[2]-x[1],2))));
	r[2] = 3.0 * ((y[2]-y[1])/(pow(x[2]-x[1],2)));

	e[0] = 0;
	e[1] = A[0];
	e[2] = A[1];
	for(int i=0; i<3; i++){
		f[i] = B[i];
	}
	g[0] = C[0];
	g[1] = C[1];
	// Forward elimination
	for(int i = 1; i < n; i++){
		factor = e[i] / f[i-1];
		f[i] = f[i] - (factor * g[i-1]);
		r[i] = r[i] - (factor * r[i-1]);
	}
	// Back substitution
	k[n-1] = r[n-1] / f[n-1];
	for(int i = 1; i >= 0; i--){
		k[i] = (r[i] - (g[i] * k[i+1])) / f[i];
	}
	// ai and bi computation
	a1 = k[0]*(x[1]-x[0]) - (y[1]-y[0]);
	a2 = k[1]*(x[2]-x[1]) - (y[2]-y[1]);
	b1 = -1.0*k[1]*(x[1]-x[0]) + (y[1]-y[0]);
	b2 = -1.0*k[2]*(x[2]-x[1]) + (y[2]-y[1]);
	cSpline->a12 = a1;
	cSpline->a22 = a2;
	cSpline->b12 = b1;
	cSpline->b22 = b2;
}

/*
 *  Computes and evaluates the cubic spline trajectory. cSpline->Y is the interpolation value
 *  Param: cSpline(CubicSpline) -
 */
void calcCubicSpline(CubicSpline *cSpline){
	float t;
	float q[2];
	float q2[2];
	float x[3];
	float x2[3];
	float y[3];
	float y2[3];
	x[0] = cSpline->xi1;
	x[1] = cSpline->xInt1;
	x[2] = cSpline->xf1;
	y[0] = cSpline->yi1;
	y[1] = cSpline->yInt1;
	y[2] = cSpline->yf1;
	x2[0] = cSpline->xi2;
	x2[1] = cSpline->xInt2;
	x2[2] = cSpline->xf2;
	y2[0] = cSpline->yi2;
	y2[1] = cSpline->yInt2;
	y2[2] = cSpline->yf2;

	if (cSpline->timeState <= (cSpline->resFactor/2.0)){
		t = ((float)cSpline->timeState - x[0]) / (x[1]-x[0]);
		q[0] = (1-t)*y[0] + t*y[1] + (t*(1-t)*(cSpline->a11*(1-t)+(cSpline->b11*t)));
		t = ((float)cSpline->timeState - x[1]) / (x[2]-x[1]);
		q[1] = (1-t)*y[1] + t*y[2] + (t*(1-t)*(cSpline->a21*(1-t)+(cSpline->b21*t)));
		if(cSpline->timeState <= ((cSpline->resFactor/2.0)*.4))
			cSpline->Y = q[0];
			else cSpline->Y = q[1];
		}
	else{
		t = ((float)cSpline->timeState - x2[0]) / (x2[1]-x2[0]);
		q2[0] = (1-t)*y2[0] + t*y2[1] + (t*(1-t)*(cSpline->a12*(1-t)+(cSpline->b12*t)));
		t = ((float)cSpline->timeState - x2[1]) / (x2[2]-x2[1]);
		q2[1] = (1-t)*y2[1] + t*y2[2] + (t*(1-t)*(cSpline->a22*(1-t)+(cSpline->b22*t)));
		if(cSpline->timeState <= ((cSpline->resFactor-(cSpline->resFactor/2.0))*.6+(cSpline->resFactor/2.0)))
			cSpline->Y = q2[0];
		else cSpline->Y = q2[1];
	}

	if((cSpline->yi1 - cSpline->thetaSetFsm) > 0){
		if(cSpline->Y < cSpline->thetaSetFsm)
			cSpline->Y = cSpline->thetaSetFsm;
	}
	else{
		if(cSpline->Y > cSpline->thetaSetFsm)
			cSpline->Y = cSpline->thetaSetFsm;
	}

	cSpline->timeState++;
}
