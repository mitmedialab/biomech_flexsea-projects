#include "spline_functions.h"


void initializeCubicSplineParams(CubicSpline *cSpline, Act_s *actx, GainParams gainParams, float res_factor){
	cSpline->time_state = 0;
	cSpline->res_factor = res_factor; // delta X (time)
	cSpline->theta_set_fsm = gainParams.thetaDes; // Initial joint angle - theta_set_fsm = delta Y (joint angle)
	cSpline->xi_1 = 0.0; // Initial X (time) coordinate
	cSpline->x_int_1 = (cSpline->res_factor/2.0)*.4;
	cSpline->xf_1 = cSpline->res_factor/2.0;
	cSpline->yi_1 = actx->jointAngleDegrees; // Initial Y (joint angle) coordinate
	cSpline->yf_1 = cSpline->yi_1 + ((cSpline->theta_set_fsm - cSpline->yi_1)/2.0);
	cSpline->y_int_1 = cSpline->yi_1 - ((cSpline->yi_1 - cSpline->yf_1) * .15);
	cSpline->xi_2 = cSpline->res_factor/2.0;
	cSpline->x_int_2 = (cSpline->res_factor-(cSpline->res_factor/2.0))*.6+(cSpline->res_factor/2.0);
	cSpline->xf_2 = cSpline->res_factor; // Final X (time) coordinate
	cSpline->yi_2 = cSpline->yi_1 + ((cSpline->theta_set_fsm - cSpline->yi_1)/2.0);
	cSpline->yf_2 = cSpline->theta_set_fsm; // Final Y (joint angle) coordinate
	cSpline->y_int_2 = cSpline->yf_2 + ((cSpline->yi_2 - cSpline->yf_2) * .15);
	solveTridiagonalMatrix(cSpline);
}

void solveTridiagonalMatrix(CubicSpline *cSpline){ // Solves the matrix and finds the coefficients for the functions
	float B[3], A[2], C[2], r[3];
	float e[3], f[3], g[2];
	float x[3];
	float y[3];
	int n = 3; // f vector length
	float factor;
	float k[3];
	float a1, a2, b1, b2;
	x[0] = cSpline->xi_1;
	x[1] = cSpline->x_int_1;
	x[2] = cSpline->xf_1;
	y[0] = cSpline->yi_1;
	y[1] = cSpline->y_int_1;
	y[2] = cSpline->yf_1;

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
	cSpline->a1_1 = a1;
	cSpline->a2_1 = a2;
	cSpline->b1_1 = b1;
	cSpline->b2_1 = b2;
	// -----S curve complementary trajectory-----
	x[0] = cSpline->xi_2;
	x[1] = cSpline->x_int_2;
	x[2] = cSpline->xf_2;
	y[0] = cSpline->yi_2;
	y[1] = cSpline->y_int_2;
	y[2] = cSpline->yf_2;

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
	cSpline->a1_2 = a1;
	cSpline->a2_2 = a2;
	cSpline->b1_2 = b1;
	cSpline->b2_2 = b2;
}

void calcCubicSpline(CubicSpline *cSpline){ // Computes and evaluates the cubic spline trajectory. cSpline->Y is the interpolation value
	float t;
	float q[2];
	float q2[2];
	float x[3];
	float x2[3];
	float y[3];
	float y2[3];
	x[0] = cSpline->xi_1;
	x[1] = cSpline->x_int_1;
	x[2] = cSpline->xf_1;
	y[0] = cSpline->yi_1;
	y[1] = cSpline->y_int_1;
	y[2] = cSpline->yf_1;
	x2[0] = cSpline->xi_2;
	x2[1] = cSpline->x_int_2;
	x2[2] = cSpline->xf_2;
	y2[0] = cSpline->yi_2;
	y2[1] = cSpline->y_int_2;
	y2[2] = cSpline->yf_2;

	if (cSpline->time_state <= (cSpline->res_factor/2.0)){
		t = ((float)cSpline->time_state - x[0]) / (x[1]-x[0]);
		q[0] = (1-t)*y[0] + t*y[1] + (t*(1-t)*(cSpline->a1_1*(1-t)+(cSpline->b1_1*t)));
		t = ((float)cSpline->time_state - x[1]) / (x[2]-x[1]);
		q[1] = (1-t)*y[1] + t*y[2] + (t*(1-t)*(cSpline->a2_1*(1-t)+(cSpline->b2_1*t)));
		if(cSpline->time_state <= ((cSpline->res_factor/2.0)*.4))
			cSpline->Y = q[0];
			else cSpline->Y = q[1];
		}
	else{
		t = ((float)cSpline->time_state - x2[0]) / (x2[1]-x2[0]);
		q2[0] = (1-t)*y2[0] + t*y2[1] + (t*(1-t)*(cSpline->a1_2*(1-t)+(cSpline->b1_2*t)));
		t = ((float)cSpline->time_state - x2[1]) / (x2[2]-x2[1]);
		q2[1] = (1-t)*y2[1] + t*y2[2] + (t*(1-t)*(cSpline->a2_2*(1-t)+(cSpline->b2_2*t)));
		if(cSpline->time_state <= ((cSpline->res_factor-(cSpline->res_factor/2.0))*.6+(cSpline->res_factor/2.0)))
			cSpline->Y = q2[0];
		else cSpline->Y = q2[1];
	}

	if((cSpline->yi_1 - cSpline->theta_set_fsm) > 0){
		if(cSpline->Y < cSpline->theta_set_fsm)
			cSpline->Y = cSpline->theta_set_fsm;
	}
	else{
		if(cSpline->Y > cSpline->theta_set_fsm)
			cSpline->Y = cSpline->theta_set_fsm;
	}

	cSpline->time_state++;
}
