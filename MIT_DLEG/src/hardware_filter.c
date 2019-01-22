#include "hardware_filter.h"

//****************************************************************************
// Definitions
//****************************************************************************
// LPF IS LOW PASS FILTER

#ifdef LPF1 // Passband 100Hz, Stopband 200Hz
#define N 26
float firFilterHw[N] = {
	0.00092,0.00336,0.00667,0.00755,0.00103,-0.01548,-0.03696,-0.04899,-0.03326,0.0209,
	0.10577,0.19408,0.25055,0.25055,0.19408,0.10577,0.0209,-0.03326,-0.04899,-0.03696,
	-0.01548,0.00103,0.00755,0.00667,0.00336,0.00092
};
#endif

#ifdef LPF2 // Passband 50Hz, Stopband 100Hz
#define N 51
float firFilterHw[N] = {
	0.00029,0.00062,0.00113,0.00174,0.0023,0.00257,0.00228,0.00112,-0.00116,-0.00464,
	-0.00916,-0.01428,-0.01924,-0.02301,-0.02443,-0.02236,-0.01591,-0.00461,0.0114,0.031290,
	0.05362,0.07638,0.09734,0.11425,0.12524,0.12905,0.12524,0.11425,0.09734,0.07638,
	0.05362,0.03129,0.0114,-0.00461,-0.01591,-0.02236,-0.02443,-0.02301,-0.01924,-0.01428,
	-0.00916,	-0.00464,-0.00116,0.00112,0.00228,0.00257,0.0023,0.00174,0.00113,0.00062,
	0.00029
};
#endif

#ifdef LPF3 // Passband 50Hz, Stopband 70Hz

//float firFilterHw[N] = {
//	0.00002,-0.00012,-0.00024,-0.00045,-0.00075,-0.00116,-0.00167,-0.00226,-0.00291,-0.00357,
//	-0.00418,-0.00468,-0.00499,-0.00505,-0.00481,-0.00424,-0.00333,-0.00214,-0.00073,0.00078,
//	0.00226,0.00355,0.0045,0.00497,0.00489,0.00419,0.00293,0.00118,-0.00087,-0.00301,
//	-0.00498,-0.00652,-0.00738,-0.0074,-0.00647,-0.0046,-0.00194,0.00127,0.00469,0.0079,
//	0.01047,0.012,0.01214,0.01072,0.00769,0.00325,-0.00226,-0.00828,-0.01412,-0.01902,
//	-0.0222,-0.02297,-0.02076,-0.01525,-0.00638,0.00561,0.02017,0.0365,0.05358,0.07027,
//	0.08539,0.09785,0.10672,0.11134,0.11134,0.10672,0.09785,0.08539,0.07027,0.05358,
//	0.0365,0.02017,0.00561, -0.00638,-0.01525,-0.02076,-0.02297,-0.0222,-0.01902,-0.01412,
//	-0.00828,-0.00226,0.00325,0.00769,0.01072,0.01214,0.012,0.01047,0.0079,0.00469,
//	0.00127,-0.00194,-0.0046,-0.00647,-0.0074,-0.00738,-0.00652,-0.00498,-0.00301,-0.00087,
//	0.00118,0.00293,0.00419,0.00489, 0.00497,0.0045,0.00355,0.00226,0.00078,-0.00073,
//	-0.00214,-0.00333,-0.00424,-0.00481,-0.00505,-0.00499,-0.00468,-0.00418,-0.00357,-0.00291,
//	-0.00226,-0.00167,-0.00116,	-0.00075,-0.00045,-0.00024,-0.00012,0.00002
//};
#define N 145
// Passband 35Hz, Stopband 70Hz
float firFilterHw[N] = {
		-1.16056454095328e-05,-1.74755813296825e-05,-2.21105457303896e-05,-2.33020676380724e-05,
		-1.86708800205725e-05,-6.15861968228209e-06,1.53838813625615e-05,4.56063149007905e-05,
		8.21870080295132e-05,0.000120608206131029,0.000154292853838001,0.000175177608791988,
		0.000174733998475374,0.000145370111717043,8.20598521990715e-05,-1.60304150145268e-05,
		-0.000144201739878730,-0.000291469088423208,-0.000440662892955870,-0.000569570466885628,
		-0.000653170483867974,-0.000666846719723775,-0.000590285555236064,-0.000411592033377275,
		-0.000131030906813687,0.000236260181117980,0.000659207048057817,0.00109187408269524,
		0.00147693683983838,0.00175150103360784,0.00185486142770685,0.00173743882793505,0.00136983075140661,
		0.000750713843065251,-8.77116653426879e-05,-0.00107791407157737,-0.00211972017041419,
		-0.00308787896358523,-0.00384426600169555,-0.00425383017698588,-0.00420274756872796,
		-0.00361672805523459,-0.00247710506495353,-0.000832294672975838,0.00119752386250386,
		0.00342407628057443,0.00560373009237651,0.00745897634148950,0.00870720871799600,
		0.00909428213750828,0.00842945673364543,0.00661777978461936,0.00368582675465097,
		-0.000202931695194902,-0.00474604718049951,-0.00951496536285803,-0.0139829782150149,
		-0.0175666967827158,-0.0196778444955014,-0.0197807731137787,-0.0174501263958376,
		-0.0124226599204468,-0.00463744996729914,0.00574040062462775,0.0183159924626163,
		0.0324823222623095,0.0474588070112176,0.0623478299755545,0.0762045414273700,
		0.0881136341963571,0.0972659250272520,0.103027419512752,0.104994152212658,0.103027419512752,
		0.0972659250272520,0.0881136341963571,0.0762045414273700,0.0623478299755545,0.0474588070112176,
		0.0324823222623095,0.0183159924626163,0.00574040062462775,-0.00463744996729914,
		-0.0124226599204468,-0.0174501263958376,-0.0197807731137787,-0.0196778444955014,
		-0.0175666967827158,-0.0139829782150149,-0.00951496536285803,-0.00474604718049951,
		-0.000202931695194902,0.00368582675465097,0.00661777978461936,0.00842945673364543,
		0.00909428213750828,0.00870720871799600,0.00745897634148950,0.00560373009237651,
		0.00342407628057443,0.00119752386250386,-0.000832294672975838,-0.00247710506495353,
		-0.00361672805523459,-0.00420274756872796,-0.00425383017698588,-0.00384426600169555,
		-0.00308787896358523,-0.00211972017041419,-0.00107791407157737,-8.77116653426879e-05,
		0.000750713843065251,0.00136983075140661,0.00173743882793505,0.00185486142770685,
		0.00175150103360784,0.00147693683983838,0.00109187408269524,0.000659207048057817,
		0.000236260181117980,-0.000131030906813687,-0.000411592033377275,-0.000590285555236064,
		-0.000666846719723775,-0.000653170483867974,-0.000569570466885628,-0.000440662892955870,
		-0.000291469088423208,-0.000144201739878730,-1.60304150145268e-05,8.20598521990715e-05,
		0.000145370111717043,0.000174733998475374,0.000175177608791988,0.000154292853838001,
		0.000120608206131029,8.21870080295132e-05,4.56063149007905e-05,1.53838813625615e-05,
		-6.15861968228209e-06,-1.86708800205725e-05,-2.33020676380724e-05,-2.21105457303896e-05,
		-1.74755813296825e-05,-1.16056454095328e-05
};
#endif

#ifdef LPF4 // Passband 35Hz, Stopband 70Hz
#define N 73tauRestoring(float) -
float firFilterHw[N] = {
	0.0002,0.00032,0.00053,0.0008,0.00111,0.00143,0.00171,0.0019,0.00192,0.0017,
	0.00116,0.00025,-0.00107,-0.00281,-0.00492,-0.00733,-0.00989,-0.01239,-0.01461,-0.01626,
	-0.01707,-0.01675,-0.01506,-0.01181,-0.00689,-0.00029,0.00788,0.0174,0.02794,0.03907,
	0.05028,0.06103,0.07076,0.07896,0.08517,0.08904,0.09036,0.08904,0.08517,0.07896,
	0.07076,0.06103,0.05028,0.03907,0.02794,0.0174,0.00788,-0.00029,-0.00689,-0.01181,
	-0.01506,-0.01675,-0.01707,-0.01626,-0.01461,-0.01239,-0.00989,-0.00733,-0.00492,-0.00281,
	-0.00107,0.00025,0.00116,0.0017,0.00192,0.0019,0.00171,0.00143,0.00111,0.0008,
	0.00053,0.00032,0.0002
};
#endif
float32_t lpfBuffer[N];

arm_fir_instance_f32 SLpf;
float32_t lpfOut;
float32_t lpfInput[N*2];
uint16_t lpfIndex=0;
float lpfResult;

//****************************************************************************
// Method(s)
//****************************************************************************

/*
 *  Initializes the lpf
 */
void init_LPF(void)
{
	arm_fir_init_f32(&SLpf, N, (float32_t *) &firFilterHw[0], (float32_t *) &lpfBuffer[0], 1);
	lpfIndex = 0;
	memset(lpfInput,0, N*2*4); //4byte per float32_t * twice larger circular buffer
	lpfOut = 0;
	lpfResult = 0;

	return;
}

/*
 *  updates low pass filter with new value
 *  Param: val(float) - value to push into the lpf input
 *  Updates the lpfInput:
 *  	increases lpfIndex by one
 *  	replaces value at lpfIndex and lpfIndex+N to the given value
 *
 *  Return: null
 */
void update_LPF(float val)
{
	lpfIndex++;
	if(lpfIndex>=N)
		lpfIndex = 0;

	lpfInput[lpfIndex] = (float32_t)val;
	lpfInput[lpfIndex+N] = (float32_t)val;

	return;
}

/*
 *  applies low pass filter and returns output
 *  Param: val(float) - val(float) - value to push into the lpf input
 *  Return: lpfOutput(float) - the output of the lpf
 */
float filter_LPF(float val)
{
	update_LPF(val);
    arm_fir_f32(&SLpf, (float32_t *)( &lpfInput[lpfIndex] ) , (float32_t*) &lpfOut, 1);

    return (float)lpfOut;
}
