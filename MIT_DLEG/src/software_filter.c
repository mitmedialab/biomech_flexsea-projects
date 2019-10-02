/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-manage' Mid-level computing, and networking
	Copyright (C) 2017 Dephy, Inc. <http://dephy.com/>
*****************************************************************************
	[Lead developper] Jean-Francois (JF) Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors] Seong Ho Yeon, FIR filter implemntation on MIT ankle
*****************************************************************************
	[This file] FIR filter
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2018-03-20 | syeon | Initial release
	*
****************************************************************************/


//****************************************************************************
// Include(s)
//****************************************************************************
#include <software_filter.h>
//****************************************************************************
// Variable(s)
//****************************************************************************

//#define FIR_FILTER
#ifdef FIR_FILTER

	#ifdef LPF1 // Passband 100Hz, Stopband 200Hz
	#define N 26
	float firFilter[N] = {
		0.00092,0.00336,0.00667,0.00755,0.00103,-0.01548,-0.03696,-0.04899,-0.03326,0.0209,
		0.10577,0.19408,0.25055,0.25055,0.19408,0.10577,0.0209,-0.03326,-0.04899,-0.03696,
		-0.01548,0.00103,0.00755,0.00667,0.00336,0.00092
	};
	#endif

	#ifdef LPF2 // Passband 50Hz, Stopband 100Hz
	#define N 51
	float firFilter[N] = {
		0.00029,0.00062,0.00113,0.00174,0.0023,0.00257,0.00228,0.00112,-0.00116,-0.00464,
		-0.00916,-0.01428,-0.01924,-0.02301,-0.02443,-0.02236,-0.01591,-0.00461,0.0114,0.031290,
		0.05362,0.07638,0.09734,0.11425,0.12524,0.12905,0.12524,0.11425,0.09734,0.07638,
		0.05362,0.03129,0.0114,-0.00461,-0.01591,-0.02236,-0.02443,-0.02301,-0.01924,-0.01428,
		-0.00916,	-0.00464,-0.00116,0.00112,0.00228,0.00257,0.0023,0.00174,0.00113,0.00062,
		0.00029
	};
	#endif

	#ifdef LPF3 // Passband 50Hz, Stopband 70Hz

	//float firFilter[N] = {
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
	float firFilter[N] = {
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
	#define N 73
	float firFilter[N] = {
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

	float firInput[2*N];
	float firAccbuf;
	float firOutput;
	uint8_t firIndex;

	//****************************************************************************
	// Method(s)
	//****************************************************************************

	/*
	 *  Initializes the ______
	 */
	void initMitFir(void)
	{
		memset(firInput,0, 2*N);
		firAccbuf = 0;
		firOutput = 0;
		firIndex = 0;
		return;
	}

	/*
	 *  Definition
	 *  Param: n(float) -
	 *  Return: firOutput(float) -
	 */
	float mitFirFilter1kHz(float input)
	{
		mitFirLatchInput(input);

		firOutput=0;
		for(uint8_t i=0;i<N;i++)
		{
			firOutput += firFilter[i] * firInput[firIndex+N-i];
		}

		return firOutput;
	}

	/*
	 *  Definition
	 *  Param: n(float) -
	 */
	void mitFirLatchInput(float n)
	{
		firIndex++;
		if(firIndex>=N)
			firIndex = 0;

		firInput[firIndex] = n;
		firInput[firIndex+N] = n;
		return;
	}
#endif


/*
 * Matt's Version of Software FIR Filter
 *
 */

#define FIR_FILTER_SOFT_MATT

#ifdef FIR_FILTER_SOFT_MATT
	//	#define SOFT_FIR_LPF1
	//	#define SOFT_FIR_LPF2
	//	#define SOFT_FIR_LPF3
//	#define SOFT_FIR_LPF4
#define SOFT_FIR_LPF5


	//#define SOFT_FILTER_DEFAULT_IIR_30HZ
	//#define SOFT_FILTER_DEFAULT_IIR_50HZ
	//#define SOFT_FILTER_TORQ_IIR_10HZ
	#define SOFT_FILTER_TORQ_IIR_15HZ
//	#define SOFT_FILTER_ENCTORQ_IIR_30HZ
	#define SOFT_FILTER_ENCTORQ_IIR_20HZ
	//#define SOFT_FILTER_ENCTORQ_IIR_50HZ
	//#define SOFT_FILTER_TORQ_IIR_20HZ			// torque signal
	#define SOFT_FILTER_DERIVATIVE_IIR_15HZ		// PID, D filter

	#define SOFT_FILTER_JOINTANGLE_IIR_20HZ		// joint angle
	#define SOFT_FILTER_JOINTVEL_IIR_20HZ


		#ifdef SOFT_FIR_LPF1 // Passband 200Hz
		#define FIR_SAMPLE_SIZE 27
		static float SoftFirFilter[FIR_SAMPLE_SIZE] = {
		0.00185701310086635,	0.00234789890703083,	0.00224971212249315,	-1.52075942009649e-18,
		-0.00577510528507341,	-0.0142161977520060,	-0.0208887135102763,	-0.0185085527538822,
		5.46425834385188e-18,	0.0373612283679754,	0.0889516569822084,	0.142932290865991,	0.184007885117237,
		0.199361767674871,	0.184007885117237,	0.142932290865991,	0.0889516569822084,	0.0373612283679754,
		5.46425834385188e-18,	-0.0185085527538822,	-0.0208887135102763,	-0.0142161977520060,
		-0.00577510528507341,	-1.52075942009649e-18,	0.00224971212249315,	0.00234789890703083,	0.00185701310086635
		};
		#endif

		#ifdef SOFT_FIR_LPF2 // Passband 50Hz, Stopband 300Hz
			#define FIR_SAMPLE_SIZE 15
			static float SoftFirFilter[FIR_SAMPLE_SIZE] = {
					0.0081,    0.0138,    0.0299,    0.0546,    0.0836,    0.1109,    0.1304,    0.1374,    0.1304,    0.1109,    0.0836,    0.0546,    0.0299,    0.0138,    0.0081
			};
		#endif

	#ifdef SOFT_FIR_LPF3 // Passband 60Hz, Stopband 250Hz
		#define FIR_SAMPLE_SIZE 21
		static float SoftFirFilter[FIR_SAMPLE_SIZE] = {
				0.00408480247660398,	0.00606704636866225,	0.0112422678021466,	0.0200292076167293,	0.0322114903733109,	0.0469089971372014,	0.0626753242626235,	0.0777082783059545,	0.0901373446917016,	0.0983357817111039,	0.101198918507924,	0.0983357817111039,	0.0901373446917016,	0.0777082783059545,	0.0626753242626235,	0.0469089971372014,	0.0322114903733109,	0.0200292076167293,	0.0112422678021466,	0.00606704636866225,	0.00408480247660398
		};
	#endif

	#ifdef SOFT_FIR_LPF4 // Passband 10Hz, Stopband 500Hz
		#define FIR_SAMPLE_SIZE 21
		static float SoftFirFilter[FIR_SAMPLE_SIZE] = {
				0.00638568446274278,	0.00845584707235466,	0.0142525476918065,	0.0234801800773368,	0.0354080822148568,	0.0489427470450588,	0.0627487174980378,	0.0754035452889812,	0.0855673242094410,	0.0921450324563356,	0.0944205839660966,	0.0921450324563356,	0.0855673242094410,	0.0754035452889812,	0.0627487174980378,	0.0489427470450588,	0.0354080822148568,	0.0234801800773368,	0.0142525476918065,	0.00845584707235466,	0.00638568446274278
		};
	#endif

	#ifdef SOFT_FIR_LPF5 // Passband 10Hz, Stopband 500Hz
		#define FIR_SAMPLE_SIZE 8+1
		static float SoftFirFilter[FIR_SAMPLE_SIZE] = {
				 0.0180933120901105,
				        0.0486911282615515,
				         0.122674289447104,
				         0.196785631464519,
				          0.22751127747343,
				         0.196785631464519,
				         0.122674289447104,
				        0.0486911282615515,
				        0.0180933120901105
		};
	#endif



		static float softFirFiltBuffer[FIR_SAMPLE_SIZE];
		static float softFirFiltOutput;
		static int16_t softFirIndex;
		static int16_t circCounter;


		void initSoftFIRFilt(void)
		{
			memset(softFirFiltBuffer, 0, FIR_SAMPLE_SIZE*4);	// initialize input blocks
			softFirFiltOutput = 0;							// initialize output
			softFirIndex = 0;								// initialize index
		}

		float runSoftFirFilt(float inputVal)
		{
			float result = 0;
			int16_t i = 0;
			softFirFiltBuffer[0] = inputVal;		// load new value into bottom of index

			// filter the input using the accumulator
			for (i = 0; i < FIR_SAMPLE_SIZE;  i++)
			{
				result += SoftFirFilter[FIR_SAMPLE_SIZE-1-i] * softFirFiltBuffer[i];
			}

			// Update the Filter History from reverse, incrementing saved values up one
			for ( i = FIR_SAMPLE_SIZE-1; i > 0; i--)
			{
				softFirFiltBuffer[i] = softFirFiltBuffer[i-1];
			}

			return result;
		}


		void initCircularSoftFIRFilt(void)
		{
			memset(softFirFiltBuffer, 0, FIR_SAMPLE_SIZE);	// initialize input blocks
			softFirFiltOutput = 0;							// initialize output
			softFirIndex = 0;
			circCounter = 0;// initialize index
		}

		float runCircularSoftFirFilt(float inputVal)
		{
			float result = 0;
			int16_t i = 0;

			++circCounter;	//increment circular counter
			if(circCounter == FIR_SAMPLE_SIZE)
			{
				circCounter = 0;	//reset counter
			}

			softFirFiltBuffer[circCounter] = inputVal;		// load new value into bottom of index
			softFirIndex = circCounter;


			// filter the input using the accumulator
			for (i = 0; i < FIR_SAMPLE_SIZE;  i++)
			{
				result += SoftFirFilter[i] * softFirFiltBuffer[softFirIndex];
				--softFirIndex;
				if(softFirIndex == -1)
				{
					softFirIndex = FIR_SAMPLE_SIZE-1;
				}
			}


			return result;
		}

	#ifdef SOFT_FILTER_DEFAULT_IIR_30HZ
		/* Digital filter designed by mkfilter/mkshape/gencode   A.J. Fisher, http://www-users.cs.york.ac.uk/~fisher/cgi-bin/mkfscript
		   Command line: /www/usr/fisher/helpers/mkfilter -Bu -Lp -o 2 -a 3.0000000000e-02 0.0000000000e+00 -l
		   Cutoff = 30Hz, 2nd order, Butterworth filter, Sampling at 1000Hz
		*/

		#define NZEROS 2
		#define NPOLES 2
		#define BUTWRTH_FILT_GAIN   1.278738361e+02

		static float xv30[NZEROS+1], yv30[NPOLES+1];

		float filterButterworth30Hz(float inputVal)
		{
			xv30[0] = xv30[1];
			xv30[1] = xv30[2];
			xv30[2] = inputVal / BUTWRTH_FILT_GAIN;
			yv30[0] = yv30[1];
			yv30[1] = yv30[2];
			yv30[2] = (xv30[0] + xv30[2]) + 2 * xv30[1]
						+ ( -0.7660066009 * yv[0]) + (  1.7347257688 * yv[1]);
			return yv30[2];

		}
	#endif //end 30Hz

	#ifdef SOFT_FILTER_DEFAULT_IIR_50HZ
		/* Digital filter designed by mkfilter/mkshape/gencode   A.J. Fisher, http://www-users.cs.york.ac.uk/~fisher/cgi-bin/mkfscript
		   Command line: /www/usr/fisher/helpers/mkfilter -Bu -Lp -o 2 -a 5.0000000000e-02 0.0000000000e+00 -l
		   Cutoff = 50Hz, 2nd order, Butterworth filter, Sampling at 1000Hz
		*/

		#define NZEROS 2
		#define NPOLES 2
		#define BUTWRTH_FILT_GAIN   4.979245121e+01

		static float xv50[NZEROS+1], yv50[NPOLES+1];

		float filterButterworth50Hz(float inputVal)
		{
			xv50[0] = xv50[1];
			xv50[1] = xv50[2];
			xv50[2] = inputVal / BUTWRTH_FILT_GAIN;
			yv50[0] = yv50[1];
			yv50[1] = yv50[2];
			yv50[2] = (xv50[0] + xv50[2]) + 2 * xv50[1]
						+ ( -0.6413515381 * yv[0]) + (  1.5610180758 * yv[1]);
			return yv50[2];

		}
	#endif //end 50Hz


	#ifdef SOFT_FILTER_TORQ_IIR_10HZ
		/* Digital filter designed by mkfilter/mkshape/gencode   A.J. Fisher, http://www-users.cs.york.ac.uk/~fisher/mkfilter/
		   Command line: /www/usr/fisher/helpers/mkfilter -Bu -Lp -o 2 -a 1.0000000000e-02 0.0000000000e+00 -l
		   Cutoff = 10Hz, 2nd order, Butterworth filter, Sampling at 1000Hz
		*/

		float filterTorqueButterworth(float inputVal)
		{
			#define NZEROS 2
			#define NPOLES 2
			#define BUTWRTH_FILT_GAIN   1.058546241e+03

			static float xv[NZEROS+1], yv[NPOLES+1];

			xv[0] = xv[1];
			xv[1] = xv[2];
			xv[2] = inputVal / BUTWRTH_FILT_GAIN;
			yv[0] = yv[1];
			yv[1] = yv[2];
			yv[2] =   (xv[0] + xv[2]) + 2 * xv[1]
						 + ( -0.9149758348 * yv[0]) + (  1.9111970674 * yv[1]);
			return yv[2];

		}
	#endif //end 10Hz

	#ifdef SOFT_FILTER_TORQ_IIR_15HZ
		/*  lpFilt = designfilt('lowpassiir','FilterOrder',2, ...
			 'PassbandFrequency',15,'PassbandRipple',0.1, ...
			 'SampleRate',1000);
			[b,a] = tf(lpFilt)
			[z p k ] = zpk(lpFilt)
			fvtool(lpFilt)
			M = idpoly(a,b,'NoiseVariance',0)
		   Cutoff = 15Hz, 2nd order, 0.1 ripple, Butterworth filter, Sampling at 1000Hz
		*/


		float filterTorqueButterworth(float inputVal)
		{
			#define NTZEROS 2
			#define NTPOLES 2

			static float xvT[NTZEROS+1], yvT[NTPOLES+1];

			xvT[0] = xvT[1];
			xvT[1] = xvT[2];
			xvT[2] = inputVal;
			yvT[0] = yvT[1];
			yvT[1] = yvT[2];
			yvT[2] = 1.77374445515908*yvT[1] - 0.800084509789504*yvT[0] + 0.00650963562958534*xvT[2] + 0.0130192712591707*xvT[1] + 0.00650963562958534*xvT[0];
			return yvT[2];
		}
	#endif //end 15Hz

		//fc=15hz
	float filterTorqueEncButterworth(float inputVal)
	{

		static float xvT[3]={0,0,0};
		static float yvT[3]={0,0,0};

		xvT[0] = xvT[1];
		xvT[1] = xvT[2];
		xvT[2] = inputVal;
		yvT[0] = yvT[1];
		yvT[1] = yvT[2];
		yvT[2] = 1.77374445515908*yvT[1] - 0.800084509789504*yvT[0] + 0.00650963562958534*xvT[2] + 0.0130192712591707*xvT[1] + 0.00650963562958534*xvT[0];
		return yvT[2];
	}

	//fc=15hz
	float filterMotorCommandButterworth(float inputVal)
	{

		static float xvT[3]={0,0,0};
		static float yvT[3]={0,0,0};

		xvT[0] = xvT[1];
		xvT[1] = xvT[2];
		xvT[2] = inputVal;
		yvT[0] = yvT[1];
		yvT[1] = yvT[2];
		yvT[2] = 1.77374445515908*yvT[1] - 0.800084509789504*yvT[0] + 0.00650963562958534*xvT[2] + 0.0130192712591707*xvT[1] + 0.00650963562958534*xvT[0];
		return yvT[2];
	}



	#ifdef SOFT_FILTER_TORQ_IIR_20HZ
		/* Digital filter designed by mkfilter/mkshape/gencode   A.J. Fisher, http://www-users.cs.york.ac.uk/~fisher/cgi-bin/mkfscript
		   Command line: /www/usr/fisher/helpers/mkfilter -Bu -Lp -o 2 -a 2.0000000000e-02 0.0000000000e+00 -l
		   Cutoff = 20Hz, 2nd order, Butterworth filter, Sampling at 1000Hz
		*/


		float filterTorqueButterworth(float inputVal)
		{

			#define NTZEROS 2
			#define NTPOLES 2
			#define BUTWRTH_FILT_GAIN_TORQ   2.761148367e+02

			static float xvT[NTZEROS+1], yvT[NTPOLES+1];

			xvT[0] = xvT[1];
			xvT[1] = xvT[2];
			xvT[2] = inputVal / BUTWRTH_FILT_GAIN_TORQ;
			yvT[0] = yvT[1];
			yvT[1] = yvT[2];
			yvT[2] = (xvT[0] + xvT[2]) + 2 * xvT[1]
					 + ( -0.8371816513 * yvT[0]) + (  1.8226949252 * yvT[1]);
			return yvT[2];

		}
	#endif //end 20Hz


	#ifdef SOFT_FILTER_ENCTORQ_IIR_50HZ
		/*  lpFilt = designfilt('lowpassiir','FilterOrder',2, ...
			 'PassbandFrequency',50,'PassbandRipple',0.2, ...
			 'SampleRate',1000);
			[b,a] = tf(lpFilt)
			[z p k ] = zpk(lpFilt)
			fvtool(lpFilt)
			M = idpoly(a,b,'NoiseVariance',0)
			fvtool(lpFilt)
		*/

		#define NTENCZEROS 2
		#define NTENCPOLES 2

		static float xvTENC[NTENCZEROS+1], yvTENC[NTENCPOLES+1];

		float filterEncoderTorqueButterworth(float inputVal)
		{
			xvTENC[0] = xvTENC[1];
			xvTENC[1] = xvTENC[2];
			xvTENC[2] = inputVal;
			yvTENC[0] = yvTENC[1];
			yvTENC[1] = yvTENC[2];
			yvTENC[2] = 1.37923908288931*yvTENC[1] - 0.552575367724063*yvTENC[0] + 0.0423476673206792*xvTENC[2] + 0.0846953346413584*xvTENC[1] + 0.0423476673206792*xvTENC[0];
			return yvTENC[2];
		}
	#endif //end 50Hz





	#ifdef SOFT_FILTER_ENCTORQ_IIR_20HZ
		/* Digital filter designed by mkfilter/mkshape/gencode   A.J. Fisher, http://www-users.cs.york.ac.uk/~fisher/cgi-bin/mkfscript
			   Command line: /www/usr/fisher/helpers/mkfilter -Bu -Lp -o 2 -a 2.0000000000e-02 0.0000000000e+00 -l */


		float filterEncoderTorqueButterworth(float inputVal)
		{
			#define NTENCZEROS 2
			#define NTENCPOLES 2
			#define BUTWRTH_ENCTQ_FILT_GAIN_VEL   2.761148367e+02

			static float xvTENC[NTENCZEROS+1];
			static float yvTENC2[NTENCPOLES+1];


			xvTENC[0] = xvTENC[1];
			xvTENC[1] = xvTENC[2];
			xvTENC[2] = inputVal/BUTWRTH_ENCTQ_FILT_GAIN_VEL;
			yvTENC2[0] = yvTENC2[1];
			yvTENC2[1] = yvTENC2[2];
			yvTENC2[2] = (xvTENC[0] + xvTENC[2]) + 2 * xvTENC[1]
							 + ( -0.8371816513 * yvTENC2[0]) + (  1.8226949252 * yvTENC2[1]);

			return yvTENC2[2];
		}
	#endif //end 20Hz

	#ifdef SOFT_FILTER_JOINTANGLE_IIR_20HZ
		/* Digital filter designed by mkfilter/mkshape/gencode   A.J. Fisher, http://www-users.cs.york.ac.uk/~fisher/cgi-bin/mkfscript
		   Command line: /www/usr/fisher/helpers/mkfilter -Bu -Lp -o 2 -a 2.0000000000e-02 0.0000000000e+00 -l */

		float filterJointAngleButterworth(float inputVal)
		{
			#define NJAZEROS 2
			#define NJAPOLES 2
			#define BUTWRTH_FILT_GAIN_ANG   2.761148367e+02

			static float xvJA[NJAZEROS+1], yvJA[NJAPOLES+1];

			xvJA[0] = xvJA[1];
			xvJA[1] = xvJA[2];
			xvJA[2] = inputVal / ( (float)BUTWRTH_FILT_GAIN_ANG);
			yvJA[0] = yvJA[1];
			yvJA[1] = yvJA[2];
			yvJA[2] = (xvJA[0] + xvJA[2]) + 2 * xvJA[1]
					 + ( -0.8371816513 * yvJA[0]) + (  1.8226949252 * yvJA[1]);
			return yvJA[2];

		}
	#endif //end 20Hz

	#ifdef SOFT_FILTER_JOINTVEL_IIR_20HZ
		/* Digital filter designed by mkfilter/mkshape/gencode   A.J. Fisher, http://www-users.cs.york.ac.uk/~fisher/cgi-bin/mkfscript
		   Command line: /www/usr/fisher/helpers/mkfilter -Bu -Lp -o 2 -a 2.0000000000e-02 0.0000000000e+00 -l */

		#define NJVZEROS 2
		#define NJVPOLES 2
		#define BUTWRTH_FILT_GAIN_VEL   2.761148367e+02

		float filterJointVelocityButterworth(float inputVal)
		{

			static float xvJV[NJVZEROS+1], yvJV[NJVPOLES+1];

			xvJV[0] = xvJV[1];
			xvJV[1] = xvJV[2];
			xvJV[2] = inputVal / BUTWRTH_FILT_GAIN_VEL;
			yvJV[0] = yvJV[1];
			yvJV[1] = yvJV[2];
			yvJV[2] = (xvJV[0] + xvJV[2]) + 2 * xvJV[1]
					 + ( -0.8371816513 * yvJV[0]) + (  1.8226949252 * yvJV[1]);
			return yvJV[2];

		}

		float filterJointAngleOutputButterworth(float inputVal)
		{

			static float xvJV[NJVZEROS+1], yvJV[NJVPOLES+1];

			xvJV[0] = xvJV[1];
			xvJV[1] = xvJV[2];
			xvJV[2] = inputVal / BUTWRTH_FILT_GAIN_VEL;
			yvJV[0] = yvJV[1];
			yvJV[1] = yvJV[2];
			yvJV[2] = (xvJV[0] + xvJV[2]) + 2 * xvJV[1]
					 + ( -0.8371816513 * yvJV[0]) + (  1.8226949252 * yvJV[1]);
			return yvJV[2];

		}

		float filterJointAngleLimitOutputButterworth(float inputVal)
		{

			static float xvJV[NJVZEROS+1], yvJV[NJVPOLES+1];

			xvJV[0] = xvJV[1];
			xvJV[1] = xvJV[2];
			xvJV[2] = inputVal / BUTWRTH_FILT_GAIN_VEL;
			yvJV[0] = yvJV[1];
			yvJV[1] = yvJV[2];
			yvJV[2] = (xvJV[0] + xvJV[2]) + 2 * xvJV[1]
					 + ( -0.8371816513 * yvJV[0]) + (  1.8226949252 * yvJV[1]);
			return yvJV[2];

		}

	#endif //end 20Hz

	#ifdef SOFT_FILTER_DERIVATIVE_IIR_15HZ
		/*  lpFilt = designfilt('lowpassiir','FilterOrder',2, ...
			 'PassbandFrequency',15,'PassbandRipple',0.1, ...
			 'SampleRate',1000);
			[b,a] = tf(lpFilt)
			[z p k ] = zpk(lpFilt)
			fvtool(lpFilt)
			M = idpoly(a,b,'NoiseVariance',0)
		   Cutoff = 15Hz, 2nd order, 0.1 ripple, Butterworth filter, Sampling at 1000Hz
		*/

		#define NTDZEROS 1
		#define NTDPOLES 1

		static float xvTD[NTDZEROS+1], yvTD[NTDPOLES+1];

		float filterTorqueDerivativeButterworth(float inputVal)
		{
			xvTD[0] = xvTD[1];
			xvTD[1] = inputVal;
			yvTD[0] = yvTD[1];
			yvTD[1] = 0.527890814211024*yvTD[0] + 0.236054592894488*xvTD[1] + 0.236054592894488*xvTD[0];
			return yvTD[1];
		}
	#endif //end 15Hz







#endif // end MATT_SOFT_FILT


