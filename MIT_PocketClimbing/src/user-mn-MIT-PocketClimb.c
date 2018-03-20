/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-projects' User projects
	Copyright (C) 2018 Dephy, Inc. <http://dephy.com/>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*****************************************************************************
	[Lead developer] Jean-Francois Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] user--mn-MIT-PocketClim: Demo state machine for Pocket 2x DC
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2018-03-02 | jfduval | New release
****************************************************************************/
#define PID_CONTROLLER
#define INITIALIZE_ENCODERS
#define EMG_INPUT
#ifdef INCLUDE_UPROJ_MIT_POCKET_CLIMB
#ifdef BOARD_TYPE_FLEXSEA_MANAGE
#ifdef PID_CONTROLLER
#ifdef INCLUDE_UPROJ_MIT_POCKET_CLIMB
#ifdef BOARD_TYPE_FLEXSEA_MANAGE

//****************************************************************************
// Include(s)
//****************************************************************************

#include "user-mn.h"
#include "user-mn-MIT-PocketClimb.h"
#include "user-mn-ActPack.h"
#include <flexsea_comm.h>
#include <math.h>
#include "flexsea_sys_def.h"
#include "flexsea_system.h"
#include "flexsea_cmd_calibration.h"

#include "user-mn-MIT-EMG.h"
//****************************************************************************
// Variable(s)
//****************************************************************************


uint8_t mitClimbInfo[2] = {PORT_RS485_2, PORT_RS485_2};
float error0, error_sum0, error_diff0, error_past0;
float error1, error_sum1, error_diff1, error_past1;
float initForce0, initForce1;
float p_gain0, p_gain1;
float d_gain0, d_gain1;
int m0_pwm = 0, m1_pwm = 0;
int emg_0, emg_1;
int emg_state;
signed  long motor_pwm0, motor_pwm1;
signed long  m0_target, m1_target;
int16_t pwm[2] = {0,0};
float axialForce[2]={0,0};
int encoderzero_0, encoderzero_1;
int pos_encoder0_past, pos_encoder1_past;
int encoderstatus = 0;
//static int8_t encoderState = -1;
int m0init_status = 0;
int m1init_status = 0;
int dampingflag =0;
int dampingflag_previous =0;
int loadedflag =0;
int init1 =0, init0 =0;
// emg variables

float stepsize = .001; //dt, if running at 1kHz
float PFDF_state [3] = {0,0,0};
float INEV_state [3] = {0,0,0};
float TAact = 0, LGact = 0, PLact = 0, TPact = 0;
float stiffness = 1;


//int LGset =1000,TPset = 1000, TAset = 1000, PLset = 0001;

uint16_t TAact_raw, LGact_raw, PLact_raw, TPact_raw;
uint16_t TAact_hold, LGact_hold, PLact_hold, TPact_hold;


//emg board interface
extern int16_t emg_data[8];
extern uint8_t emg_packet;
extern volatile uint8_t emg_on_flag; //this flag decide whether use EMG or not
extern uint16_t emg_timestamp;


//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************

void getAxialForce(void);
void pidControl(signed long pos_target0, signed long pos_target1, int32_t pos_encoder0, int32_t pos_encoder1, float stiffness);
static void ang_to_enc(float angle_0, float angle_1);
static void motorInit(void);
//void encoderInit(void);
void interpretEMG(void);
int getEnc(int encoder);


//****************************************************************************
// Public Function(s)
//****************************************************************************

//Call this function once in main.c, just before the while()
void init_MIT_PocketClimb(void)
{

}

//MIT DLeg Finite State Machine.
//Call this function in one of the main while time slots.
void MIT_PocketClimb_fsm_1(void)
{
	#if(ACTIVE_PROJECT == PROJECT_POCKET_2XDC)
	static uint32_t time = 0, state = -2;
	static uint32_t load_time = 0;
	time++;

	switch(state) {

	case -2:
		//Same power-on delay as FSM2 (+ some time to refresh values)

		#ifdef INITIALIZE_ENCODERS
		if (time >= (2000)) {
			motorInit();
			getAxialForce();
		}

		if(time >= (5000)) {
			initForce0 =axialForce[0];
			initForce1= axialForce[1];
			state = -1;
			//rigid1.mn.genVar[1] = 1;
			time = 0;
		}
		break;

		case -1:
	//	rigid1.mn.genVar[2] = time;
		getAxialForce();

		if (m0init_status == 0 ){
			m0_pwm=PWM_MOTORINIT;
			setMotorVoltage(m0_pwm, MOTOR_0);
			if (axialForce[0]  < (initForce0 - THRESH_MOTORINIT)){
			m0init_status = 1;
			}
		}

		else if (m0init_status==1) {
			m0_pwm=0;
			setMotorVoltage(m0_pwm, MOTOR_0);
		}

		if (m1init_status == 0) {
			m1_pwm=PWM_MOTORINIT;
			setMotorVoltage(m1_pwm, MOTOR_1);
			if (axialForce[1]  <  (initForce1 - THRESH_MOTORINIT)) {
				m1init_status = 1;
			}
		}

		else if (m1init_status == 1) {
			m1_pwm=0;
			setMotorVoltage(m1_pwm, MOTOR_1);
		}

		if (m0init_status == 1 && m1init_status == 1) {
			m0_pwm=0;
			setMotorVoltage(m0_pwm, MOTOR_0);
			m1_pwm=0;
			setMotorVoltage(m1_pwm, MOTOR_1);
			encoderzero_0 = *pocket1.ex[0].enc_ang;
			encoderzero_1 = *pocket1.ex[1].enc_ang;
			state =0;
		}
		break;

		case 0:

		if (getEnc(0) > ENCODERMIDPOINT)
			m0_pwm=-200;
		else {
			m0_pwm=0;
			init0=1;
		}

		if (getEnc(1) > ENCODERMIDPOINT)
			m1_pwm=-200;
		else {
			m1_pwm=0;
			init1=1;
		}

		setMotorVoltage(m0_pwm, MOTOR_0);
		setMotorVoltage(m1_pwm, MOTOR_1);

		if (init1 == 1 && init0 ==1) {
			state = 1;
		}

		#endif // INITIALIZE_ENCODERS

		#ifndef INITIALIZE_ENCODERS
		if (time >= (2000)) {
			motorInit();
			getAxialForce();
			encoderzero_0 = *pocket1.ex[0].enc_ang;
			encoderzero_1 = *pocket1.ex[1].enc_ang;
			state = 1;
			}

		#endif
		break;

		case 1:

		#ifdef EMG_INPUT

			ang_to_enc((float)PFDF_state[0], (float)INEV_state[0]);
			stiffness = (PFDF_state[2]+INEV_state[2])/2;
			getAxialForce();
			load_time++;
			dampingflag_previous = dampingflag;
			if(abs(axialForce[0])>FORCE_THRESHOLD || abs(axialForce[1])>FORCE_THRESHOLD) {
				m0_pwm=0;
				m1_pwm=0;
				dampingflag = 1;
			//	load_time = 0;
			}
			else {
				pidControl(m0_target, m1_target, getEnc(0), getEnc(1), stiffness);
				m0_pwm = motor_pwm0;
				m1_pwm = motor_pwm1;
				//dampingflag = 0;
			}

			if(load_time > 100) {
				dampingflag = 0;
			}

			setMotorVoltage(m0_pwm, MOTOR_0);
			setMotorVoltage(m1_pwm, MOTOR_1);


			rigid1.mn.genVar[0] = axialForce[0];
			rigid1.mn.genVar[1] = axialForce[1];

			rigid1.mn.genVar[2] = PFDF_state[0];
			rigid1.mn.genVar[3] = INEV_state[0];

			rigid1.mn.genVar[4] = m0_pwm;
			rigid1.mn.genVar[5] = m1_pwm;

			rigid1.mn.genVar[6] = getEnc(0);
			rigid1.mn.genVar[7] = getEnc(1);

			rigid1.mn.genVar[8] = m0_target;
			rigid1.mn.genVar[9] = m1_target;


		#endif // EMG_INPUT



#ifndef EMG_INPUT
			testfn();
			if(time >= (DEBUG_TIMER))
			{
				emg_state++;
				if (emg_state>1) {
					emg_state=0;
				}
				time = 0;
			}

#endif //if not EMG_INPUT
			break;


			default:
			//Handle exceptions here
			break;
	}


	#endif	//ACTIVE_PROJECT == PROJECT_POCKET_2XDC
}


//Second state machine for the DLeg project
void MIT_PocketClimb_fsm_2(void)
{
	#if(ACTIVE_PROJECT == PROJECT_POCKET_2XDC)

		// EMG board communication
	// TAact, LG act coming in from Seong's board
/*
 	extern int16_t emg_data[8];
	extern uint8_t emg_packet;
	extern volatile uint8_t emg_on_flag; //this flag decide whether use EMG or not
	extern uint16_t emg_timestamp;
	TAact_raw = emg_data[3];
	LGact_raw = emg_data[6];
	PLact_raw = emg_data[5];
	TPact_raw = emg_data[0];
*/
	// If not using emg board

	TAact_raw = user_data_1.w[0];
	LGact_raw = user_data_1.w[1];
	PLact_raw = user_data_1.w[2];
	TPact_raw = user_data_1.w[3];

	if (dampingflag == 1 && dampingflag_previous ==1) {
		TAact_hold = TAact_raw;
		LGact_hold = LGact_raw;
		PLact_hold = PLact_raw;
		TPact_hold = TPact_raw;
	}

	if (dampingflag == 1) {
		TAact_raw = TAact_hold;
		LGact_raw = LGact_hold;
		PLact_raw = PLact_hold;
		TPact_raw = TPact_hold;
	}
	//interpretEMG();

	#endif	//ACTIVE_PROJECT == PROJECT_POCKET_2XDC
}


void MIT_PocketClimb_fsm_3(void)
{
	#if(ACTIVE_PROJECT == PROJECT_POCKET_2XDC)
	interpretEMG();

	#endif	//ACTIVE_PROJECT == PROJECT_POCKET_2XDC
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

int getEnc(int enc)
{
	int encoder;
	if (enc==0) {
		encoder = *pocket1.ex[0].enc_ang - encoderzero_0;

	}

	else {
		encoder = *pocket1.ex[1].enc_ang - encoderzero_1;
	}
	return encoder;
}

void getAxialForce(void)
{

	static int8_t tareState = -1;
	static uint32_t timer = 0;
	static uint16_t strainReading_0 = 0;
	static uint16_t strainReading_1 = 0;
	static uint16_t tareOffset_0 = 0;
	static uint16_t tareOffset_1 = 0;
	strainReading_0 = pocket1.ex[MOTOR_0].strain;
	strainReading_1 = pocket1.ex[MOTOR_1].strain;

		switch(tareState)
	{
		case -1:
			//Tare the balance the first time this gets called.
			timer++;
			if(timer > 250)
			{
				strainReading_0 = pocket1.ex[MOTOR_0].strain;
				strainReading_1 = pocket1.ex[MOTOR_1].strain;
				tareOffset_0 = strainReading_0;
				tareOffset_1 = strainReading_1;
				tareState = 0;
			}
			break;
		case 0:
			axialForce[0] =  FORCE_DIR * ( strainReading_0 - tareOffset_0 ) * FORCE_PER_TICK;	// Looks correct with simple weight, need to test with a scale
			axialForce[1] =  FORCE_DIR * ( strainReading_1 - tareOffset_1 ) * FORCE_PER_TICK;
			break;

		default:
			//problem occurred
			break;
	}
}

static void motorInit()
{

	m0_pwm=0;
	m1_pwm=0;
	setControlMode(CTRL_OPEN, MOTOR_0);
	setMotorVoltage(m0_pwm, MOTOR_0);
	setControlMode(CTRL_OPEN, MOTOR_1);
	setMotorVoltage(m1_pwm, MOTOR_1);
	//getAxialForce();
}


void pidControl(signed long pos_target0, signed long pos_target1, int32_t pos_encoder0, int32_t pos_encoder1, float stiffness)
{
	int32_t encodermin = -7500;
	int32_t encodermax = -1000;

	float encodermin0= encodermin + (abs(pos_encoder0 - pos_encoder1)*(float)RANGELIMIT);
	float encodermin1= encodermin + (abs(pos_encoder0 - pos_encoder1)*(float)RANGELIMIT);
	float encodermax0= encodermax - (abs(pos_encoder0 - pos_encoder1)*(float)RANGELIMIT);
	float encodermax1= encodermax - (abs(pos_encoder0 - pos_encoder1)*(float)RANGELIMIT);

	float pid_spring = 1;
	int32_t springerror0=0;
	int32_t springerror1=0;

	signed long error0 = pos_target0 - pos_encoder0;
	signed long error1 = pos_target1 - pos_encoder1;
	error_diff0 = error0 - error_past0;
	error_past0 = error_diff0;
	error_diff1 = error1 - error_past1;
	error_past1 = error_diff1;

	if (pos_encoder0 < encodermin0) {
		springerror0 = (encodermin0 - pos_encoder0)* pid_spring;
	}
	else if (pos_encoder0 > encodermax0) {
			springerror0 = (encodermax0 - pos_encoder0)* pid_spring;
	}
			if (pos_encoder1 < encodermin1) {
	springerror1 = (encodermin1 - pos_encoder1)* pid_spring;
	}
	else if (pos_encoder1 > encodermax1) {
				springerror1 = (encodermax1 - pos_encoder1)* pid_spring;
			}

	p_gain0 = PID_KP * (error0) + stiffness + springerror0;  // * stiffness
	d_gain0 = PID_KD*(error_diff0);
	d_gain1 = PID_KD*(error_diff1);
	motor_pwm0 = (signed long)p_gain0 + (signed long)d_gain0;

	if (motor_pwm0 > MAX_PWM)
	motor_pwm0 = MAX_PWM;
	else if (motor_pwm0 < -MAX_PWM)
	motor_pwm0 = -MAX_PWM;
	else if ( (motor_pwm0 > -MIN_PWM) && (motor_pwm0 < MIN_PWM))
	motor_pwm0 = 0;

	p_gain1 = PID_KP * (error1) + stiffness + springerror1;  // * stiffness
	motor_pwm1 = (signed long)p_gain1 + (signed long)d_gain1;

	if (motor_pwm1 > MAX_PWM)
	motor_pwm1 = MAX_PWM;
	else if (motor_pwm1 < -MAX_PWM)
	motor_pwm1 = -MAX_PWM;
	else if ( (motor_pwm1 > -MIN_PWM) && (motor_pwm1 < MIN_PWM))
	motor_pwm1 = 0;
}


static void ang_to_enc(float angle_0, float angle_1)
{
	// angle 0 PF/DF, PF +, DF -
	// angle 1 IN/EV, IN +, EV -
	// Variables for angle calculation

	float  r = .046776;
	float d = .035 / 2;
	float depth_a = .040;
	float width_a = .045;
	float height_a = .1587;

	float pf = (angle_0+5);
	float ev = angle_1 * 0.0174533;
	pf = 0.0174533 * ((pf) - 30); //adjust for foot angle and convert to rad

	float L_act_right = sqrt(pow(depth_a - (r * cos(pf)),2) + pow(height_a - ((r * sin(pf) * cos((ev))) + sin((ev)) * d),2) + pow((width_a / 2) - ((r * sin((pf)) * sin((ev))) + cos((ev)) * d),2));
	float L_act_left = sqrt(pow(depth_a - (r * cos(pf)),2) + pow(height_a - ((r * sin(pf) * cos((ev))) - sin((ev)) * d),2) + pow((-width_a / 2) - ((r * sin((pf)) * sin((ev))) - cos((ev)) * d),2));

	float enc_desired_right = (.1646 - L_act_right) * (512 / .0015875);
	float enc_desired_left = (.1646 - L_act_left) * (512 / .0015875);

	m0_target = (int)enc_desired_right;
	m1_target = (int)enc_desired_left;

}


// emg processing

void interpretEMG(void)
{
	float k = VIRTUALK;
	float b = VIRTUALB;
	float J = VIRTUALJ;

	if (TAact_raw < DPONTHRESH)
	{
	TAact_raw = 0;
	}
	if (LGact_raw < DPONTHRESH)
	{
	LGact_raw = 0;
	}
	if (PLact_raw < IEONTHRESH)
	{
	PLact_raw = 0;
	}
	if (TPact_raw < IEONTHRESH)
	{
	TPact_raw = 0;
	}

	TAact=((float)TAGAIN*TAact_raw-DPONTHRESH)/(10000-DPONTHRESH);
	LGact=((float)LGGAIN*LGact_raw-DPONTHRESH)/(10000-DPONTHRESH);
	PLact=((float)PLGAIN*PLact_raw-IEONTHRESH)/(10000-IEONTHRESH);
	TPact=((float)TPGAIN*TPact_raw-IEONTHRESH)/(10000-IEONTHRESH);

	float P_PFDF;
	if ((TAact - LGact) < 0)
	{
		P_PFDF = -1*(TAact - LGact) * PFTORQUEGAIN; // if TA > LG --> dorsiflex (negative)
	}
	else
	{
		P_PFDF = -1*(TAact - LGact) * DFTORQUEGAIN; // if TA < LG --> plantarflex (positive)
	}

	// JOINT MODEL
	//first, we apply extra damping in co-contraction
	if ((TAact > COCONTHRESH) && (LGact > COCONTHRESH))
	{
		float stiffness = (TAact + LGact)/2.0;
		b = b * (erff(10.0*(stiffness-.15-COCONTHRESH))+1)*20;
	}

	float domega = -k/J * PFDF_state[0] - b/J * PFDF_state[1] + 1/J * P_PFDF;

	RK4_SIMPLE(PFDF_state[1], domega, PFDF_state);
	PFDF_state[2] = (TAact + LGact)/2.0 * PFDFSTIFFGAIN;

	// IN - EV Calcs
	float P_INEV;
	if ((PLact - TPact) < 0)
	{
		P_INEV = -1*(PLact - TPact) * INTORQUEGAIN; // inversion positive
	}
	else
	{
		P_INEV = -1*(PLact - TPact) * EVTORQUEGAIN; // eversion negative
	}

	float domega_inev = -k/J * INEV_state[0] - b/J * INEV_state[1] + 1/J * P_INEV;

	RK4_SIMPLE(INEV_state[1], domega_inev, INEV_state);
	INEV_state[2] = (PLact + TPact)/2.0 * INEVSTIFFGAIN;
}


void RK4_SIMPLE(float d1_dt,float d2_dt, float* cur_state)
{
	float next_state[2];
	float F1 = d1_dt;
	float F2 = d1_dt + .5 * stepsize * F1;
	float F3 = d1_dt + .5 * stepsize * F2;
	float F4 = d1_dt + stepsize * F3;
	next_state[0] = cur_state[0] + (stepsize/6) * (F1 + 2*F2 + 2*F3 + F4);
	F1 = d2_dt;
	F2 = d2_dt + .5 * stepsize * F1;
	F3 = d2_dt + .5 * stepsize * F2;
	F4 = d2_dt + stepsize * F3;
	next_state[1] = cur_state[1] + (stepsize/6) * (F1 + 2*F2 + 2*F3 + F4);
	cur_state[0] = next_state[0];
	cur_state[1] = next_state[1];
}


#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
#endif //INCLUDE_UPROJ_MIT_POCKET_CLIMB

#endif //PID_CONTROLLER
#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
#endif //INCLUDE_UPROJ_MIT_POCKET_CLIMB


