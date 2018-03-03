/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-user' User projects
	Copyright (C) 2016 Dephy, Inc. <http://dephy.com/>

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
	[Lead developper] Luke Mooney, lmooney at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] user-ex-MIT_2DoF_Ankle_v1: User code running on Manage
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-10-28 | jfduval | New release
	* 2016-11-16 | jfduval | Cleaned code, improved formatting
****************************************************************************/

#ifdef INCLUDE_UPROJ_MIT_DLEG
#ifdef BOARD_TYPE_FLEXSEA_MANAGE

//****************************************************************************
// Include(s)
//****************************************************************************

#include "user-mn.h"
#include <user-mn-MIT-DLeg-2dof.h>
#include "user-mn-ActPack.h"
#include <flexsea_comm.h>
#include <math.h>
#include "flexsea_sys_def.h"
#include "flexsea_system.h"
#include "flexsea_cmd_calibration.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

uint8_t mitDlegInfo[2] = {PORT_RS485_2, PORT_RS485_2};

//SAFETY FLAGS
static int8_t ANGLE_LIMIT_FLAG = 0;
static int8_t FORCE_LIMIT_FLAG = 0;

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Call this function once in main.c, just before the while()
void init_MIT_DLeg(void)
{

}

//MIT DLeg Finite State Machine.
//Call this function in one of the main while time slots.
void MIT_DLeg_fsm_1(void)
{
	#if(ACTIVE_PROJECT == PROJECT_MIT_DLEG)

    static uint32_t time = 0, state = -2;

    //Increment time (1 tick = 1ms)
    time++;

	//begin safety check
    if (safetyFailure()) {
    	//motor behavior changes based on failure mode
    	//bypasses the switch statement if return true

    	return;
    }
	//end safety check

	switch(state)
	{
		case -2:
			//Same power-on delay as FSM2:
			if(time >= AP_FSM2_POWER_ON_DELAY)
			{
				state = -1;
				time = 0;
			}

			break;

		case -1:
			if(findPoles()) {
				state = 0;
				time = 0;
			}

			break;

		case 0:
			//reserve for additional initialization
			state = 1;
			time = 0;
			break;
		case 1:
			//Pick one of those demos:
			//openSpeedFSM();
			twoPositionFSM();
			break;

        default:
			//Handle exceptions here
			break;
	}

	#endif	//ACTIVE_PROJECT == PROJECT_ANKLE_2DOF
}


//Second state machine for the DLeg project
void MIT_DLeg_fsm_2(void)
{
	#if(ACTIVE_PROJECT == PROJECT_MIT_DLEG)

		//Currently unused - we use ActPack's FSM2 for comm

	#endif	//ACTIVE_PROJECT == PROJECT_MIT_DLEG
}

//****************************************************************************
// Private Function(s)
//****************************************************************************
int8_t safetyFailure(void) {
	//check joint angles
//	if (*rigid1.ex.joint_ang <= )
	//check torque sensor

	//check motor and board temps
	return 1;
}

// Output joint angle in degrees, measured from joint zero
float getJointAngle(void)
{
	static uint32_t timer = 0;	//probably don't need this state business.
	static int8_t jointAngleState = -1;
	static int32_t jointAngleCnts = 0;
	static float jointAngle = 0;

	switch(jointAngleState)
		{
			case -1:
				//We give FSM2 some time to refresh values, first time -- likley not necessary?
				timer++;
				if(timer > 25)
				{
					jointAngleCnts = *(rigid1.ex.enc_ang);
					jointAngleState = 0;
				}
				break;
			case 0:
				jointAngleCnts = JOINT_ENC_DIR * ( JOINT_ZERO_OFFSET - *(rigid1.ex.enc_ang) ); //todo: check this directionality
				jointAngle = jointAngleCnts / JOINT_CPR * 360.0;

				//Check angle limits, raise flag for safety check
				if( jointAngle < JOINT_MIN || jointAngle > JOINT_MAX)
				{
					ANGLE_LIMIT_FLAG = 1;
				}
				break;
			default:
				//do nothing, if something went wrong.
				break;
		}
	return jointAngle;
}

// Output axial force on screw, Returns [Newtons]
float getAxialForce(void)
{
	static int8_t tareState = -1;
	static uint32_t timer = 0;
	static uint16_t strainReading = 0;
	static uint16_t tareOffset = 0;
	static int16_t strainMeasBias = 32768;	// centered around 2.5V, or 2^16/2 ticks.
	static float axialForce = 0;

	strainReading = (rigid1.ex.strain);

	//Check for over force reading, set flag, and exit.
	if (strainReading >= MAX_FORCE_TICKS || strainReading <= MIN_FORCE_TICKS)
	{
		FORCE_LIMIT_FLAG = 1;
		return MAX_FORCE;
	}

	switch(tareState)
	{
		case -1:
			//Tare the balance the first time this gets called.
			timer++;
			if(timer > 250)
			{
				strainReading = (rigid1.ex.strain);
				tareOffset = (rigid1.ex.strain);

				tareState = 0;
			}
			break;
		case 0:
			axialForce = ( strainReading - strainMeasBias - tareOffset ) * FORCE_PER_TICK;	// bias = 2.5V, or 32768 ticks
			break;
		default:
			//problem occurred
			break;
	}
	return axialForce;
}

// Linear Actuator Actual Moment Arm,
// input(theta [deg] )
// return moment arm projected length  [m]
float calc_linearActuatorMomentArm(float theta)
{
	static float theta_r = 0;
	theta_r = theta * M_PI / 180;
	static float A = 0, c = 0, r = 0, C_ang = 0;
	static float a = 0, b = 0, T = 0, F = 0;

	//Do calculations in m
    static float t = 0.047; 	// tibial offset
    static float t_k = 0.140; 	// offset from knee along tibia
    static float f = 0.039;  	// femur offset
    static float f_k = 0.018;	// offset from knee along femur

    a = sqrt( t*t + t_k*t_k);
    b = sqrt( f*f + f_k*f_k);

    T = atan(t/t_k);
    F = atan(f/f_k);

    C_ang = M_PI - theta_r - (T + F); 	// angle
    c = sqrt(a*a + b*b - 2 * a * b * cos( C_ang ) );  // length of actuator from pivot to output
    A = acos( ( a*a - ( c*c + b*b ) ) / ( -2 * b * c ) );
    r = b * sin(A);

    return r;
}

int8_t findPoles(void) {
	static uint32_t timer = 0;
	static int8_t polesState = 0;

	timer++;

	switch(polesState) {
		case 0:
			//Disable FSM2:
			disableActPackFSM2();
			if(timer > 10)
			{
				polesState = 1;
				timer = 0;
			}
			return 0;

			break;

		case 1:
			//Send Find Poles command:

			tx_cmd_calibration_mode_rw(TX_N_DEFAULT, CALIBRATION_FIND_POLES);
			packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_1, mitDlegInfo, SEND_TO_SLAVE);
			polesState = 2;
			timer = 0;
			return 0;

			break;

		case 2:
			//Wait 60s... (conservative)

			if(timer >= 60*SECONDS)
			{
				//Enable FSM2, position controller
				enableActPackFSM2();
				return 1;
			}
			return 0;

			break;
	}

	return 0;
}

void openSpeedFSM(void)
{
	static uint32_t timer = 0, deltaT = 0;
	static uint8_t fsm1State = 0;

	switch(fsm1State)
	{
		case 0:
			setControlMode(CTRL_OPEN);
			setMotorVoltage(0);
			fsm1State = 1;
			deltaT = 0;
			break;
		case 1:
			deltaT++;
			if(deltaT > 3000)
			{
				deltaT = 0;
				fsm1State = 2;
			}
			setMotorVoltage(0);
			break;
		case 2:
			deltaT++;
			if(deltaT > 3000)
			{
				deltaT = 0;
				fsm1State = 1;
			}
			setMotorVoltage(1000);
			break;
	}
}

void twoPositionFSM(void)
{
	static uint32_t timer = 0, deltaT = 0;
	static int8_t fsm1State = -1;
	static int32_t initPos = 0;

	switch(fsm1State)
	{
		case -1:
			//We give FSM2 some time to refresh values
			timer++;
			if(timer > 25)
			{
				initPos = *(rigid1.ex.enc_ang);
				fsm1State = 0;
			}
			break;
		case 0:
			setControlMode(CTRL_POSITION);
			setControlGains(20, 6, 0, 0);	//kp = 20, ki = 6
			setMotorPosition(initPos);
			fsm1State = 1;
			deltaT = 0;
			break;
		case 1:
			deltaT++;
			if(deltaT > 1000)
			{
				deltaT = 0;
				fsm1State = 2;
			}
			setMotorPosition(initPos + 10000);
			break;
		case 2:
			deltaT++;
			if(deltaT > 1000)
			{
				deltaT = 0;
				fsm1State = 1;
			}
			setMotorPosition(initPos);
			break;
	}
}

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
#endif //INCLUDE_UPROJ_MIT_DLEG
