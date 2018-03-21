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
	[Lead developer] Luke Mooney, lmooney at dephy dot com.
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

#ifdef BOARD_TYPE_FLEXSEA_MANAGE

//****************************************************************************
// Include(s)
//****************************************************************************

#include <flexsea_comm.h>
#include <math.h>
#include "flexsea_sys_def.h"
#include "flexsea_system.h"
#include "MIT_Ankle_EMG.h"
#include "user-mn-MIT-EMG.h"
#include "user-mn-MIT-DLeg-2dof.h"
#include "state_variables.h"
#include "flexsea_user_structs.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

int32_t EMGavgs[2] = {0, 0}; // Initialize the EMG signals to 0;
float PFDF_state[3] = {0, 0, 0};

float stepsize = .001; //dt, if running at 1kHz

//HANDLE EMG FROM SEONG
float gainLG = GAIN_LG;
float gainTA = GAIN_TA;
float emgInMax = EMG_IN_MAX;

//Constants for tuning the controller
float pfTorqueGain = PF_TORQUE_GAIN;
float dfTorqueGain = DF_TORQUE_GAIN;
float pfdfStiffGain = PFDF_STIFF_GAIN;
float dpOnThresh = DP_ON_THRESH;
float cocontractThresh = COCON_THRESH;

int k_lim = 100; //virtual spring constant opposing motion at virtual joint limit.
int b_lim = 1; //virtual damping constant opposing motion at virtual joint limit.

//VIRTUAL DYNAMIC JOINT PARAMS
float virtualK = VIRTUAL_K;
float virtualB = VIRTUAL_B;
float virtualJ = VIRTUAL_J;

// joint limits
float max_DFangle = -20; // These need to match the robot. They are hard limits on desired joint angle.
float max_PFangle = 40;
float equilibriumAngle = 30; // 30 degrees plantarflexion

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************


//****************************************************************************
// Public Function(s)
//****************************************************************************

//PFDF_state[0] is position of virtual joint (i.e. desired position of robot)
//PFDF_state[1] is velocity of virtual joint (not used in current controller, but must be stored for numerical integration)
//PFDF_state[2] is desired stiffness, derived from average muscle activation.

//This is where we get EMG, and then interpret to calculate desired position, velocity, and stiffness ("PFDF_state" and "INEV_state" global vars)
//This function is called at 1kHz when in the proper state

void updateVirtualJoint(GainParams* pgains)
{
	get_EMG();
	interpret_EMG(virtualK, virtualB, virtualJ);
	pgains->thetaDes = PFDF_state[0];
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

void get_EMG(void) //Read the EMG signal, rectify, and integrate. Output an integrated signal.
{
    // Read Seong's variables toDo:CHANGE TO ACTUAL
	uint16_t EMGin_LG = emg_data[0]; //SEONGS BOARD LG_VAR gastroc, 0-10000
	uint16_t EMGin_TA = emg_data[1]; //SEONGS BOARD TA_VAR tibialis anterior, 0-10000
	
	//pack for Plan
	rigid1.mn.genVar[0] = EMGin_LG;
	rigid1.mn.genVar[1] = EMGin_TA;

	//Apply gains
	EMGavgs[0] = EMGin_LG * gainLG;
	EMGavgs[1] = EMGin_TA * gainTA;
}

//updates PFDF_state[] based on EMG activation
void interpret_EMG (float k, float b, float J)
{
	float LGact = 0;
	float TAact = 0;
	float TALG_diff = 0;
	float Torque_PFDF = 0;
	float activationThresh = dpOnThresh * emgInMax;

	// Calculate LG activation
	if (EMGavgs[0] > activationThresh)
	{
		LGact = (EMGavgs[0] - activationThresh) / (emgInMax - activationThresh); //scaled activation 0-1
	}

	// Calculate TA activation

	if (EMGavgs[1] > activationThresh)
	{
		TAact = (EMGavgs[1] - activationThresh) / (emgInMax - activationThresh);
	}

	// PF/DF Calc
	TALG_diff = LGact - TAact;
	Torque_PFDF = TALG_diff > 0 ? TALG_diff*pfTorqueGain : TALG_diff*dfTorqueGain;

	//pack for Plan
	rigid1.mn.genVar[2] = TALG_diff*1000;

	// JOINT MODEL
	//first, we apply extra damping in co-contraction
	if ((TAact > cocontractThresh) && (LGact > cocontractThresh))
	{
		float stiffness = (TAact + LGact)/2.0;
		b = b * (erff(10.0*(stiffness-0.15-cocontractThresh))+1)*20;
	}

	//pack for Plan
	rigid1.mn.genVar[3] = b*1000;

	// scaling restoring force based on loose ankle at 35 degrees plantarflexion (Tony's idea. Test first)
	if (PFDF_state[0] > equilibriumAngle) {
		k = k*(PFDF_state[0] - equilibriumAngle)/(max_PFangle - equilibriumAngle);
	} else {
		k = -k*(PFDF_state[0] - equilibriumAngle)/(max_DFangle - equilibriumAngle);
	}

	//pack for Plan
	rigid1.mn.genVar[4] = k*1000;

	// forward dynamics equation
	float dtheta = PFDF_state[1];
	float domega = -k/J * PFDF_state[0] - b/J * PFDF_state[1] + 1/J * Torque_PFDF; //-restoring stiffness - damping + torque

	// virtual joint physical constraints
	if (PFDF_state[0] < max_DFangle)
	{
		domega = domega - k_lim/J * (PFDF_state[0] - max_DFangle) - b_lim/J * PFDF_state[1]; // Hittin a wall.
	}
	else if (PFDF_state[0] > max_PFangle)
	{
		domega = domega - k_lim/J * (PFDF_state[0] - max_PFangle) - b_lim/J * PFDF_state[1]; // Hittin the other wall.
	}

	//pack for Plan
	rigid1.mn.genVar[5] = dtheta*1000;
	rigid1.mn.genVar[6] = domega*1000;

	// Integration station.
	RK4_SIMPLE(dtheta, domega, PFDF_state);
	PFDF_state[2] = (TAact + LGact)/2.0 * pfdfStiffGain; // stiffness comes from the common signal

	//pack for Plan
	rigid1.mn.genVar[7] = PFDF_state[0]*1000;
//	rigid1.mn.genVar[8] = PFDF_state[1]*1000;
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
