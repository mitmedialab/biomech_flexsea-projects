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
#include "stand_ankle_EMG.h"
#include "user-mn-MIT-EMG.h"
#include <user-mn-MIT-DLeg.h>
#include "state_variables.h"
#include "biom_state_machine.h"
#include "flexsea_user_structs.h"

//****************************************************************************
// Variable(s)
//****************************************************************************
// joint and movement params
float stand_max_PFangle = STAND_HOLD_ANGLE; // 15 degrees plantarflexion
float movementTime = MOVEMENT_TIME;
float comedownTime = COMEDOWN_TIME;
float userOffsetAngle = USER_OFFSET_ANGLE;

//internal state vars
float stand_state_init = 0; // current angle to drive toward while on tiptoes
float down_state_init = 0; // angle to start from when coming down
float standEMGThresh = -0.8; // plantarflexion threshold under which we activate tiptoe standing
int8_t isComingDown = 0;
uint16_t standTimer = 0;

//averaging window vars
int8_t avgWindow[EMG_STAND_WINDOW_SIZE];
static float intentAverage = 0;
static int16_t winIndex = -1;



//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************


//****************************************************************************
// Public Function(s)
//****************************************************************************

//stand_state is position of virtual joint (i.e. desired position of robot)

void updateStandJoint(GainParams* pgainsEMGStand)
{
	get_stand_EMG();
	pgainsEMGStand->thetaDes = interpret_stand_EMG();
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

void get_stand_EMG(void)
{
	float currentContribution = 0;

    // Read Seong's variables
	int8_t intent = emg_misc[0]; //0 = plantarflexion; 2 = dorsiflexion
	int16_t motionStrength = emg_misc[1]; //motion strength; 10000 = max dorsi; -10000 = max plantar

	// increment index and subtract out oldest value
	winIndex = (winIndex + 1) % EMG_STAND_WINDOW_SIZE;
	intentAverage -= avgWindow[winIndex]/EMG_STAND_WINDOW_SIZE;

	// range of normalized motionStrength == +-1. where negative is plantarflexion
	currentContribution = motionStrength/10000.;

	// add current contribution to average intent
	intentAverage += currentContribution/EMG_STAND_WINDOW_SIZE;

	// store current contribution to the avgWindow
	avgWindow[winIndex] = currentContribution;
}

//updates stand_state based on EMG activation
float interpret_stand_EMG(void)
{
	float stand_state = act1.jointAngleDegrees;
	stateMachine.current_state = STATE_EMG_STAND_ON_TOE; //set this by default. If only early stance, will reassign later

	// if desire to plantarflex
	if (intentAverage <= standEMGThresh && !isComingDown) {

		standTimer++;

		// and we haven't reached our target tiptoe position yet
		if (standTimer < movementTime) {
			stand_state = stand_state_init + (stand_max_PFangle - stand_state_init) \
					*(10*powf(standTimer/movementTime,3) - 15*powf(standTimer/movementTime,4) + 6*powf(standTimer/movementTime,5)); //min jerk trajectory (Hogan)

		// else we are already at our target position and we want to hold it
		} else {
			stand_state = stand_max_PFangle;
			standTimer = movementTime;
		}
	// if plantarflexion threshold not reached, do dorsiflexion back down
	} else if (isComingDown) {

		standTimer++;

		// if we're still coming down, drive toward the user preferred early stance equilibrium
		if (standTimer < comedownTime) {
			stand_state = down_state_init + (userOffsetAngle - down_state_init) \
					*(10*powf(standTimer/movementTime,3) - 15*powf(standTimer/movementTime,4) + 6*powf(standTimer/movementTime,5)); //min jerk trajectory (Hogan)
		} else {
			stand_state = userOffsetAngle;
			standTimer = comedownTime;
		}

		//if we came down and are close enough to user early stance preferred angle
		if (abs(stand_state - userOffsetAngle) < 3) {
			reset_EMG_stand(act1.jointAngleDegrees);
			stand_state = userOffsetAngle;
		}

	//trigger isComingDown and follow time-based trajectory back to early stance equilibrium
	} else if (intentAverage > standEMGThresh && abs(stand_state - stand_state_init) > 3 && standTimer > 0) {
		isComingDown = 1;
		standTimer = 0;
		down_state_init = act1.jointAngleDegrees; //angle when starting descent
		stand_state = act1.jointAngleDegrees;

	} else {
		// go to early stance (perhaps combine EMG and early stance together)
		stateMachine.current_state = STATE_EARLY_STANCE;
	}

	return stand_state;
}
void reset_EMG_stand(float resetAngle) {
	winIndex = -1;
	standTimer = 0;
	intentAverage = 0;
	isComingDown = 0;

	for (uint16_t i = 0; i < sizeof(avgWindow)/sizeof(avgWindow[0]); i++) {
		avgWindow[i] = 0;
	}

	stand_state_init = resetAngle;
}

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
