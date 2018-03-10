/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'user/ActPack' Dephy's Actuator Package (ActPack)
	Copyright (C) 2017 Dephy, Inc. <http://dephy.com/>
*****************************************************************************
	[Lead developper] Jean-Francois Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] user-mn-RunningExo: Running Exoskeleton Project
****************************************************************************/

#ifdef INCLUDE_UPROJ_RUNNINGEXO
#ifdef BOARD_TYPE_FLEXSEA_MANAGE

#ifndef INC_RUNNINGEXO_MN_H
#define INC_RUNNINGEXO_MN_H

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"

//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

void init_runningExo(void);
void RunningExo_fsm_1(void);
void RunningExo_fsm_2(void);

//****************************************************************************
// Accessor(s)
//****************************************************************************


//****************************************************************************
// Constant Definition(s):
//****************************************************************************
//Controlling Strategy Options
//TODO:
#define OPEN_LOOP 1

//Angle Limit
//TODO:
#define ENC_POS_MAX 16384
#define ENC_POS_MIN 0

//Velocity Limit
//TODO:
#define ENC_VEL_MAX 1000
#define ENC_VEL_MIN -1000


//****************************************************************************
// Structure(s)
//****************************************************************************


//****************************************************************************
// Shared variable(s)
//****************************************************************************

#endif	//INC_RUNNINGEXO_MN_H

#endif //BOARD_TYPE_FLEXSEA_MANAGE
#endif //INCLUDE_UPROJ_RUNNINGEXO
