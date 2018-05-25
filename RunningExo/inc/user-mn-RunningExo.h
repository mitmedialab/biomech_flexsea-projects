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

#if defined INCLUDE_UPROJ_RUNNINGEXO || defined BOARD_TYPE_FLEXSEA_PLAN
#if defined BOARD_TYPE_FLEXSEA_MANAGE || defined BOARD_TYPE_FLEXSEA_PLAN

#ifndef INC_RUNNINGEXO_MN_H
#define INC_RUNNINGEXO_MN_H

//****************************************************************************
// Include(s)
//****************************************************************************
#include "main.h"
#include "runningExo-parameters.h"
#include "runningExo-structs.h"
//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

void init_runningExo(void);
void RunningExo_fsm_1(void);
void RunningExo_fsm_2(void);
void gaitStateTransition(void);

//****************************************************************************
// Accessor(s)
//****************************************************************************

#endif	//INC_RUNNINGEXO_MN_H

#endif //defined BOARD_TYPE_FLEXSEA_MANAGE || defined BOARD_TYPE_FLEXSEA_PLAN
#endif //defined INCLUDE_UPROJ_RUNNINGEXO || defined BOARD_TYPE_FLEXSEA_PLAN
