#ifndef __TASKMACHINE_H__
#define __TASKMACHINE_H__

#include "flexsea_user_structs.h"
#include "kinematics_calculations.h"
#include "machine_learning_methods.h"
#include "user-mn-MIT-DLeg.h"
#include <stdio.h>







struct taskmachine_s
{
	
	int mode;
	int task;
	int state;

	int latestFootStaticSamples;
	int latestFootOffTime;

	int foundFirstZvupAfterFootOn;
	int taskPredicted;
	int inSwing;
	int achievedMinTorqueForStance;


	int resetTrigger;

};



void init_task_machine(void);
extern int task_machine_demux(struct rigid_s* rigid);

struct taskmachine_s tm;

enum Learning_States {
	INIT_TASK_MACHINE,
	INIT_LEARNING,
	RUN_TASK_MACHINE
};

#endif