
#ifdef __cplusplus
extern "C" {
#endif

//****************************************************************************
// Include(s)
//****************************************************************************
#include "walking_knee_ankle_state_machine.h"
#include <user-mn-MIT-DLeg.h>
//#include "user-mn-MIT-EMG.h"
#include "spline_functions.h"
#include "free_ankle_EMG.h"
#include "torque_replay.h"

//****************************************************************************
// Definition(s):
//****************************************************************************
WalkingStateMachine kneeAnkleStateMachine;
WalkParams *ankleWalkParams, kneeWalkParams;
CubicSpline cubicSpline;
TorqueRep torqueRep;

//NOTE: All of the damping values have been reduced by 1/10 due to controller
// Gain Parameters are modified to match our joint angle convention (RHR for right ankle, wearer's perspective). Positive Plantaflexion
//GainParams ankleGainsEst = {2.5, 0.0, 0.18, -5.0};	// may want to increase this damping, at least.
//GainParams ankleGainsMst = {2.5, 0.0, 0.18, 0.0};	// may want to increase this damping, at least.
//GainParams ankleGainsLst = {4.0, 0.0, 0.18, 10.0};
//GainParams ankleGainsEsw = {1.5, 0.0, 0.18, -5.0};
//GainParams ankleGainsLsw = {1.5, 0.0, 0.18, -5.0};

GainParams ankleGainsEMG = {0.0, 0.0, 0.0, 0.0};

float splineTime = 100.0;

//Knee, Positive Knee Flexion
GainParams kneeGainsEst = {2.5, 0.0, 0.15, 10.0};
GainParams kneeGainsMst = {2.5, 0.0, 0.15, 10.0};	// {2.5, 0.0, 0.1, 10.0};
GainParams kneeGainsLst = {1.0, 0.0, 0.15, 30.0};	// {2.5, 0.0, 0.1, 15.0};
GainParams kneeGainsEsw = {2.5, 0.0, 0.15, 30.0}; // GainParams kneeGainsEsw = {1.5, 0.0, 0.1, 50.0};
GainParams kneeGainsLsw = {2.5, 0.0, 0.15, 10.0};




#ifdef BOARD_TYPE_FLEXSEA_MANAGE

//****************************************************************************
// Functions:
//****************************************************************************

void setSimpleAnkleFlatGroundFSM(Act_s *actx, WalkParams *ankleWalkParamx) {
	static int8_t isTransitioning = 0;
	static uint32_t timeInState = 0;
	static int8_t passedStanceThresh = 0;

    kneeAnkleStateMachine.onEntrySmState = kneeAnkleStateMachine.currentState; // save the state on entry, assigned to last_currentState on exit
	actx->tauDes = 0.0;


	// Check for state change, then set isTransitioning flag
	if (kneeAnkleStateMachine.currentState == kneeAnkleStateMachine.lastSmState) {
		isTransitioning = 0;
		timeInState++;
	} else {
		// State transition, reset timers and set entry flag
		timeInState = 0;
		isTransitioning = 1;
	}


	    switch (kneeAnkleStateMachine.currentState) {

	        case STATE_IDLE: //-1
	        {
	        	//error handling here (should never be in STATE_IDLE by the time you get here)
	            break;
	        }
	        case STATE_INIT: //-2
	        {
	        	kneeAnkleStateMachine.currentState = STATE_EARLY_SWING;	// enter into early stance, this has stability. good idea for torque replay? might be better late swing

	            break;
	        }
	        case STATE_EARLY_STANCE: //0 check impedance mode in here - only stance state for torque replay (goes directly to early swing)
	        {

				if (isTransitioning) {

					ankleWalkParamx->scaleFactor = 1.0;

					ankleWalkParamx->ankleGainsEst.k1 = ankleWalkParamx->earlyStanceK0;
					ankleWalkParamx->ankleGainsEst.thetaDes = actx->jointAngleDegrees;	//note, this might be a key thingy

					passedStanceThresh = 0;
					ankleWalkParamx->timerInStance = 0;
					ankleWalkParamx->timerInSwingLast = ankleWalkParamx->timerInSwing;

				}

				ankleWalkParamx->timerInStance++;

				updateAnkleVirtualHardstopTorque(actx, ankleWalkParamx);
				actx->tauDes = ankleWalkParamx->virtualHardstopTq + getImpedanceTorqueParams(actx, &ankleWalkParamx->ankleGainsEst);

				//Early Stance transition vectors
				if (abs(actx->jointTorque) > 20.0){
					passedStanceThresh = 1;
				}

				if (actx->jointTorque > ankleWalkParamx->lspEngagementTorque) {
					kneeAnkleStateMachine.currentState = STATE_LATE_STANCE_POWER;      //Transition occurs even if the early swing motion is not finished
				}

				break;
	        }


	        case STATE_MID_STANCE: //1
	        { //skipping this for now.
//				if (isTransitioning) {
//
//				}
//
//				#ifdef IS_ANKLE
//
//
//	    			// Stance transition vectors, only go into next state. This is a stable place to be.
//	        		// Transition occurs based on reaching torque threshold. (future: update this threshold based on speed)
//
//					updateAnkleVirtualHardstopTorque(actx, &ankleWalkParamx);	// Bring in
//	//				actx->tauDes = ankleWalkParamx->virtualHardstopTq + getImpedanceTorque(actx, ankleGainsMst.k1, ankleGainsMst.b, ankleGainsMst.thetaDes);
//					actx->tauDes = ankleWalkParamx->virtualHardstopTq + getImpedanceTorqueParams(actx, &ankleWalkParamx->ankleGainsMst);
//
//	    			// Stance transition vectors, only go into next state. This is a stable place to be.
//
//	    			if (actx->jointTorque > ankleWalkParamx->lspEngagementTorque) {
//	    				kneeAnkleStateMachine.currentState = STATE_LATE_STANCE_POWER;
//	    			}
//
//				#elif defined(IS_KNEE)
//	        		actx->tauDes = getImpedanceTorque(actx, kneeGainsMst.k1, kneeGainsMst.b, kneeGainsMst.thetaDes);
//				#endif
//
				ankleWalkParamx->timerInStance++;

	        	break;
	        }
	        case STATE_LATE_STANCE_POWER: //2
	        {
				if (isTransitioning) {
					ankleWalkParamx->samplesInLSP = 0.0;
					ankleWalkParamx->lspEntryTq = actx->jointTorque;
				}

				ankleWalkParamx->timerInStance++;


				// This is the scaling factor for ramping into powered pushoff
				if (ankleWalkParamx->samplesInLSP < ankleWalkParamx->lstPGDelTics){
					ankleWalkParamx->samplesInLSP++;
				}

				updateAnkleVirtualHardstopTorque(actx, ankleWalkParamx);

					//Linear ramp to push off, pickup where hardstop leftoff, use stiffness ankleGainsLst to get us to target point.
					actx->tauDes = ankleWalkParamx->virtualHardstopTq + (ankleWalkParamx->samplesInLSP/ankleWalkParamx->lstPGDelTics) * getImpedanceTorqueParams(actx, &ankleWalkParamx->ankleGainsLst);  // drops off after zero when hardstop goes away

					//Late Stance Power transition vectors
					if ( (fabs(actx->jointTorque) < ANKLE_UNLOADED_TORQUE_THRESH) && (timeInState > LST_TO_ESW_DELAY )) //&& (actx->jointAngleDegrees >=  ankleGainsLst.thetaDes -1.0) ) {	// not sure we need the timeInState? what's the point? just maker sure it's kicking?
					{
						kneeAnkleStateMachine.currentState = STATE_EARLY_SWING;
						ankleWalkParamx->timerInStanceLast = ankleWalkParamx->timerInStance;

					}

	            break;
	        }

	        case STATE_EARLY_SWING: //3
	        {
				//Put anything you want to run ONCE during state entry.
				if (isTransitioning)
				{
					ankleWalkParamx->virtualHardstopTq = 0.0;
					ankleWalkParamx->timerInSwing = 0;

				}
				ankleWalkParamx->timerInSwing++;

				actx->tauDes = getImpedanceTorqueParams(actx, &ankleWalkParamx->ankleGainsEsw);

				if(timeInState >= ESW_TO_LSW_DELAY)
				{
					if(fabs(actx->tauMeas) >= fabs(GENTLE_HEELSTRIKE_TORQUE_THRESH) )
					{
						kneeAnkleStateMachine.currentState = STATE_EARLY_STANCE;//STATE_LATE_SWING;
					}
				}

				break; // case STATE_EARLY_SWING
	        }

			case STATE_LATE_SWING: //4
			{
				if (isTransitioning) {
					ankleWalkParamx->transitionId = 0;
				}
				ankleWalkParamx->timerInSwing++;

				actx->tauDes = getImpedanceTorqueParams(actx, &ankleWalkParamx->ankleGainsLsw);

				//---------------------- LATE SWING TRANSITION VECTORS ----------------------//
				if(timeInState > LSW_TO_EST_DELAY) {

					if (timeInState >= LSW_TO_EMG_DELAY && mitEmgGetState() == 1){
					//---------------------- FREE SPACE EMG TRANSITION VECTORS ----------------------//
						kneeAnkleStateMachine.currentState = STATE_LSW_EMG;
						ankleWalkParamx->transitionId = 4;

					} // VECTOR (1): Late Swing -> Early Stance (hard heal strike) - Condition 1
					else if (actx->jointTorque > HARD_HEELSTRIKE_TORQUE_THRESH && actx->jointTorqueRate > HARD_HEELSTRIKE_TORQ_RATE_THRESH) {
						kneeAnkleStateMachine.currentState = STATE_EARLY_STANCE;
						ankleWalkParamx->transitionId = 1;
					}
					// VECTOR (1): Late Swing -> Early Stance (gentle heal strike) - Condition 2 -
					else if (actx->jointTorqueRate > GENTLE_HEELSTRIKE_TORQ_RATE_THRESH) {
						kneeAnkleStateMachine.currentState = STATE_EARLY_STANCE;
						ankleWalkParamx->transitionId = 2;
					}
					// VECTOR (1): Late Swing -> Early Stance (toe strike) - Condition 3
					else if (actx->jointAngleDegrees < HARD_TOESTRIKE_ANGLE_THRESH) {
						kneeAnkleStateMachine.currentState = STATE_EARLY_STANCE;
						ankleWalkParamx->transitionId = 3;
					}

				}
					//------------------------- END OF TRANSITION VECTORS ------------------------//


				break;
			}
	        default:
	        {
	            //turn off control.
	            actx->tauDes = 0;
				ankleWalkParamx->timerInSwing = 0;
				ankleWalkParamx->timerInStance = 0;

	            kneeAnkleStateMachine.currentState = STATE_EARLY_STANCE;
	            break;
	        }
	    }

	    //update last state in preparation for next loop
	    kneeAnkleStateMachine.lastSmState = kneeAnkleStateMachine.onEntrySmState;

}

/** Impedance Control Level-ground Walking FSM
	Ankle is SLAVE:	determines walking state. Sends out the state to the Knee
	Knee is MASTER: receives kneeAnkleStateMachine.currentState updates from
					the Ankle actuator, and changes impedance settings accordingly
	Param: actx(Act_s) - Actuator structure to track sensor values
	Param: actx(Act_s)

	ptorqueDes pointer to float meant to be updated with desired torque TODO:find out what this is
*/ //void setKneeAnkleFlatGroundFSM(Act_s *actx);

void setKneeAnkleFlatGroundFSM(Act_s *actx, WalkParams *ankleWalkParamx) {

    static int8_t isTransitioning = 0;
    static uint32_t timeInState = 0;
    static int8_t passedStanceThresh = 0;

//    ankleGainsMst = ankleGainsEst;
//    ankleGainsLsw = ankleGainsEsw;
//    ankleGainsLsw.k1 = ankleGainsEsw.k1;

//    kneeGainsMst = kneeGainsEst;
//    kneeGainsLsw = kneeGainsEst;


    kneeAnkleStateMachine.onEntrySlaveSmState = kneeAnkleStateMachine.slaveCurrentState; // save the state on entry, assigned to last_currentState on exit

	#ifdef IS_KNEE
		//TODO: See if this is reasonable, change state to slave state, unless in early swing. In that case only switch when local state decides to change
	    if (kneeAnkleStateMachine.currentState == STATE_EARLY_SWING  || kneeAnkleStateMachine.currentState == STATE_LATE_SWING)
	    {
//	    	kneeAnkleStateMachine.currentState = kneeAnkleStateMachine.slaveCurrentState;
	    	//kneeAnkleStateMachine.currentState = kneeAnkleStateMachine.currentState;
	    	// do nothing, stay in current state.
	    } else
	    {
	    	kneeAnkleStateMachine.currentState = kneeAnkleStateMachine.slaveCurrentState;
	    }
	#endif

    kneeAnkleStateMachine.onEntrySmState = kneeAnkleStateMachine.currentState; // save the state on entry, assigned to last_currentState on exit

    // Check for state change, then set isTransitioning flag
    if (kneeAnkleStateMachine.currentState == kneeAnkleStateMachine.lastSmState) {
        isTransitioning = 0;
        timeInState++;
    } else {
        // State transition, reset timers and set entry flag
        timeInState = 0;
        isTransitioning = 1;
    }


    switch (kneeAnkleStateMachine.currentState) {

        case STATE_IDLE:
        {
        	//error handling here (should never be in STATE_IDLE by the time you get here)
            break;
        }
        case STATE_INIT:
        {
        	#ifdef IS_ANKLE
        		actx->tauDes = 0.0;	// Initialize to no commanded torque
			#elif defined(IS_KNEE)
        		actx->tauDes = 0.0;
			#endif

			#ifdef IS_TORQUE_REPLAY
        		torqueRep.time_stance = 0.0;
        		torqueRep.standard_stance_period = 555.0; 	// [ms]
        		torqueRep.previous_stance_period = 555.0; 	// [ms]
        		torqueRep.time_swing = 0.0;
        		torqueRep.standard_swing_period = 400.0;  	// [ms]
        		torqueRep.previous_swing_period = 400.0;  	// [ms]
        		torqueRep.torqueScalingFactor = 0.1; 		// adjust maximum torque output
        		torqueRep.entry_replay = 0;					// verify if we were already running torque replay
    			torqueRep.begin = 0;


        		for(int i=0; i<TRAJ_SIZE; i++)
        		{
        			torqueRep.torque_traj_mscaled[i] = torque_traj[i] + ((USER_MASS - 70.0)*massGains[i]);
        		}
			#endif

        	kneeAnkleStateMachine.currentState = STATE_LATE_SWING;	// enter into early stance, this has stability. good idea for torque replay? might be better late swing

            break;
        }
        case STATE_EARLY_STANCE://0 // check impedance mode in here - only stance state for torque replay (goes directly to early swing)
        {

			if (isTransitioning && !passedStanceThresh) {
				ankleWalkParamx->scaleFactor = 1.0;
				passedStanceThresh = 0;

				#ifdef USE_EMG
				resetVirtualJoint(actx->jointAngleDegrees,0,0);
				#endif //USE_EMG
			}

			#ifdef IS_ANKLE

				#ifdef USE_EMG
				updateVirtualJoint(&ankleGainsEMG);
  	  	  	  	#endif //USE_EMG


				// Check impedance mode, length of swing will turn off torque replay
				// True goes into normal walking controller, False goes into TorqueReplay
//				if(checkImpedanceMode(&torqueRep))
//				{
					updateAnkleVirtualHardstopTorque(actx, ankleWalkParamx);
//					actx->tauDes = ankleWalkParamx->virtualHardstopTq + getImpedanceTorque(actx, ankleGainsEst.k1, ankleGainsEst.b, ankleGainsEst.thetaDes) + getImpedanceTorque(actx, ankleGainsEMG.k1, ankleGainsEMG.b, ankleGainsEMG.thetaDes);
					actx->tauDes = ankleWalkParamx->virtualHardstopTq + getImpedanceTorqueParams(actx, &ankleWalkParamx->ankleGainsEst);

					if (JNT_ORIENT*actx->jointAngleDegrees > ankleWalkParamx->virtualHardstopEngagementAngle)
					{
						kneeAnkleStateMachine.currentState = STATE_MID_STANCE;
						passedStanceThresh = 1;
					}
//				} else {
//
//					torqueRep.entry_replay = 1;	// let's us know we're in torqueReplay mode
//
//					actx->tauDes = torqueTracking(&torqueRep);	//this is where teh magic happens, generate torque profile
//
//					torqueRep.time_stance++;	// stance timer
//
//					if ( (abs(actx->jointTorque) < ANKLE_UNLOADED_TORQUE_THRESH) && (timeInState > LST_TO_ESW_DELAY ) )
//					{
//						kneeAnkleStateMachine.currentState = STATE_LATE_SWING;
//						torqueRep.previous_stance_period = torqueRep.time_stance;
//						torqueRep.time_stance = 0.0;
//					}
//				}

				// transition to early swing
				//Late Stance Power transition vectors
				//todo: Should there be a way to jump back into early_stance in the event of running?


			#elif defined(IS_KNEE)
				actx->tauDes = getImpedanceTorque(actx, kneeGainsEst.k1, kneeGainsEst.b, kneeGainsEst.thetaDes);
			#endif

			break;
        }


        case STATE_MID_STANCE: //1
        {
			if (isTransitioning) {

			}

			#ifdef IS_ANKLE


    			// Stance transition vectors, only go into next state. This is a stable place to be.
        		// Transition occurs based on reaching torque threshold. (future: update this threshold based on speed)

				updateAnkleVirtualHardstopTorque(actx, ankleWalkParamx);	// Bring in
//				actx->tauDes = ankleWalkParamx->virtualHardstopTq + getImpedanceTorque(actx, ankleGainsMst.k1, ankleGainsMst.b, ankleGainsMst.thetaDes);
				actx->tauDes = ankleWalkParamx->virtualHardstopTq + getImpedanceTorqueParams(actx, &ankleWalkParamx->ankleGainsMst);

    			// Stance transition vectors, only go into next state. This is a stable place to be.

    			if (actx->jointTorque > ankleWalkParamx->lspEngagementTorque) {
    				kneeAnkleStateMachine.currentState = STATE_LATE_STANCE_POWER;
    			}

			#elif defined(IS_KNEE)
        		actx->tauDes = getImpedanceTorque(actx, kneeGainsMst.k1, kneeGainsMst.b, kneeGainsMst.thetaDes);
			#endif

        	break;
        }
        case STATE_LATE_STANCE_POWER: //2
        {
			if (isTransitioning) {
				ankleWalkParamx->samplesInLSP = 0.0;
				ankleWalkParamx->lspEntryTq = actx->jointTorque;
			}

			#ifdef IS_ANKLE
				// This is the scaling factor for ramping into powered pushoff
				if (ankleWalkParamx->samplesInLSP < ankleWalkParamx->lstPGDelTics){
					ankleWalkParamx->samplesInLSP++;
				}

				updateAnkleVirtualHardstopTorque(actx, ankleWalkParamx);

				//Linear ramp to push off, pickup where hardstop leftoff, use stiffness ankleGainsLst to get us to target point.
//				actx->tauDes = ankleWalkParamx->virtualHardstopTq + (ankleWalkParamx->samplesInLSP/ankleWalkParamx->lstPGDelTics) * getImpedanceTorque(actx, ankleGainsLst.k1, ankleGainsLst.b, ankleGainsLst.thetaDes);  // drops off after zero when hardstop goes away
				actx->tauDes = ankleWalkParamx->virtualHardstopTq + (ankleWalkParamx->samplesInLSP/ankleWalkParamx->lstPGDelTics) * getImpedanceTorqueParams(actx, &ankleWalkParamx->ankleGainsLst);  // drops off after zero when hardstop goes away
				//Late Stance Power transition vectors
				//todo: Should there be a way to jump back into early_stance in the event of running?
				if ( (fabs(actx->jointTorque) < ANKLE_UNLOADED_TORQUE_THRESH) && (timeInState > LST_TO_ESW_DELAY )) //&& (actx->jointAngleDegrees >=  ankleGainsLst.thetaDes -1.0) ) {	// not sure we need the timeInState? what's the point? just maker sure it's kicking?
				{
					kneeAnkleStateMachine.currentState = STATE_LATE_SWING;
				}
			#elif defined(IS_KNEE)
				actx->tauDes = getImpedanceTorque(actx, kneeGainsLst.k1, kneeGainsLst.b, kneeGainsLst.thetaDes);
			#endif

            break;
        }

        case STATE_EARLY_SWING: //3
        {
			//Put anything you want to run ONCE during state entry.
			if (isTransitioning) {
				ankleWalkParamx->virtualHardstopTq = 0.0;

				// initialize cubic spline params once
//				initializeCubicSplineParams(&cubicSpline, actx, ankleGainsEsw, splineTime); // last parameter is res_factor (delta X - time)
			}

			#ifdef IS_ANKLE

//				actx->tauDes = getImpedanceTorque(actx, ankleGainsEsw.k1, ankleGainsEsw.b, ankleGainsEsw.thetaDes);
				actx->tauDes = getImpedanceTorqueParams(actx, &ankleWalkParamx->ankleGainsEsw);
				torqueRep.time_swing++;

				if(actx->jointAngleDegrees <= ankleWalkParamx->ankleGainsEsw.thetaDes && timeInState >= ESW_TO_LSW_DELAY)
				{
					kneeAnkleStateMachine.currentState = STATE_LATE_SWING;
				}

			#elif defined(IS_KNEE)
				actx->tauDes = getImpedanceTorque(actx, kneeGainsEsw.k1, kneeGainsEsw.b, kneeGainsEsw.thetaDes);

				// This State is different, Knee decides on its own when to transition to STATE_LATE_SWING,
				// waits for knee to reach max flexion, then switches to late
//				if( (actx->jointVelDegrees <= 0) || (actx->jointAngleDegrees >= kneeGainsEsw.thetaDes) )
				if(  (actx->jointAngleDegrees >= kneeGainsEsw.thetaDes)   )
				{
					kneeAnkleStateMachine.currentState = STATE_LATE_SWING;
				}

			#endif

			break; // case STATE_EARLY_SWING
        }
		case STATE_LATE_SWING: //4
		{
			if (isTransitioning) {
				ankleWalkParamx->transitionId = 0;
			}

			#ifdef IS_ANKLE

//				actx->tauDes = getImpedanceTorque(actx, ankleGainsLsw.k1, ankleGainsLsw.b, ankleGainsLsw.thetaDes);
				actx->tauDes = getImpedanceTorqueParams(actx, &ankleWalkParamx->ankleGainsLsw);

				torqueRep.time_swing++;

				//---------------------- LATE SWING TRANSITION VECTORS ----------------------//
				if(timeInState > LSW_TO_EST_DELAY) {

					// Late Swing -> Early Stance (hard toe strike) - Running
//					if ( actx->jointTorque > HARD_TOESTRIKE_TORQUE_THRESH || actx->jointVelDegrees > HARD_TOESTRIKE_VEL_THRESH_DEG)
					if ( fabs(actx->jointTorque) > HARD_TOESTRIKE_TORQUE_THRESH )
					{
//						kneeAnkleStateMachine.currentState = STATE_MID_STANCE;
						kneeAnkleStateMachine.currentState = STATE_EARLY_STANCE;
						ankleWalkParamx->transitionId = 0;

						torqueRep.previous_swing_period = torqueRep.time_swing;
						torqueRep.time_swing = 0;
					}
					// Late Swing -> Early Stance (hard heal strike)
					else if ( fabs(actx->jointTorque) > HARD_HEELSTRIKE_TORQUE_THRESH && fabs(actx->jointTorqueRate) > GENTLE_HEELSTRIKE_TORQ_RATE_THRESH)
					{
						kneeAnkleStateMachine.currentState = STATE_EARLY_STANCE;
						ankleWalkParamx->transitionId = 1;

						torqueRep.previous_swing_period = torqueRep.time_swing;
						torqueRep.time_swing = 0;
					}
					// Late Swing -> Early Stance (gentle heal strike) - Condition 2 -
					else if ( actx->jointTorque < GENTLE_HEELSTRIKE_TORQUE_THRESH && actx->jointAngleDegrees <= (ankleWalkParamx->ankleGainsEst.thetaDes + 1.0) )
					{
						kneeAnkleStateMachine.currentState = STATE_EARLY_STANCE;
						ankleWalkParamx->transitionId = 2;

						torqueRep.previous_swing_period = torqueRep.time_swing;
						torqueRep.time_swing = 0;
					}
					else if ( fabs(actx->jointTorque) > GENTLE_HEELSTRIKE_TORQUE_THRESH)
					{
						kneeAnkleStateMachine.currentState = STATE_EARLY_STANCE;
						ankleWalkParamx->transitionId = 3;

						torqueRep.previous_swing_period = torqueRep.time_swing;
						torqueRep.time_swing = 0;
					}
					//TODO: might want to go into an idle state of some sort, check how this works.

				}
				//------------------------- END OF TRANSITION VECTORS ------------------------//
			#elif defined(IS_KNEE)
				float angleTracking = actx->jointAngleDegrees;
				actx->tauDes = getImpedanceTorque(actx, kneeGainsLsw.k1, kneeGainsLsw.b, angleTracking);
				if(timeInState > LSW_TO_EST_DELAY)
				{
					if ( (actx->jointAngleDegrees <= kneeGainsLsw.thetaDes) || (kneeAnkleStateMachine.onEntrySlaveSmState == STATE_EARLY_STANCE) )
					{
						kneeAnkleStateMachine.currentState = STATE_EARLY_STANCE;
					}
				}
			#endif

			break;
		}
        default:
        {
            //turn off control.
            actx->tauDes = 0;
            kneeAnkleStateMachine.currentState = STATE_INIT;

            break;
        }
    }

    //update last state in preparation for next loop
    kneeAnkleStateMachine.lastSmState = kneeAnkleStateMachine.onEntrySmState;



}


void setAnklePassiveDeviceFSM(Act_s *actx){

}

//
/** This ramps down the Stiffness of early stance K, picking up the user and bringing them up to ankleGainsEst.thetaDes
    NOTE/TODO: Hugh may prefer much more stiffness here, this ramps down teh stiffness.
	Param: actx(Act_s) - Actuator structure to track sensor values
	Param: wParams(WalkParams) - Parameters relating to walking states
*/
void updateImpedanceParams(Act_s *actx, WalkParams *wParams) {

	wParams->scaleFactor = wParams->scaleFactor * wParams->earlyStanceDecayConstant;

	wParams->ankleGainsEst.k1 = wParams->earlyStanceKF + wParams->scaleFactor * (wParams->earlyStanceK0 - wParams->earlyStanceKF);

    if (actx->jointAngleDegrees < wParams->ankleGainsEst.thetaDes) {
    	wParams->ankleGainsEst.thetaDes = actx->jointAngleDegrees;
    }
}

/** TODO: Find out what this does
	Param: actx(Act_s) - Actuator structure to track sensor values
	Param: wParams(WalkParams) - Parameters relating to walking states
*/
void updateAnkleVirtualHardstopTorque(Act_s *actx, WalkParams *wParams) {

	if (JNT_ORIENT*actx->jointAngleDegrees > wParams->virtualHardstopEngagementAngle) {
		wParams->virtualHardstopTq = wParams->virtualHardstopK * ((JNT_ORIENT * actx->jointAngleDegrees) - wParams->virtualHardstopEngagementAngle);
	} else {
		wParams->virtualHardstopTq = 0.0;
	}
}

// torque replay functions
int8_t checkImpedanceMode(TorqueRep *torqueRep){
	// include user writing for setting impedance mode

	if( (torqueRep->previous_swing_period >= IMPEDANCE_MODE_THRESHOLD) && (torqueRep->entry_replay == 1) )
	{ // Check that you were in TorqueReplay, but slowed down so turn off
		return 1; // might add an user write to switch to impedance mode
	}
	else if ( ( torqueRep->previous_swing_period <= torqueRep->standard_swing_period ) && torqueRep->begin)
	{ // Get up to speed before turning on TorqueReplay
		return 0;
	}
	else
	{ // Torque replay is running.
		return 0;
	}
}


float torqueTracking(TorqueRep *torqueRep)
{	// Replay a torque profile
	if(torqueRep->previous_stance_period > torqueRep->standard_stance_period)
	{
		torqueRep->previous_stance_period = torqueRep->standard_stance_period;
	}

	torqueRep->speedFactor = (1.0 - (float) ( torqueRep->previous_stance_period / torqueRep->standard_stance_period ) )*100.0;
	torqueRep->percent = torqueRep->time_stance / torqueRep->previous_stance_period;

	if(torqueRep->percent > 1.0)
	{
		torqueRep->percent = 1.0;
	}

	torqueRep->index = round( torqueRep->percent * (float) TRAJ_SIZE );

	if( torqueRep->index > 1000 )
	{
		torqueRep->index = 1000;
	}

	torqueRep->tauDes = torqueRep->torqueScalingFactor * ( torqueRep->torque_traj_mscaled[torqueRep->index] + ( torqueRep->speedFactor*speedGains[torqueRep->index] ) );

	return torqueRep->tauDes;
}


void setTorqueAnklePassive(Act_s *actx, WalkParams *wParams)
{ // simulate a passive actuator, just a damped spring
	actx->tauDes = getImpedanceTorque(actx, wParams->virtualHardstopK, wParams->virtualHardstopB, wParams->virtualHardstopEngagementAngle);
}

void setTorqueQuasiPassive(Act_s *actx, WalkParams *wParams)
{ // simulate a quasi-passive actuator, two springs, one for heelstrike, one for mid-stance.
	actx->tauDes = 0; //
}

#endif //BOARD_TYPE_FLEXSEA_MANAGE

#ifdef __cplusplus
}
#endif


