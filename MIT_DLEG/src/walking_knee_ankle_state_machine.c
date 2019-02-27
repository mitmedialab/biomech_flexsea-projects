
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

//****************************************************************************
// Definition(s):
//****************************************************************************
WalkingStateMachine kneeAnkleStateMachine;
WalkParams ankleWalkParams, kneeWalkParams;
CubicSpline cubicSpline;

// Gain Parameters are modified to match our joint angle convention (RHR for right ankle, wearer's perspective). Positive Plantaflexion
GainParams ankleGainsEst = {3.5, 0.0, 0.2, 0.0};	// may want to increase this damping, at least.
GainParams ankleGainsMst = {3.5, 0.0, 0.2, 0.0};	// may want to increase this damping, at least.
GainParams ankleGainsLst = {4.5,0.0,  0.1, 14};
GainParams ankleGainsEsw = {1.5, 0.0, 0.3, -10.0};
GainParams ankleGainsLsw = {1.5, 0.0,  0.3, -5.0};

//Knee, Positive Knee Flexion
GainParams kneeGainsEst = {1.5, 0.0, 0.3, -10.0};
GainParams kneeGainsMst = {1.5, 0.0, 0.3, -10.0};
GainParams kneeGainsLst = {1.5, 0.0, 0.3, -10.0};
GainParams kneeGainsEsw = {1.5, 0.0, 0.3, -10.0};
GainParams kneeGainsLsw = {1.5, 0.0, 0.3, -10.0};

#ifdef BOARD_TYPE_FLEXSEA_MANAGE

//****************************************************************************
// Functions:
//****************************************************************************

/** Impedance Control Level-ground Walking FSM
	Finds desired torque.
	Param: ankleAct(Act_s) - Actuator structure to track sensor values

	ptorqueDes pointer to float meant to be updated with desired torque TODO:find out what this is
*/ //void setKneeAnkleFlatGroundFSM(Act_s *actx);
void setKneeAnkleFlatGroundFSM(Act_s *ankleAct, Act_s *kneeAct) {

    static int8_t isTransitioning = 0;
    static uint32_t timeInState = 0;
    static int8_t passedStanceThresh = 0;

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
            //error handling here (should never be in STATE_IDLE by the time you get here)
            break;

        case STATE_INIT:

        	ankleAct->tauDes = 0;	// Initialize to no commanded torque
        	kneeAnkleStateMachine.currentState = STATE_EARLY_STANCE;	// enter into early stance, this has stability.

            break;

        case STATE_EARLY_STANCE:

			if (isTransitioning && !passedStanceThresh) {
				ankleWalkParams.scaleFactor = 1.0;
//					ankleGainsEst.k1 = ankleWalkParams.earlyStanceK0;
//				ankleGainsEst.thetaDes = ankleAct->jointAngleDegrees;	// used by updateImpedanceParams, if in use, could be turned off.
				passedStanceThresh = 0;
			}

			//Try moving this into MID-STANCE
			//updateAnkleVirtualHardstopTorque(ankleAct, &ankleWalkParams);	// Bring in

//			updateImpedanceParams(ankleAct, &ankleWalkParams);	//Todo: Probably want to bring this back to ease into stance, though Hugh prefers a stiff ankle - why it was removed
			ankleAct->tauDes = getImpedanceTorque(ankleAct, ankleGainsEst.k1, ankleGainsEst.b, ankleGainsEst.thetaDes);
			kneeAct->tauDes = getImpedanceTorque(kneeAct, kneeGainsEst.k1, kneeGainsEst.b, kneeGainsEst.thetaDes);


			//Try moving this into midStance, might bring it back again.
//			ankleAct->tauDes = ankleWalkParams.virtualHardstopTq + getImpedanceTorque(ankleAct, ankleGainsEst.k1, ankleGainsEst.b, ankleGainsEst.thetaDes);

			if (JNT_ORIENT*ankleAct->jointAngleDegrees > ankleWalkParams.virtualHardstopEngagementAngle) {
				kneeAnkleStateMachine.currentState = STATE_MID_STANCE;
				passedStanceThresh = 1;
			}


			// Todo: try out moving this into midStance
//			//Early Stance transition vectors
//
//			if (abs(ankleAct->jointTorque) > 20.0){
//				passedStanceThresh = 1;
//			}
//
//			if (ankleAct->jointTorque > ankleWalkParams.lspEngagementTorque) {
//				kneeAnkleStateMachine.currentState = STATE_LATE_STANCE_POWER;      //Transition occurs even if the early swing motion is not finished
//			}


			//------------------------- END OF TRANSITION VECTORS ------------------------//
			break;

        case STATE_MID_STANCE:

			if (isTransitioning) {

			}

        	updateAnkleVirtualHardstopTorque(ankleAct, &ankleWalkParams);	// Bring in
			ankleAct->tauDes = ankleWalkParams.virtualHardstopTq + getImpedanceTorque(ankleAct, ankleGainsMst.k1, ankleGainsMst.b, ankleGainsMst.thetaDes);
			kneeAct->tauDes = getImpedanceTorque(kneeAct, kneeGainsMst.k1, kneeGainsMst.b, kneeGainsMst.thetaDes);

			//Stance transition vectors
			if (ankleAct->jointTorque > ankleWalkParams.lspEngagementTorque) {
				kneeAnkleStateMachine.currentState = STATE_LATE_STANCE_POWER;      //Transition occurs even if the early swing motion is not finished
			} else if (JNT_ORIENT*ankleAct->jointAngleDegrees <= ankleWalkParams.virtualHardstopEngagementAngle)	// go back to Early stance
			{
				kneeAnkleStateMachine.currentState = STATE_EARLY_STANCE;			// If Hardstop is no longer engaged, go back to early stance, for standing.
				passedStanceThresh = 1;
			}

        	break;

        case STATE_LATE_STANCE_POWER: //6

			if (isTransitioning) {
				ankleWalkParams.samplesInLSP = 0.0;
				ankleWalkParams.lspEntryTq = ankleAct->jointTorque;
			}

			if (ankleWalkParams.samplesInLSP < ankleWalkParams.lstPGDelTics){
				ankleWalkParams.samplesInLSP++;
			}

			updateAnkleVirtualHardstopTorque(ankleAct, &ankleWalkParams);

			//Linear ramp push off
			ankleAct->tauDes = ankleWalkParams.virtualHardstopTq + (ankleWalkParams.samplesInLSP/ankleWalkParams.lstPGDelTics) * getImpedanceTorque(ankleAct, ankleGainsLst.k1, ankleGainsLst.b, ankleGainsLst.thetaDes);

			kneeAct->tauDes = getImpedanceTorque(kneeAct, kneeGainsLst.k1, kneeGainsLst.b, kneeGainsLst.thetaDes);


			//Late Stance Power transition vectors
			// VECTOR (1): Late Stance Power -> Early Swing - Condition 1
			if (abs(ankleAct->jointTorque) < ANKLE_UNLOADED_TORQUE_THRESH && timeInState > 100) {
				kneeAnkleStateMachine.currentState = STATE_EARLY_SWING;
			}

            break;

        case STATE_EARLY_SWING:

			//Put anything you want to run ONCE during state entry.
			if (isTransitioning) {
				ankleWalkParams.virtualHardstopTq = 0.0;

				// initialize cubic spline params once
				initializeCubicSplineParams(&cubicSpline, ankleAct, ankleGainsEsw, 100.0); // last parameter is res_factor (delta X - time)
			}

			// Cubic Spline
			calcCubicSpline(&cubicSpline);
			ankleGainsEsw.thetaDes = cubicSpline.Y; //new thetaDes after cubic spline

			ankleAct->tauDes = getImpedanceTorque(ankleAct, ankleGainsEsw.k1, ankleGainsEsw.b, ankleGainsEsw.thetaDes);
			kneeAct->tauDes = getImpedanceTorque(kneeAct, kneeGainsEsw.k1, kneeGainsEsw.b, kneeGainsEsw.thetaDes);

			//Early Swing transition vectors

			if (timeInState >= 200) {
				kneeAnkleStateMachine.currentState = STATE_LATE_SWING;      //Transition occurs even if the early swing motion is not finished
			}

			//run any exit code here

			break; // case STATE_EARLY_SWING

		case STATE_LATE_SWING:

			if (isTransitioning) {
				ankleWalkParams.transitionId = 0;
			}

			ankleAct->tauDes = getImpedanceTorque(ankleAct, ankleGainsLsw.k1, ankleGainsLsw.b, ankleGainsLsw.thetaDes);
			kneeAct->tauDes = getImpedanceTorque(kneeAct, kneeGainsLsw.k1, kneeGainsLsw.b, kneeGainsLsw.thetaDes);


			//---------------------- LATE SWING TRANSITION VECTORS ----------------------//
			if(timeInState > LSW_TO_EST_DELAY) {

				// VECTOR (1): Late Swing -> Early Stance (hard heal strike) - Condition 1
				if ( fabs(ankleAct->jointTorque) > HARD_HEELSTRIKE_TORQUE_THRESH && fabs(ankleAct->jointTorqueRate) > GENTLE_HEELSTRIKE_TORQ_RATE_THRESH) {
					kneeAnkleStateMachine.currentState = STATE_EARLY_STANCE;
					ankleWalkParams.transitionId = 1;
				}
				// VECTOR (1): Late Swing -> Early Stance (gentle heal strike) - Condition 2 -
				else if (fabs(ankleAct->jointTorque) > GENTLE_HEELSTRIKE_TORQUE_THRESH && ankleAct->jointAngleDegrees <= (ankleGainsEst.thetaDes + 1.0) ) {
					kneeAnkleStateMachine.currentState = STATE_EARLY_STANCE;
					ankleWalkParams.transitionId = 2;
				}
				//TODO: might want to go into an idle state of some sort, check how this works.

			}


			//------------------------- END OF TRANSITION VECTORS ------------------------//
			break;
        
        default:

            //turn off control.
            ankleAct->tauDes = 0;
            kneeAnkleStateMachine.currentState = STATE_INIT;

            break;
	
    }

    //update last state in preparation for next loop
    kneeAnkleStateMachine.lastSmState = kneeAnkleStateMachine.onEntrySmState;



}

///** Impedance Control Torque
//	Calculates desired torque based on impedance gain parameters
//
//	Param: gainParams(GainParams) - struct with all the state's impedance parameters
//	Param: ankleAct(Act_s) - Actuator structure to track sensor values
//	Param: wParams(WalkParams) - Parameters relating to walking states
//    Return: float desired torque at joint in NEWTON-METERS
//*/
//float calcJointTorque(GainParams gainParams, Act_s *actx, WalkParams *wParams) {
//
//	return gainParams.k1 * (gainParams.thetaDes - ankleAct->jointAngleDegrees) \
//         - gainParams.b * actx.jointVelDegrees ;
//}



//
/** This ramps down the Stiffness of early stance K, picking up the user and bringing them up to ankleGainsEst.thetaDes
    NOTE/TODO: Hugh may prefer much more stiffness here, this ramps down teh stiffness.
	Param: ankleAct(Act_s) - Actuator structure to track sensor values
	Param: wParams(WalkParams) - Parameters relating to walking states
*/
void updateImpedanceParams(Act_s *actx, WalkParams *wParams) {

	wParams->scaleFactor = wParams->scaleFactor * wParams->earlyStanceDecayConstant;

    ankleGainsEst.k1 = wParams->earlyStanceKF + wParams->scaleFactor * (wParams->earlyStanceK0 - wParams->earlyStanceKF);

    if (actx->jointAngleDegrees < ankleGainsEst.thetaDes) {
    	ankleGainsEst.thetaDes = actx->jointAngleDegrees;
    }
}

/** TODO: Find out what this does
	Param: ankleAct(Act_s) - Actuator structure to track sensor values
	Param: wParams(WalkParams) - Parameters relating to walking states
*/
void updateAnkleVirtualHardstopTorque(Act_s *actx, WalkParams *wParams) {

	if (JNT_ORIENT*actx->jointAngleDegrees > wParams->virtualHardstopEngagementAngle) {
		wParams->virtualHardstopTq = wParams->virtualHardstopK * ((JNT_ORIENT * actx->jointAngleDegrees) - wParams->virtualHardstopEngagementAngle);
	} else {
		wParams->virtualHardstopTq = 0.0;
	}
}



#endif //BOARD_TYPE_FLEXSEA_MANAGE

#ifdef __cplusplus
}
#endif


