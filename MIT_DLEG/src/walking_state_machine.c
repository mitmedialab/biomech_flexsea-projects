
#ifdef __cplusplus
extern "C" {
#endif

//****************************************************************************
// Include(s)
//****************************************************************************
#include "walking_state_machine.h"
#include <user-mn-MIT-DLeg.h>
//#include "user-mn-MIT-EMG.h"
#include "spline_functions.h"

//****************************************************************************
// Definition(s):
//****************************************************************************
WalkingStateMachine stateMachine;
WalkParams walkParams;
CubicSpline cubicSpline;

// Gain Parameters are modified to match our joint angle convention (RHR for right ankle, wearer's perspective)
GainParams eswGains = {1.5, 0.0, 0.3, -10.0};
GainParams lswGains = {1.5, 0.0,  0.3, -5.0};
GainParams estGains = {0.0, 0.0, 0.2, 0.0};	// may want to increase this damping, at least.
GainParams lstPowerGains = {4.5,0.0,  0.1, 14};



#ifdef BOARD_TYPE_FLEXSEA_MANAGE

//****************************************************************************
// Functions:
//****************************************************************************

/** Impedance Control Level-ground Walking FSM
	Finds desired torque.
	Param: actx(Act_s) - Actuator structure to track sensor values

	ptorqueDes pointer to float meant to be updated with desired torque TODO:find out what this is
*/
void runFlatGroundFSM(Act_s *actx) {

    static int8_t isTransitioning = 0;
    static uint32_t timeInState = 0;
    static int8_t passedStanceThresh = 0;

    stateMachine.onEntrySmState = stateMachine.currentState; // save the state on entry, assigned to last_currentState on exit

    actx->tauDes = 0; //todo: probably remove this.

    // Check for state change, then set isTransitioning flag
    if (stateMachine.currentState == stateMachine.lastSmState) {
        isTransitioning = 0;
        timeInState++;
    } else {
        // State transition, reset timers and set entry flag
        timeInState = 0;
        isTransitioning = 1;
    }

    switch (stateMachine.currentState) {

        case STATE_IDLE: //0
            //error handling here (should never be in STATE_IDLE by the time you get here)
            break;

        case STATE_INIT: //1

        	actx->tauDes = 0;	// Initialize to no commanded torque

        	stateMachine.currentState = STATE_EARLY_STANCE;

            break;
					
        case STATE_EARLY_SWING: //2

            //Put anything you want to run ONCE during state entry.
			if (isTransitioning) {
				walkParams.virtualHardstopTq = 0.0;

				// initialize cubic spline params once
				initializeCubicSplineParams(&cubicSpline, actx, eswGains, 100.0); // last parameter is res_factor (delta X - time)
			}

			// Cubic Spline
			calcCubicSpline(&cubicSpline);
			eswGains.thetaDes = cubicSpline.Y; //new thetaDes after cubic spline

            actx->tauDes = calcJointTorque(eswGains, actx, &walkParams);

            //Early Swing transition vectors

			if (timeInState >= 200) {
                stateMachine.currentState = STATE_EARLY_STANCE;      //Transition occurs even if the early swing motion is not finished
            }

            //run any exit code here

            break; // case STATE_EARLY_SWING

        case STATE_LATE_SWING: //3

			if (isTransitioning) {
				walkParams.transitionId = 0;
			}

			actx->tauDes = calcJointTorque(lswGains, actx, &walkParams);


			//---------------------- LATE SWING TRANSITION VECTORS ----------------------//
			if(timeInState > LSW_TO_EST_DELAY) {

				if (timeInState >= LSW_TO_EMG_DELAY && mitEmgGetState() == 1){
					//---------------------- FREE SPACE EMG TRANSITION VECTORS ----------------------//
					stateMachine.currentState = STATE_LSW_EMG;
					walkParams.transitionId = 4;

				} // VECTOR (1): Late Swing -> Early Stance (hard heal strike) - Condition 1
				else if (actx->jointTorque > HARD_HEELSTRIKE_TORQUE_THRESH && actx->jointTorqueRate > HARD_HEELSTRIKE_TORQ_RATE_THRESH) {
					stateMachine.currentState = STATE_EARLY_STANCE;
					walkParams.transitionId = 1;
				}
				// VECTOR (1): Late Swing -> Early Stance (gentle heal strike) - Condition 2 -
				else if (actx->jointTorqueRate > GENTLE_HEELSTRIKE_TORQ_RATE_THRESH) {
					stateMachine.currentState = STATE_EARLY_STANCE;
					walkParams.transitionId = 2;
				}
				// VECTOR (1): Late Swing -> Early Stance (toe strike) - Condition 3
				else if (actx->jointAngleDegrees < HARD_TOESTRIKE_ANGLE_THRESH) {
					stateMachine.currentState = STATE_EARLY_STANCE;
					walkParams.transitionId = 3;
				}

			}


            //------------------------- END OF TRANSITION VECTORS ------------------------//
            break;

        case STATE_EARLY_STANCE: //4

				if (isTransitioning) {
					walkParams.scaleFactor = 1.0;

					estGains.k1 = walkParams.earlyStanceK0;
					estGains.thetaDes = actx->jointAngleDegrees;

					passedStanceThresh = 0;
				}


				updateVirtualHardstopTorque(actx, &walkParams);
//				updateImpedanceParams(actx, &walkParams);	//Todo: Probably want to bring this back to ease into stance, though Hugh prefers a stiff ankle - why it was removed

				actx->tauDes = walkParams.virtualHardstopTq + calcJointTorque(estGains, actx, &walkParams);


				//Early Stance transition vectors

				if (abs(actx->jointTorque) > 20.0){
					passedStanceThresh = 1;
				}

				if (actx->jointTorque > walkParams.lspEngagementTorque) {
					stateMachine.currentState = STATE_LATE_STANCE_POWER;      //Transition occurs even if the early swing motion is not finished
				}


				//------------------------- END OF TRANSITION VECTORS ------------------------//
				break;

        case STATE_LATE_STANCE_POWER: //6

        	{

				if (isTransitioning) {
					walkParams.samplesInLSP = 0.0;
					walkParams.lspEntryTq = actx->jointTorque;
				}

				if (walkParams.samplesInLSP < walkParams.lstPGDelTics){
					walkParams.samplesInLSP++;
				}

				updateVirtualHardstopTorque(actx, &walkParams);


				//Linear ramp push off
				actx->tauDes = walkParams.virtualHardstopTq + (walkParams.samplesInLSP/walkParams.lstPGDelTics) * calcJointTorque(lstPowerGains, actx, &walkParams);



				//Late Stance Power transition vectors
				// VECTOR (1): Late Stance Power -> Early Swing - Condition 1
				if (abs(actx->jointTorque) < ANKLE_UNLOADED_TORQUE_THRESH && timeInState > 100) {
					stateMachine.currentState = STATE_EARLY_SWING;
				}
        	}

            break;

        
        default:

            //turn off control.
            actx->tauDes = 0;
            stateMachine.currentState = STATE_LATE_SWING;

            break;
	
    }

    //update last state in preparation for next loop
    stateMachine.lastSmState = stateMachine.onEntrySmState;



}

/** Impedance Control Torque
	Calculates desired torque based on impedance gain parameters

	Param: gainParams(GainParams) - struct with all the state's impedance parameters
	Param: actx(Act_s) - Actuator structure to track sensor values
	Param: wParams(WalkParams) - Parameters relating to walking states
    Return: float desired torque at joint in NEWTON-METERS
*/
float calcJointTorque(GainParams gainParams, Act_s *actx, WalkParams *wParams) {

	return gainParams.k1 * (gainParams.thetaDes - actx->jointAngleDegrees) \
         - gainParams.b * actx->jointVelDegrees ;
}



//
/** This ramps down the Stiffness of early stance K, picking up the user and bringing them up to estGains.thetaDes
    NOTE/TODO: Hugh may prefer much more stiffness here, this ramps down teh stiffness.
	Param: actx(Act_s) - Actuator structure to track sensor values
	Param: wParams(WalkParams) - Parameters relating to walking states
*/
void updateImpedanceParams(Act_s *actx, WalkParams *wParams) {

	wParams->scaleFactor = wParams->scaleFactor * wParams->earlyStanceDecayConstant;

    estGains.k1 = wParams->earlyStanceKF + wParams->scaleFactor * (wParams->earlyStanceK0 - wParams->earlyStanceKF);

    if (actx->jointAngleDegrees < estGains.thetaDes) {
    	estGains.thetaDes = actx->jointAngleDegrees;
    }
}

/** TODO: Find out what this does
	Param: actx(Act_s) - Actuator structure to track sensor values
	Param: wParams(WalkParams) - Parameters relating to walking states
*/
void updateVirtualHardstopTorque(Act_s *actx, WalkParams *wParams) {

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


