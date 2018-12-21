
#ifdef __cplusplus
extern "C" {
#endif

#include "walking_state_machine.h"
#include "state_variables.h"
#include <user-mn-MIT-DLeg.h>
//#include "user-mn-MIT-EMG.h"
#include "flexsea_user_structs.h"
#include "flexsea_system.h"
#include "flexsea.h"
#include "spline_functions.h"
#include "main.h"
#include <math.h>

WalkingStateMachine stateMachine;
Act_s act1;
WalkParams walkParams;
CubicSpline cubicSpline;

// Gain Parameters are modified to match our joint angle convention (RHR for right ankle, wearer's perspective)
GainParams eswGains = {1.5, 0.0, 0.3, -10.0};
GainParams lswGains = {1.5,0.0,  0.3, -5.0};
GainParams estGains = {0.0, 0.0, 0.2, 0.0};	// may want to increase this damping, at least.
GainParams lstPowerGains = {4.5,0.0,  0.1, 14};



#ifdef BOARD_TYPE_FLEXSEA_MANAGE



/** Impedance Control Level-ground Walking FSM
	Based on BiOM ankle and simplified.
	Finds desired torque.

	@param ptorqueDes pointer to float meant to be updated with desired torque
*/
void runFlatGroundFSM(Act_s *actx) {

    static int8_t isTransitioning = 0;
    static uint32_t time_in_state = 0;
    static int8_t passedStanceThresh = 0;



    stateMachine.on_entry_sm_state = stateMachine.current_state; // save the state on entry, assigned to last_current_state on exit

    actx->tauDes = 0;

    // Check for state change, then set isTransitioning flag
    if (stateMachine.current_state == stateMachine.last_sm_state) {
        isTransitioning = 0;
        time_in_state++;
    } else {
        // State transition, reset timers and set entry flag
        time_in_state = 0;
        isTransitioning = 1;
    }

    switch (stateMachine.current_state) {

        case STATE_IDLE: //0
            //error handling here (should never be in STATE_IDLE by the time you get here)
            break;

        case STATE_INIT: //1

        	stateMachine.current_state = STATE_EARLY_STANCE;

            break;
					
        case STATE_EARLY_SWING: //2

            //Put anything you want to run ONCE during state entry.
			if (isTransitioning) {
				walkParams.virtual_hardstop_tq = 0.0;

				// initialize cubic spline params once
				initializeCubicSplineParams(&cubicSpline, actx, eswGains, 100.0); // last parameter is res_factor (delta X - time)
			}

			// Cubic Spline
			calcCubicSpline(&cubicSpline);
			eswGains.thetaDes = cubicSpline.Y; //new thetaDes after cubic spline

            actx->tauDes = calcJointTorque(eswGains, actx, &walkParams);

            //Early Swing transition vectors

			if (time_in_state >= 200) {
                stateMachine.current_state = STATE_EARLY_STANCE;      //Transition occurs even if the early swing motion is not finished
            }

            //run any exit code here

            break; // case STATE_EARLY_SWING

        case STATE_LATE_SWING: //3

			if (isTransitioning) {
				walkParams.transition_id = 0;
			}

			actx->tauDes = calcJointTorque(lswGains, actx, &walkParams);


			//---------------------- LATE SWING TRANSITION VECTORS ----------------------//
			if(time_in_state > LSW_TO_EST_DELAY) {

				if (time_in_state >= LSW_TO_EMG_DELAY && MIT_EMG_getState() == 1){
					//---------------------- FREE SPACE EMG TRANSITION VECTORS ----------------------//
					stateMachine.current_state = STATE_LSW_EMG;
					walkParams.transition_id = 4;

				} // VECTOR (1): Late Swing -> Early Stance (hard heal strike) - Condition 1
				else if (actx->jointTorque > HARD_HEELSTRIKE_TORQUE_THRESH && actx->jointTorqueRate > HARD_HEELSTRIKE_TORQ_RATE_THRESH) {
					stateMachine.current_state = STATE_EARLY_STANCE;
					walkParams.transition_id = 1;
				}
				// VECTOR (1): Late Swing -> Early Stance (gentle heal strike) - Condition 2 -
				else if (actx->jointTorqueRate > GENTLE_HEELSTRIKE_TORQ_RATE_THRESH) {
					stateMachine.current_state = STATE_EARLY_STANCE;
					walkParams.transition_id = 2;
				}
				// VECTOR (1): Late Swing -> Early Stance (toe strike) - Condition 3
				else if (actx->jointAngleDegrees < HARD_TOESTRIKE_ANGLE_THRESH) {
					stateMachine.current_state = STATE_EARLY_STANCE;
					walkParams.transition_id = 3;
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

				actx->tauDes = walkParams.virtual_hardstop_tq + calcJointTorque(estGains, actx, &walkParams);


				//Early Stance transition vectors

				if (abs(actx->jointTorque) > 20.0){
					passedStanceThresh = 1;
				}

				if (actx->jointTorque > walkParams.lspEngagementTorque) {
					stateMachine.current_state = STATE_LATE_STANCE_POWER;      //Transition occurs even if the early swing motion is not finished
				}


				//------------------------- END OF TRANSITION VECTORS ------------------------//
				break;

        case STATE_LATE_STANCE_POWER: //6

        	{

				if (isTransitioning) {
					walkParams.samplesInLSP = 0.0;
					walkParams.lsp_entry_tq = actx->jointTorque;
				}

				if (walkParams.samplesInLSP < walkParams.lstPGDelTics){
					walkParams.samplesInLSP++;
				}

				updateVirtualHardstopTorque(actx, &walkParams);


				//Linear ramp push off
				actx->tauDes = walkParams.virtual_hardstop_tq + (walkParams.samplesInLSP/walkParams.lstPGDelTics) * calcJointTorque(lstPowerGains, actx, &walkParams);



				//Late Stance Power transition vectors
				// VECTOR (1): Late Stance Power -> Early Swing - Condition 1
				if (abs(actx->jointTorque) < ANKLE_UNLOADED_TORQUE_THRESH && time_in_state > 100) {
					stateMachine.current_state = STATE_EARLY_SWING;
				}
        	}

            break;

        
        default:

            //turn off control.
            actx->tauDes = 0;
            stateMachine.current_state = STATE_LATE_SWING;

            break;
	
    }

    //update last state in preparation for next loop
    stateMachine.last_sm_state = stateMachine.on_entry_sm_state;



}

/** Impedance Control Torque
	Calculates desired torque based on impedance gain parameters

	@param  gainParams struct with all the state's impedance parameters
	@param	actx Actuator params
	@param  wParams Parameters relating to walking states
    @return float desired torque at joint (Nm)
*/
float calcJointTorque(GainParams gainParams, Act_s *actx, WalkParams *wParams) {

	return gainParams.k1 * (gainParams.thetaDes - actx->jointAngleDegrees) \
         - gainParams.b * actx->jointVelDegrees ;
}



// This ramps down the Stiffness of early stance K, picking up the user and bringing them up to estGains.thetaDes
// NOTE/TODO: Hugh may prefer much more stiffness here, this ramps down teh stiffness.
void updateImpedanceParams(Act_s *actx, WalkParams *wParams) {

	wParams->scaleFactor = wParams->scaleFactor * wParams->earlyStanceDecayConstant;

    estGains.k1 = wParams->earlyStanceKF + wParams->scaleFactor * (wParams->earlyStanceK0 - wParams->earlyStanceKF);

    if (actx->jointAngleDegrees < estGains.thetaDes) {
    	estGains.thetaDes = actx->jointAngleDegrees;
    }
}


void updateVirtualHardstopTorque(Act_s *actx, WalkParams *wParams) {

	if (JNT_ORIENT*actx->jointAngleDegrees > wParams->virtualHardstopEngagementAngle) {
		wParams->virtual_hardstop_tq = wParams->virtualHardstopK * ((JNT_ORIENT * actx->jointAngleDegrees) - wParams->virtualHardstopEngagementAngle);
	} else {
		wParams->virtual_hardstop_tq = 0.0;
	}
}



#endif //BOARD_TYPE_FLEXSEA_MANAGE

#ifdef __cplusplus
}
#endif


