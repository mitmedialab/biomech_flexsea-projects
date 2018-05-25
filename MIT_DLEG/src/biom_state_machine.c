#ifdef __cplusplus
extern "C" {
#endif

#include "state_variables.h"
#include "biom_state_machine.h"
#include "user-mn-MIT-DLeg-2dof.h"
#include "user-mn-MIT-EMG.h"
#include "flexsea_user_structs.h"
#include "free_ankle_EMG.h"
#include "stand_ankle_EMG.h"
#include "cmd-DLeg.h"
#include "flexsea_system.h"
#include "flexsea.h"
#include <math.h>

WalkingStateMachine stateMachine;
Act_s act1;
WalkParams walkParams;

// Gain Parameters are modified to match our joint angle convention (RHR for right ankle, wearer's perspective)
GainParams eswGains = {1.5, 0.0, 0.3, -5.0};
GainParams lswGains = {1.5, 1, 0.3, -5.0};
GainParams estGains = {0.0, 0.0, 0.1, 0.0};
GainParams lstGains = {0.0, 0.0, 0.0, 0.0}; //currently unused in simple implementation
GainParams lstPowerGains = {0.0, 0.0, 0.0, 0.0};
GainParams emgStandGains = {0.0, 0.0, 0.0, 0.0}; //currently unused
GainParams emgFreeGains = {0.0, 0.0, 0.0, 0.0};

#ifdef BOARD_TYPE_FLEXSEA_MANAGE

//****************************************************************************
// Static Functions:
//****************************************************************************

static void updatePFDFState(Act_s *actx);

static void updateImpedanceParams(Act_s *actx, WalkParams *wParams);
static void updateUserWrites(Act_s *actx, WalkParams *wParams);
static float calcJointTorque(GainParams gainParams, Act_s *actx, WalkParams *wParams);
static float biomCalcImpedanceOneState(float k1, float b, float theta_set);
static void updateVirtualHardstopTorque(Act_s *actx, WalkParams *wParams);
static void initializeUserWrites(WalkParams *wParams);


/** Impedance Control Level-ground Walking FSM
	Based on BiOM ankle and simplified.
	Finds desired torque.

	@param ptorqueDes pointer to float meant to be updated with desired torque
*/
void runFlatGroundFSM(Act_s *actx) {

    static int8_t isTransitioning = 0;
    static uint32_t time_in_state = 0;


    if (!walkParams.initializedStateMachineVariables){
    	initializeUserWrites(&walkParams);

        actx->tauDes = 0;
    }

    updateUserWrites(actx, &walkParams);

    stateMachine.on_entry_sm_state = stateMachine.current_state; // save the state on entry, assigned to last_current_state on exit


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

//            stateMachine.current_state = STATE_LATE_SWING;
            stateMachine.current_state = 48;

            break;
					
        case STATE_EARLY_SWING: //2

            //Put anything you want to run ONCE during state entry.
			if (isTransitioning) {
				walkParams.virtual_hardstop_tq = 0.0;
			}

            actx->tauDes = calcJointTorque(eswGains, actx, &walkParams);

            //Early Swing transition vectors
            // VECTOR(1): Early Swing -> Late Swing
            if (time_in_state >= ESW_TO_LSW_DELAY) {
                stateMachine.current_state = STATE_LATE_SWING;      //Transition occurs even if the early swing motion is not finished
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

				if (actx->jointTorque > HARD_HEELSTRIKE_TORQUE_THRESH && actx->jointTorqueRate > HARD_HEELSTRIKE_TORQ_RATE_THRESH) {
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
					estGains.thetaDes = 0.0;
				}


//				updateVirtualHardstopTorque(actx, &walkParams);

				updateImpedanceParams(actx, &walkParams);

				actx->tauDes = calcJointTorque(estGains, actx, &walkParams);

				// VECTOR (A): Early Stance -> Late Stance (foot flat) - Condition 1

				if (abs(actx->jointTorque) < ANKLE_UNLOADED_TORQUE_THRESH && time_in_state > 100) {
					stateMachine.current_state = STATE_LATE_SWING;
				}

				//------------------------- END OF TRANSITION VECTORS ------------------------//
				break;
        case 48:

        	actx->tauDes = biomCalcImpedanceOneState(walkParams.earlyStanceKF, walkParams.earlyStanceB, walkParams.earlyStanceTheta);

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
static float calcJointTorque(GainParams gainParams, Act_s *actx, WalkParams *wParams) {
	return gainParams.k1 * (gainParams.thetaDes - actx->jointAngleDegrees) \
         - gainParams.b * actx->jointVelDegrees  + wParams->virtual_hardstop_tq;
}

static void updateUserWrites(Act_s *actx, WalkParams *wParams){

	//USER WRITE INITIALIZATION GOES HERE//////////////
	wParams->earlyStanceK0 = ( (float)user_data_1.w[0] ) /100.0;
	wParams->earlyStanceKF =( (float) user_data_1.w[1] ) /100.0 ;
	wParams->earlyStanceB = ( (float) user_data_1.w[2] ) / 100.0 ;
	wParams->thetaThreshDorsi = ( (float) user_data_1.w[3] );
	wParams->thetaThreshPlant = (float)user_data_1.w[4] ;
	wParams->earlyStanceTheta = (float) user_data_1.w[5] ;


	///////////////////////////////////////////////////

}

static void initializeUserWrites(WalkParams *wParams){

	wParams->earlyStanceK0 = 6.23;
	wParams->earlyStanceKF = 3.0;
	wParams->earlyStanceB = 0.2;
	wParams->earlyStanceTheta = 0;
	wParams->thetaThreshDorsi = -10;
	wParams->thetaThreshPlant = 10;
	wParams->earlyStanceDecayConstant = EARLYSTANCE_DECAY_CONSTANT;
	wParams->lstPGDelTics = 1;
	wParams->earlyStanceKF = 1;
	wParams->virtualHardstopK = 7;				//7 x 100
	wParams->virtualHardstopEngagementAngle = 0;	//0.0 x 1
	wParams->lspEngagementTorque = 80;
	wParams->virtual_hardstop_tq = 0.0;

	wParams->initializedStateMachineVariables = 1;

	//USER WRITE INITIALIZATION GOES HERE//////////////
	user_data_1.w[0] = (int16_t) ( wParams->earlyStanceK0 * 100.0 );
	user_data_1.w[1] = (int16_t) ( wParams->earlyStanceKF * 100.0 );
	user_data_1.w[2] = (int16_t) ( wParams->earlyStanceB * 100.0 );
	user_data_1.w[3] = (int16_t) wParams->thetaThreshDorsi;
	user_data_1.w[4] = (int16_t) wParams->thetaThreshPlant;
	user_data_1.w[5] = (int16_t) wParams->earlyStanceTheta;
	///////////////////////////////////////////////////


}

static void updateImpedanceParams(Act_s *actx, WalkParams *wParams) {

	wParams->scaleFactor = wParams->scaleFactor * wParams->earlyStanceDecayConstant;

    estGains.k1 = wParams->earlyStanceKF + wParams->scaleFactor * (wParams->earlyStanceK0 - wParams->earlyStanceKF);

//    if (actx->jointAngleDegrees < estGains.thetaDes) {
//    	estGains.thetaDes = actx->jointAngleDegrees;
//    }
}

static float biomCalcImpedanceOneState(float k1, float b, float theta_set)
{
	float theta = 0, theta_d = 0;
	float tor_d = 0;

	theta = act1.jointAngleDegrees;
	theta_d = act1.jointVelDegrees;

	if (theta < walkParams.earlyStanceTheta + walkParams.thetaThreshDorsi){
		theta = walkParams.earlyStanceTheta + walkParams.thetaThreshDorsi;
	}
	if (theta > walkParams.earlyStanceTheta + walkParams.thetaThreshPlant){
			theta = walkParams.earlyStanceTheta + walkParams.thetaThreshPlant;
	}

	tor_d = k1 * (theta_set - theta ) - b*theta_d;

	return tor_d;

}

static void updateVirtualHardstopTorque(Act_s *actx, WalkParams *wParams) {

	if (JNT_ORIENT*actx->jointAngleDegrees > wParams->virtualHardstopEngagementAngle) {
		wParams->virtual_hardstop_tq = wParams->virtualHardstopK * ((JNT_ORIENT * actx->jointAngleDegrees) - wParams->virtualHardstopEngagementAngle);
	} else {
		wParams->virtual_hardstop_tq = 0.0;
	}
}


//reset virtual joint to robot joint state
void updatePFDFState(struct act_s *actx) {
	PFDF_state[0] = equilibriumAngle;
	PFDF_state[1] = 0;
	PFDF_state[2] = 0;
}

#endif //BOARD_TYPE_FLEXSEA_MANAGE

#ifdef __cplusplus
}
#endif


