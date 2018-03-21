#ifdef __cplusplus
extern "C" {
#endif

#include "state_variables.h"
//#include "state_machine.h"
#include "biom_state_machine.h"
#include "user-mn-MIT-DLeg-2dof.h"
#include "user-mn-MIT-EMG.h"
#include "flexsea_user_structs.h"
#include "MIT_Ankle_EMG.h"
#include "cmd-DLeg.h"
#include "flexsea_system.h"
#include "flexsea.h"
#include <math.h>

WalkingStateMachine stateMachine;
Act_s act1;
// Gain Parameters are modified to match our joint angle convention (RHR for right ankle, wearer's perspective)
GainParams eswGains = {0.134, 0, 0.001, 0};	// goldfarb setpt = 23
GainParams lswGains = {0.134, 0, 0.002, 0}; // goldfarb setpt = 2
GainParams estGains = {0, 0, 0, 0};
GainParams lstGains = {0, 0, 0, JNT_ORIENT * 0}; //currently unused in simple implementation

GainParams lstPowerGains = {4.5, 0, 0.005, JNT_ORIENT * -18};
GainParams emgStandGains = {0, 0, 0, 0}; //currently unused
GainParams emgFreeGains = {2, 0, 0.005, 0};

#ifndef BOARD_TYPE_FLEXSEA_PLAN

//Static functions
static float calcJointTorque(GainParams gainParams);
static void updatePFDFState(void);

/** Impedance Control Level-ground Walking FSM
	Based on BiOM ankle and simplified.
	Finds desired torque.

	@param ptorqueDes pointer to float meant to be updated with desired torque
*/
void runFlatGroundFSM(struct act_s *actx) {

    static int8_t isTransitioning = 0;
    static uint32_t time_in_state = 0;
	
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

        //update Plan with state changes
//        static uint8_t info[2] = {PORT_USB, PORT_USB};
//        tx_cmd_dleg_w(TX_N_DEFAULT, stateMachine.current_state);
//        packAndSend(P_AND_S_DEFAULT, FLEXSEA_PLAN_1, info, SEND_TO_MASTER); //3rd arg unused
    }

    switch (stateMachine.current_state) {

        case STATE_IDLE:
            //error handling here (should never be in STATE_IDLE by the time you get here)
            break;

        case STATE_INIT:

            stateMachine.current_state = STATE_LATE_SWING;

            break;
					
        case STATE_EARLY_SWING:
            //Put anything you want to run ONCE during state entry.
//			if (isTransitioning) {
//
//			}

            actx->tauDes = calcJointTorque(eswGains);

            //Early Swing transition vectors
            // VECTOR(1): Early Swing -> Late Swing
            if (time_in_state >= ESW_TO_LSW_DELAY) {
                stateMachine.current_state = STATE_LATE_SWING;      //Transition occurs even if the early swing motion is not finished
            }

            //run any exit code here

            break; // case STATE_EARLY_SWING

        case STATE_LATE_SWING:

            actx->tauDes = calcJointTorque(lswGains);

            //Late Swing transition vectors
            // VECTOR (1): Late Swing -> Early Stance (hard heel strike)
            //toDo: better transition criterion than this
            if (actx->jointTorque < HARD_HEELSTRIKE_TORQUE_THRESH) {
                stateMachine.current_state = STATE_EARLY_STANCE;
            }

            break;

        case STATE_EARLY_STANCE:
          if (isTransitioning) {
        	  actx->scaleFactor = 1.0;
        	  estGains.k1 = K_ES_INITIAL_NM_P_DEG;
                estGains.thetaDes = actx->jointAngleDegrees;
            }


            updateImpedanceParams();

            actx->tauDes = calcJointTorque(estGains);

            //Early Stance transition vectors
            // VECTOR (1): Early Stance -> Late Stance POWER!
            //toDo counterclockwise is positive?
//            if (actx->jointTorque < LSTPWR_HS_TORQ_TRIGGER_THRESH) {  // real one.
//			if (actx->jointTorque < HARD_HEELSTRIKE_TORQUE_THRESH) {	// for testing
//
////                stateMachine.current_state = STATE_LATE_STANCE_POWER;  // real one
//				stateMachine.current_state = STATE_EARLY_SWING;		// for testing
//            }

            if (time_in_state >= EST_TO_ESW_DELAY) {
            	stateMachine.current_state = STATE_EARLY_SWING;      //Transition occurs even if the early swing motion is not finished
            }
		
            break;
		
        case STATE_LATE_STANCE_POWER:
            if (isTransitioning) {
                actx->samplesInLSP = 0.0;
                actx->pff_lumped_gain = act1.pff_gain * powf(1.0/(PCI - LSTPWR_HS_TORQ_TRIGGER_THRESH), actx->pff_exponent);
            }
            //actx->tauDes = updatePffTq();

            //Late Stance Power transition vectors
            // VECTOR (1): Late Stance Power -> Early Swing - Condition 1
            if (abs(actx->jointTorque) < ANKLE_UNLOADED_TORQUE_THRESH) {
                stateMachine.current_state = STATE_EARLY_SWING;

            }

            break;

        case STATE_EMG_STAND_ON_TOE:
            //toDo with EMG

            break;
		
        case STATE_LSW_EMG:
        	//upon entering, make sure virtual joint and robot joint match
        	if (isTransitioning) {
        		updatePFDFState();
        	}

        	//check to make sure EMG is active
        	if (MIT_EMG_getState() == 1) {
				updateVirtualJoint(&emgFreeGains);
				actx->tauDes = calcJointTorque(emgFreeGains);
        	} else {
        		//reset and command 0 torque
        		actx->tauDes = 0;
        		updatePFDFState();
        	}

        	//toDo: Late Swing EMG transition vectors to Early Stance HOW?! Perhaps load cell

        	break;
		
        default:

            //turn off control.
            actx->tauDes = 0;
            stateMachine.current_state = STATE_LATE_SWING;

            break;
	
    }

    //update last state in preparation for next loop
    stateMachine.last_sm_state = stateMachine.on_entry_sm_state;

    rigid1.mn.genVar[6] =  (int16_t) ( estGains.k1 * 1000.0 );
    rigid1.mn.genVar[7] = (int16_t) (estGains.thetaDes * 1000.0);
    rigid1.mn.genVar[8] = stateMachine.current_state;

}

/** Impedance Control Torque
	Calculates desired torque based on impedance gain parameters

	@param  gainParams struct with all the state's impedance parameters
    @return float desired torque at joint (Nm)
*/
static float calcJointTorque(GainParams gainParams) {

	//if (fabs(act1.jointAngleDegrees) > 5){
		return gainParams.k1 * (gainParams.thetaDes - act1.jointAngleDegrees) \
         + gainParams.b * act1.jointVelDegrees;
//	}else{
//    	return gainParams.k1 * (gainParams.thetaDes - act1.jointAngleDegrees) \
//    	         + gainParams.k2 * powf((gainParams.thetaDes - act1.jointAngleDegrees), 3) - gainParams.b * act1.jointVelDegrees;
//	}
}

static void updateImpedanceParams(void) {
    act1.scaleFactor = act1.scaleFactor*EARLYSTANCE_DECAY_CONSTANT;
    estGains.k1 = K_ES_FINAL_NM_P_DEG + act1.scaleFactor * DELTA_K_DEG;
    if (act1.jointVelDegrees < -10.0){
        estGains.thetaDes = act1.jointAngleDegrees;
    }
}

static float updatePffTorque(void){

    if (act1.samplesInLSP < PFF_DELAY_SAMPLES){
        act1.samplesInLSP = act1.samplesInLSP + 1.0;
    }

    return  (act1.samplesInLSP/PFF_DELAY_SAMPLES) * act1.pff_lumped_gain * powf(act1.jointTorque - LSTPWR_HS_TORQ_TRIGGER_THRESH, act1.pff_exponent);
}

//reset virtual joint to robot joint state
static void updatePFDFState(void) {
	PFDF_state[0] = act1.jointAngleDegrees;
	PFDF_state[1] = act1.jointVelDegrees;
	PFDF_state[2] = 0;
}

#endif //BOARD_TYPE_FLEXSEA_PLAN

#ifdef __cplusplus
}
#endif


