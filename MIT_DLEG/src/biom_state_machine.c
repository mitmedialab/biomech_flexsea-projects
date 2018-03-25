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
GainParams eswGains = {0.134, 0, 0.001, -15};	// goldfarb setpt = 23
GainParams lswGains = {0.134, 0, 0.02, 0}; // goldfarb setpt = 2
GainParams estGains = {0, 0, B_ES_NM_S_P_DEG, 0};
GainParams lstGains = {0, 0, 0, 0}; //currently unused in simple implementation

GainParams lstPowerGains = {4.5, 0, 0.1, JNT_ORIENT * -15};
GainParams emgStandGains = {0, 0, 0, 0}; //currently unused
GainParams emgFreeGains = {2, 0, 0.005, 0};

float lspEngAng = 10.0;
float lstPGK1 = 4.5;
float lstPGTheta =  JNT_ORIENT * -18;
float lstPGDelTics = 1;

#ifndef BOARD_TYPE_FLEXSEA_PLAN

//Static functions


/** Impedance Control Level-ground Walking FSM
	Based on BiOM ankle and simplified.
	Finds desired torque.

	@param ptorqueDes pointer to float meant to be updated with desired torque
*/
void runFlatGroundFSM(struct act_s *actx) {

    static int8_t isTransitioning = 0;
    static uint32_t time_in_state = 0;

    //vars to edit from GUI !!MUST INITIALIZE IN GUI!!				// Starting Points
   /* lspEngAng = ((float) user_data_1.w[0])/10.0;					// 100, late stance engagement angle
    lstPGK1 = ((float) user_data_1.w[1])/10.0;						// 60,  late stance Power Gain K1
    lstPGTheta =  ((float) user_data_1.w[2])/10.0;					// 130, late stance desired set point.
    lstPGDelTics = ((float) user_data_1.w[3]) + 1;					// 50,  late stance Power Gain ramp value
    lstPowerGains.k1 = lstPGK1;
    lstPowerGains.thetaDes = lstPGTheta;*/

    lstPGDelTics = 50;


    rigid1.mn.genVar[8] = lstPGDelTics;

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
			if (isTransitioning) {
				actx->virtual_hardstop_tq = 0.0;
			}

            actx->tauDes = calcJointTorque(eswGains, actx);

            //Early Swing transition vectors
            // VECTOR(1): Early Swing -> Late Swing
            if (time_in_state >= ESW_TO_LSW_DELAY) {
                stateMachine.current_state = STATE_LATE_SWING;      //Transition occurs even if the early swing motion is not finished
            }

            //run any exit code here

            break; // case STATE_EARLY_SWING

        case STATE_LATE_SWING:

            updateVirtualHardstopTorque(actx);
            actx->tauDes = calcJointTorque(lswGains, actx);

            //---------------------- LATE SWING TRANSITION VECTORS ----------------------//

            // VECTOR (1): Late Swing -> Early Stance (hard heal strike) - Condition 1
            if (actx->jointTorque > HARD_HEELSTRIKE_TORQUE_THRESH && actx->jointTorqueRate > HARD_HEELSTRIKE_TORQ_RATE_THRESH) {
                stateMachine.current_state = STATE_EARLY_STANCE;
            }
            // VECTOR (1): Late Swing -> Early Stance (gentle heal strike) - Condition 2 -
            else if(actx->jointTorqueRate > GENTLE_HEALSTRIKE_TORQ_RATE_THRESH){
                stateMachine.current_state = STATE_EARLY_STANCE;
            }
            // VECTOR (1): Late Swing -> Early Stance (toe strike) - Condition 3
            else if(actx->jointAngleDegrees < HARD_TOESTRIKE_ANGLE_THRESH){
                stateMachine.current_state = STATE_EARLY_STANCE;
            }

            //------------------------- END OF TRANSITION VECTORS ------------------------//
            break;

        case STATE_EARLY_STANCE:
          if (isTransitioning) {
                actx->scaleFactor = 1.0;
                estGains.k1 = K_ES_INITIAL_NM_P_DEG;
                estGains.thetaDes = actx->jointAngleDegrees;
            }

          	updateVirtualHardstopTorque(actx);
            updateImpedanceParams(actx);

            actx->tauDes = calcJointTorque(estGains, actx);

            //Early Stance transition vectors
            // VECTOR (1): Early Stance -> Late Stance POWER!
            //toDo counterclockwise is positive?
//            if (actx->jointTorque < LSTPWR_HS_TORQ_TRIGGER_THRESH) {  // real one.
//			if (actx->jointTorque < HARD_HEELSTRIKE_TORQUE_THRESH) {	// for testing
//
////                stateMachine.current_state = STATE_LATE_STANCE_POWER;  // real one
//				stateMachine.current_state = STATE_EARLY_SWING;		// for testing
//            }

//              if (time_in_state > EST_TO_ESW_DELAY){
//            	  stateMachine.current_state = STATE_EARLY_SWING;
//              }

            //---------------------- EARLY STANCE TRANSITION VECTORS ----------------------//

            // VECTOR (A): Early Stance -> Late Stance (foot flat) - Condition 1
            if (actx->jointAngleDegrees < EST_TO_LST_FOOT_FLAT_HS_ANGLE_LIMIT && actx->jointTorqueRate < EST_TO_LST_FOOT_FLAT_TORQ_RATE) {
            	stateMachine.current_state = STATE_LATE_STANCE;      //Transition occurs even if the early swing motion is not finished
            }
            // VECTOR (A): Early Stance -> Late Stance (toe strike) - Condition 2
            else if(actx->jointAngleDegrees < HARD_TOESTRIKE_ANGLE_THRESH){
                stateMachine.current_state = STATE_LATE_STANCE;
            }

            //------------------------- END OF TRANSITION VECTORS ------------------------//        
            break;

        case STATE_LATE_STANCE:
            if (isTransitioning) {
                lstGains.k1 = K_ES_FINAL_NM_P_DEG;
                lstGains.thetaDes = actx->jointAngleDegrees;
            }
            updateVirtualHardstopTorque(actx);
            actx->tauDes = calcJointTorque(lstGains, actx);

            //---------------------- LATE STANCE TRANSITION VECTORS ----------------------//

            // VECTOR (1): Late Stance -> Late Stance POWER
            if (actx->jointAngleDegrees < LSTPWR_HS_TORQ_TRIGGER_THRESH ) {
                stateMachine.current_state = STATE_LATE_STANCE_POWER;      //Transition occurs even if the early swing motion is not finished
            }

            //------------------------- END OF TRANSITION VECTORS ------------------------//     
            break;

        case STATE_LATE_STANCE_POWER:
            if (isTransitioning) {
                actx->samplesInLSP = 0.0;
                //actx->lsp_entry_tq = actx->jointTorque;
               // actx->pff_lumped_gain = actx->pff_gain * powf(1.0/(PCI - actx->lsp_entry_tq), actx->pff_exponent);
            }
            if (actx->samplesInLSP < lstPGDelTics){
            	actx->samplesInLSP = actx->samplesInLSP + 1.0;
            }
            updateVirtualHardstopTorque(actx);
            actx->tauDes = (actx->samplesInLSP/lstPGDelTics) * calcJointTorque(lstPowerGains, actx);

            //Late Stance Power transition vectors
            // VECTOR (1): Late Stance Power -> Early Swing - Condition 1
            //updateVirtualHardstopTorque(actx);


            if (abs(actx->jointTorque) < ANKLE_UNLOADED_TORQUE_THRESH && time_in_state > 200) {
                stateMachine.current_state = STATE_EARLY_SWING;

            }

            break;

        case STATE_EMG_STAND_ON_TOE:
            //toDo with EMG

            break;
		
        case STATE_LSW_EMG:
        	//upon entering, make sure virtual joint and robot joint match
        	if (isTransitioning) {
        		updatePFDFState(actx);
        	}

        	//check to make sure EMG is active
        	if (MIT_EMG_getState() == 1) {
				updateVirtualJoint(&emgFreeGains);
				actx->tauDes = calcJointTorque(emgFreeGains, actx);
        	} else {
        		//reset and command 0 torque
        		actx->tauDes = 0;
        		updatePFDFState(actx);
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
//    rigid1.mn.genVar[8] = stateMachine.current_state;

}

/** Impedance Control Torque
	Calculates desired torque based on impedance gain parameters

	@param  gainParams struct with all the state's impedance parameters
    @return float desired torque at joint (Nm)
*/
static float calcJointTorque(GainParams gainParams, struct act_s *actx) {

	if (fabs(actx->jointAngleDegrees) > 5){
		return gainParams.k1 * (gainParams.thetaDes - actx->jointAngleDegrees) \
         - gainParams.b * actx->jointVelDegrees  + actx->virtual_hardstop_tq;
	}else{
    	return gainParams.k1 * (gainParams.thetaDes - actx->jointAngleDegrees) + actx->virtual_hardstop_tq;
	}
}

static void updateImpedanceParams(struct act_s *actx) {
    actx->scaleFactor = actx->scaleFactor*EARLYSTANCE_DECAY_CONSTANT;
    estGains.k1 = K_ES_FINAL_NM_P_DEG + actx->scaleFactor * DELTA_K_DEG;
    if (actx->jointVelDegrees < 0.0){
        estGains.thetaDes = actx->jointAngleDegrees;
    }
}

static float updatePffTorque(struct act_s *actx){

    if (actx->samplesInLSP < PFF_DELAY_SAMPLES){
        actx->samplesInLSP = actx->samplesInLSP + 1.0;
    }

    return  (actx->samplesInLSP/PFF_DELAY_SAMPLES) * actx->pff_lumped_gain * powf(actx->jointTorque - LSTPWR_HS_TORQ_TRIGGER_THRESH, actx->pff_exponent);
}

//reset virtual joint to robot joint state
static void updatePFDFState(struct act_s *actx) {
	PFDF_state[0] = actx->jointAngleDegrees;
	PFDF_state[1] = actx->jointVelDegrees;
	PFDF_state[2] = 0;
}

static void updateVirtualHardstopTorque(struct act_s *actx){
	if (JNT_ORIENT * actx->jointAngleDegrees > ANGLE_VIRTUAL_HARDSTOP_NM_P_DEG){
		actx->virtual_hardstop_tq = K_VIRTUAL_HARDSTOP_NM_P_DEG * ((JNT_ORIENT * actx->jointAngleDegrees) - ANGLE_VIRTUAL_HARDSTOP_NM_P_DEG);
	}
	else{
		actx->virtual_hardstop_tq = 0.0;
	}
}

#endif //BOARD_TYPE_FLEXSEA_PLAN

#ifdef __cplusplus
}
#endif


