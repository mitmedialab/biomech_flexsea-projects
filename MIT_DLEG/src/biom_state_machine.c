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
//LinearSpline linearSpline;

// Gain Parameters are modified to match our joint angle convention (RHR for right ankle, wearer's perspective)
GainParams eswGains = {1.5, 0.0, 0.3, -10.0};
GainParams lswGains = {1.5, 1, 0.3, -5.0};
GainParams estGains = {0.0, 0.0, 0.1, 0.0};
GainParams lstGains = {0.0, 0.0, 0.0, 0.0}; //currently unused in simple implementation
GainParams lstPowerGains = {4.5, 0.0, 0.1, 10};

GainParams emgStandGains = {2, 0.025, 0.04, 0};
GainParams emgFreeGains  = {1.2, 0, 0.02, 0};


#ifdef BOARD_TYPE_FLEXSEA_MANAGE

//****************************************************************************
// Static Functions:
//****************************************************************************

static void updatePFDFState(Act_s *actx);

static void updateImpedanceParams(Act_s *actx, WalkParams *wParams);
static void updateUserWrites(Act_s *actx, WalkParams *wParams);
static float calcJointTorque(GainParams gainParams, Act_s *actx, WalkParams *wParams);
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
    static int32_t emgInputPPF = 0; //used to keep track of EMG PPF input
    //static float spline_index = 0; // used for linear spline


    if (!walkParams.initializedStateMachineVariables){
    	initializeUserWrites(&walkParams);

    	//USER WRITE INITIALIZATION GOES HERE//////////////
//    	user_data_1.w[2] = ; // k term/100
//    	user_data_1.w[3] = ; // theta desired/10
//    	user_data_1.w[4] = ; // damping/100
    	///////////////////////////////////////////////////
    }

    updateUserWrites(actx, &walkParams);

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

            stateMachine.current_state = STATE_LATE_SWING;

            break;
					
        case STATE_EARLY_SWING: //2

            //Put anything you want to run ONCE during state entry.
			if (isTransitioning) {
				walkParams.virtual_hardstop_tq = 0.0;
				// initialize linearSpline params once
				//linearSpline.res_factor = 1.0; // to change interpolation resolution
				//linearSpline.res_size = (linearSpline.xf - linearSpline.xi) / linearSpline.res_factor;
				//linearSpline.xi = (float)time_in_state; // cast?
				//linearSpline.yi = actx->jointAngleDegrees;
				//linearSpline.xf = (float)time_in_state + linearSpline.res_size; // Not sure. cast?
				//linearSpline.yf = eswGains.thetaDes;
			}

			// TODO: linear spline in function. end condition?
			//spline_index = linearSpline.xi + linearSpline.res_factor;
			//linearSpline.X = spline_index;
			//linearSpline.Y = (((linearSpline.yf - linearSpline.yi) * (spline_index - linearSpline.xi)) / (linearSpline.xf - linearSpline.xi)) + linearSpline.yi;
			//spline_index = spline_index + linearSpline.res_factor; // cast?

			// Instead of fixed gains, spline
			//eswGains.thetaDes = linearSpline.Y; // gains need to be modified? - k1, b
            actx->tauDes = calcJointTorque(eswGains, actx, &walkParams);

            //Early Swing transition vectors
            // VECTOR(1): Early Swing -> Late Swing
            if (time_in_state >= ESW_TO_LSW_DELAY) { // more time needed?
                stateMachine.current_state = STATE_LATE_SWING;      //Transition occurs even if the early swing motion is not finished
            }

            //run any exit code here

            break; // case STATE_EARLY_SWING

        case STATE_LATE_SWING: //3

			if (isTransitioning) {
				walkParams.transition_id = 0;

			}

			actx->tauDes = calcJointTorque(lswGains, actx, &walkParams);

			//if (MIT_EMG_getState() == 1) windowSmoothEMG0(JIM_LG); //emg signal for Jim's LG

			//---------------------- LATE SWING TRANSITION VECTORS ----------------------//
			if(time_in_state > ESW_TO_LSW_DELAY) {

				// VECTOR (1): Late Swing -> Early Stance (hard heal strike) - Condition 1
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

        	{
				static int8_t using_EMG_free_space = 0;
				int16_t emgVal = 0;

				if (isTransitioning) {
					walkParams.scaleFactor = 1.0;

					estGains.k1 = walkParams.earlyStanceK0;
					estGains.thetaDes = actx->jointAngleDegrees;

					emgInputPPF = 0;
				}

				//update emgVal for EMG PPF
			/*	if (MIT_EMG_getState() == 1) {

					emgVal = windowSmoothEMG0(JIM_LG); //emg signal for Jim's LG

					//only consider last 500 ms for emgInputPPF
					if (time_in_state % 500 == 499) {
						emgInputPPF = 0;
					}
					//store max value of EMG during early stance
					if (emgVal > emgInputPPF) {
						emgInputPPF = emgVal;
					}

				}*/


				updateVirtualHardstopTorque(actx, &walkParams);
				updateImpedanceParams(actx, &walkParams);

				actx->tauDes = calcJointTorque(estGains, actx, &walkParams);

				// VECTOR (1): Early Stance -> Free space EMG
				//---------------------- FREE SPACE EMG TRANSITION VECTORS ----------------------//

				/*if (MIT_EMG_getState() == 1 && using_EMG_free_space) {

					if(time_in_state > 400 && abs(actx->jointTorque) < 3.5) {
						stateMachine.current_state = STATE_LSW_EMG;
					}
				}*/

				//Early Stance transition vectors
				// VECTOR (2): Early Stance -> Late Stance POWER!
				//---------------------- EARLY STANCE TRANSITION VECTORS ----------------------//

				// VECTOR (A): Early Stance -> Late Stance (foot flat) - Condition 1
				if (actx->jointTorque > walkParams.lspEngagementTorque) {
		            	stateMachine.current_state = STATE_LATE_STANCE_POWER;      //Transition occurs even if the early swing motion is not finished
		        }

				//------------------------- END OF TRANSITION VECTORS ------------------------//
				break;
        	}

//        case STATE_LATE_STANCE: //5
//
//            if (isTransitioning) {
////               lstGains.k1 = walkParams.earlyStanceKF;
////               lstGains.thetaDes = actx->jointAngleDegrees;
//            }
//
//            updateVirtualHardstopTorque(actx, &walkParams);
//            updateImpedanceParams(actx, &walkParams);
//            actx->tauDes = calcJointTorque(estGains, actx, &walkParams);
//
//            //---------------------- LATE STANCE TRANSITION VECTORS ----------------------//
//
//            // VECTOR (1): Late Stance -> Late Stance POWER
//            if (actx->jointTorque > walkParams.lspEngagementTorque) {
//            	stateMachine.current_state = STATE_LATE_STANCE_POWER;      //Transition occurs even if the early swing motion is not finished
//            }
//
//            //------------------------- END OF TRANSITION VECTORS ------------------------//
//            break;

        case STATE_LATE_STANCE_POWER: //6

        	{
				//turn this on or off to use EMG powered plantarflexion
				static int8_t using_EMG_PPF = 0;

				if (isTransitioning) {
					walkParams.samplesInLSP = 0.0;
					walkParams.lsp_entry_tq = actx->jointTorque;
//	                walkParams.virtual_hardstop_tq = 0.0;
//	                walkParams.pff_lumped_gain = walkParams.pff_gain * powf(1.0/(PCI - walkParams.lsp_entry_tq), walkParams.pff_exponent);
				}

				if (walkParams.samplesInLSP < walkParams.lstPGDelTics){
					walkParams.samplesInLSP++;
				}

				updateVirtualHardstopTorque(actx, &walkParams);

				/*if (MIT_EMG_getState() == 1 && using_EMG_PPF) {

					actx->tauDes = calcEMGPPF(actx, &walkParams);

				} else {
					//Linear ramp push off
					actx->tauDes = -1.0*actx->jointTorque + (walkParams.samplesInLSP/walkParams.lstPGDelTics) * calcJointTorque(, actx, &walkParams);
				}*/
				actx->tauDes = calcJointTorque(lstPowerGains, actx, &walkParams);

				//Late Stance Power transition vectors
				// VECTOR (1): Late Stance Power -> Early Swing - Condition 1
				if (abs(actx->jointTorque) < ANKLE_UNLOADED_TORQUE_THRESH && time_in_state > 100) {
					stateMachine.current_state = STATE_EARLY_SWING;
				}
        	}

            break;

        case STATE_EMG_STAND_ON_TOE:
        	//untested and unused. If would like to use, need to check implementation on ts_projects

        	//disable virtual hardstop while in EMG mode
        	walkParams.virtual_hardstop_tq = 0;

        	if (MIT_EMG_getState() == 1) {

        		//this function can set the current_state
				updateStandJoint(&emgStandGains);

				//if we're still in EMG control even after updating EMG control vars
				if (stateMachine.current_state == STATE_EMG_STAND_ON_TOE) {
					actx->tauDes = calcJointTorque(emgStandGains, actx, &walkParams);

				//actually in early stance (explicitly declared)
				} else if (stateMachine.current_state == STATE_EARLY_STANCE) {
					actx->tauDes = calcJointTorque(estGains, actx, &walkParams);
				}

			} else {
				stateMachine.current_state = STATE_EARLY_STANCE;
				reset_EMG_stand(actx->jointAngleDegrees);
				actx->tauDes = calcJointTorque(estGains, actx, &walkParams); //do early stance control if EMG disconnected
			}

            break;
		
        case STATE_LSW_EMG:
        	//upon entering, make sure virtual joint and robot joint match
        	if (isTransitioning) {
        		updatePFDFState(actx);
        	}

        	//disable hardstop
        	walkParams.virtual_hardstop_tq = 0;

        	//check to make sure EMG is active
        	if (MIT_EMG_getState() == 1) {
				updateVirtualJoint(&emgFreeGains);
				actx->tauDes = calcJointTorque(emgFreeGains, actx, &walkParams);
        	}
        	else {
        		stateMachine.current_state = STATE_EARLY_STANCE;
        	}

        	//Late Swing EMG transition vectors to Early Stance
        	//If activation is below a certain threshold, these become active
        	//Tune thresholds based on user

			// VECTOR (1): Late Swing -> Early Stance (hard heal strike) - Condition 1
			if (actx->jointTorque > 3.5) {
				stateMachine.current_state = STATE_EARLY_STANCE;
				walkParams.transition_id = 1;
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
static float calcJointTorque(GainParams gainParams, Act_s *actx, WalkParams *wParams) {

//	if (fabs(actx->jointVelDegrees) > 6){
		return gainParams.k1 * (gainParams.thetaDes - actx->jointAngleDegrees) \
         - gainParams.b * actx->jointVelDegrees  + wParams->virtual_hardstop_tq;
//	}else{
//    	return gainParams.k1 * (gainParams.thetaDes - actx->jointAngleDegrees) + wParams->virtual_hardstop_tq;
//	}
}

static void updateUserWrites(Act_s *actx, WalkParams *wParams){

}

static void initializeUserWrites(WalkParams *wParams){

	wParams->earlyStanceK0 = 6.23;
	wParams->earlyStanceKF = 0.1;
	wParams->earlyStanceDecayConstant = EARLYSTANCE_DECAY_CONSTANT;
	wParams->lstPGDelTics = 1;
	wParams->earlyStanceKF = 1;
	wParams->virtualHardstopK = 5;				//7 x 100
	wParams->virtualHardstopEngagementAngle = 0;	//0.0 x 1
	wParams->lspEngagementTorque = 50;

	wParams->initializedStateMachineVariables = 1;
}

static void updateImpedanceParams(Act_s *actx, WalkParams *wParams) {

	wParams->scaleFactor = wParams->scaleFactor * wParams->earlyStanceDecayConstant;

    estGains.k1 = wParams->earlyStanceKF + wParams->scaleFactor * (wParams->earlyStanceK0 - wParams->earlyStanceKF);

    if (actx->jointAngleDegrees < estGains.thetaDes) {
    	estGains.thetaDes = actx->jointAngleDegrees;
    }
}


static void updateVirtualHardstopTorque(Act_s *actx, WalkParams *wParams) {

	if (JNT_ORIENT*actx->jointAngleDegrees > wParams->virtualHardstopEngagementAngle) {
		wParams->virtual_hardstop_tq = wParams->virtualHardstopK * ((JNT_ORIENT * actx->jointAngleDegrees) - wParams->virtualHardstopEngagementAngle);
	} else {
		wParams->virtual_hardstop_tq = 0.0;
	}
}

float calcEMGPPF(Act_s *actx, WalkParams *wParam) {

	int16_t EMGin_LG;
	float scaledEMG;
	float impedanceContribution;

	float k = 14; //scalar term for impedance control
	float b = 0.3;
	float thetaDes = 20;
	float desiredTorqueThreshold = 150; //max desired torque


	//limit maximum emg_data in case something goes wrong (to 20000)
	for (uint8_t i = 0; i < (sizeof(emg_data)/sizeof(emg_data[0])); i++) {
		if (emg_data[i] > emgInMax) {
			emg_data[i] = emgInMax;
		}
	}

	//5ms moving average for gastroc only
	EMGin_LG = JIM_LG; //SEONGS BOARD LG_VAR gastroc, 0-20000. Changed channel to match Jim's gastroc.
	scaledEMG = EMGin_LG/emgInMax;

//	rigid1.mn.genVar[4] = scaledEMG;

	//torque output from the intrinsic controller
	impedanceContribution = scaledEMG*user_data_1.w[0]*(user_data_1.w[2]/10. - actx->jointAngleDegrees) - user_data_1.w[1]/100.*actx->jointVelDegrees + wParam->virtual_hardstop_tq;

	rigid1.mn.genVar[0] = impedanceContribution;
	//saturation of desired output torque
	if (impedanceContribution > desiredTorqueThreshold ) {
		return desiredTorqueThreshold;
	//safety in case of tripping
	} else if (impedanceContribution < 0) {
		return calcJointTorque(lstPowerGains, actx, &walkParams);
	//everything nominal. return combined contributions
	} else {
		return impedanceContribution;
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


