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
LinearSpline linearSpline;
CubicSpline cubicSpline;

// Gain Parameters are modified to match our joint angle convention (RHR for right ankle, wearer's perspective)
GainParams eswGains = {1.5, 0.0, 0.03, -10.0}; // b was .3
GainParams lswGains = {1.5, 1, 0.3, -5.0};
GainParams estGains = {0.0, 0.0, 0.1, 0.0};
GainParams lstGains = {0.0, 0.0, 0.0, 0.0}; //currently unused in simple implementation
GainParams lstPowerGains = {4.5, 0.0, 0.1, 14};

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

static void initializeLinearSplineParams(LinearSpline *lSpline, Act_s *actx, GainParams gainParams);
static void calcLinearSpline(LinearSpline *lSpline, Act_s *actx);
static void initializeCubicSplineParams(CubicSpline *cSpline, Act_s *actx, GainParams gainParams);
static void solveTridiagonalMatrix(CubicSpline *cSpline);
static void calcCubicSpline(CubicSpline *cSpline, Act_s *actx);


/** Impedance Control Level-ground Walking FSM
	Based on BiOM ankle and simplified.
	Finds desired torque.

	@param ptorqueDes pointer to float meant to be updated with desired torque
*/
void runFlatGroundFSM(Act_s *actx) {

    static int8_t isTransitioning = 0;
    static uint32_t time_in_state = 0;
    static int32_t emgInputPPF = 0; //used to keep track of EMG PPF input
    static int8_t passedStanceThresh = 0;

	static int8_t using_EMG_free_space = 0;
	int16_t emgVal = 0;
	float torq_thresh = 0;

    if (!walkParams.initializedStateMachineVariables){
    	initializeUserWrites(&walkParams);

    	//USER WRITE INITIALIZATION GOES HERE//////////////
    	user_data_1.w[2] = PF_TORQUE_GAIN; //pfTorqueGain;
    	user_data_1.w[3] = DF_TORQUE_GAIN; //dfTorqueGain;
    	user_data_1.w[4] = 7000; //ROBOT_K; //
    	user_data_1.w[5] = 200; //ROBOT_B; // damping/100
    	user_data_1.w[6] = 800; //BASELINE_K; // damping/100
    	user_data_1.w[7] = 35;	// torq_thresh/10
    	///////////////////////////////////////////////////
    }

    updateUserWrites(actx, &walkParams); // not implemented

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
				walkParams.virtual_hardstop_tq = 1.0;
				// initialize linear spline params once
				//initializeLinearSplineParams(&linearSpline, actx, eswGains);
				// initialize cubic spline params once
				initializeCubicSplineParams(&cubicSpline, actx, eswGains);
				solveTridiagonalMatrix(&cubicSpline);
			}

			// Linear Spline
			//calcLinearSpline(&linearSpline, actx);
			//eswGains.thetaDes = linearSpline.Y; // new thetaDes after linear spline

			// Cubic Spline
			calcCubicSpline(&cubicSpline, actx);
			eswGains.thetaDes = cubicSpline.Y; //new thetaDes after cubic spline

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

//			if (MIT_EMG_getState() == 1) windowSmoothEMG0(JIM_LG); //emg signal for Jim's LG

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

//				static int8_t using_EMG_free_space = 0;
//				int16_t emgVal = 0;

				if (isTransitioning) {
					walkParams.scaleFactor = 1.0;

					estGains.k1 = walkParams.earlyStanceK0;
					estGains.thetaDes = actx->jointAngleDegrees;

					emgInputPPF = 0;
					passedStanceThresh = 0;
				}


				updateVirtualHardstopTorque(actx, &walkParams);
				updateImpedanceParams(actx, &walkParams);

				actx->tauDes = calcJointTorque(estGains, actx, &walkParams);

				// VECTOR (1): Early Stance -> Free space EMG
				//---------------------- FREE SPACE EMG TRANSITION VECTORS ----------------------//

//				if (MIT_EMG_getState() == 1 && using_EMG_free_space) {
//
////					emgVal = windowSmoothEMG0(JIM_LG); //emg signal for Jim's LG
//
//					//update emgVal for EMG PPF
//
//					//only consider last 500 ms for emgInputPPF
//					if (time_in_state % 500 == 499) {
//						emgInputPPF = 0;
//					}
//					//store max value of EMG during early stance
//					if (emgVal > emgInputPPF) {
//						emgInputPPF = emgVal;
//					}
//
//
////					if(time_in_state > 400 && abs(actx->jointTorque) < 3.5) {
////						stateMachine.current_state = STATE_LSW_EMG;
////					}
//				}

				//Early Stance transition vectors
				// VECTOR (2): Early Stance -> Late Stance POWER!
				//---------------------- EARLY STANCE TRANSITION VECTORS ----------------------//

				// VECTOR (A): Early Stance -> Late Stance (foot flat) - Condition 1
				if (abs(actx->jointTorque) > 20.0){
					passedStanceThresh = 1;
				}

				if (actx->jointTorque > walkParams.lspEngagementTorque) {
		            	stateMachine.current_state = STATE_LATE_STANCE_POWER;      //Transition occurs even if the early swing motion is not finished
		        }

				if (passedStanceThresh && abs(actx->jointTorque) < ANKLE_UNLOADED_TORQUE_THRESH && time_in_state > 100) {
					stateMachine.current_state = STATE_LATE_SWING;
				}

				//------------------------- END OF TRANSITION VECTORS ------------------------//
				break;

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

				if (MIT_EMG_getState() == 1 && using_EMG_PPF) {

					actx->tauDes = calcEMGPPF(actx, &walkParams);

				} else {
					//Linear ramp push off
					actx->tauDes = -1.0*actx->jointTorque + (walkParams.samplesInLSP/walkParams.lstPGDelTics) * calcJointTorque(lstPowerGains, actx, &walkParams);
				}


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
        	torq_thresh = ( (float) user_data_1.w[7] ) / 10.0;

        	//disable hardstop
        	walkParams.virtual_hardstop_tq = 0;

        	//check to make sure EMG is active
        	if (MIT_EMG_getState() == 1) {
//				updateVirtualJoint(&emgFreeGains);	// was moved to main_fsm
				actx->tauDes = calcJointTorque(emgFreeGains, actx, &walkParams);
        	}
        	else {
        		stateMachine.current_state = STATE_EARLY_STANCE;
        	}

        	//Late Swing EMG transition vectors to Early Stance
        	//If activation is below a certain threshold, these become active
        	//Tune thresholds based on user

			// VECTOR (1): Late Swing -> Early Stance (hard heal strike) - Condition 1
			if ( abs(actx->jointTorque) > torq_thresh ) {
				stateMachine.current_state = STATE_EARLY_STANCE;
				walkParams.transition_id = 7;
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
	wParams->virtualHardstopK = 7;				//7 x 100
	wParams->virtualHardstopEngagementAngle = 0;	//0.0 x 1
	wParams->lspEngagementTorque = 40;

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

//
static void initializeLinearSplineParams(LinearSpline *lSpline, Act_s *actx, GainParams gainParams){
	lSpline->time_state = 0;
	lSpline->res_factor = 1.0; // resolution factor TODO: change magic number
	lSpline->xi = -1.0; // to not divide by zero
	lSpline->xf = linearSpline.res_factor;
	lSpline->theta_set_fsm = gainParams.thetaDes;
	lSpline->yi = actx->jointAngleDegrees;
	lSpline->yf = lSpline->theta_set_fsm;
}

static void calcLinearSpline(LinearSpline *lSpline, Act_s *actx) {
	lSpline->yi = actx->jointAngleDegrees;
	lSpline->Y = (((lSpline->yf - lSpline->yi) * ((float)lSpline->time_state - lSpline->xi)) / (lSpline->xf - lSpline->xi)) + lSpline->yi;
	lSpline->time_state++;
	// Condition to reset time_state - TODO: Correct?
	if (lSpline->Y <= (lSpline->theta_set_fsm + 3.0)){
		lSpline->time_state = 0;
		lSpline->Y = lSpline->theta_set_fsm;
	}
}

static void initializeCubicSplineParams(CubicSpline *cSpline, Act_s *actx, GainParams gainParams){
	cSpline->time_state = 0;
	cSpline->theta_set_fsm = gainParams.thetaDes;
	cSpline->theta_set_fsm_int = actx->jointAngleDegrees-((actx->jointAngleDegrees-cSpline->theta_set_fsm)/5.0); // TODO: magic number (4)
	cSpline->res_factor = 90.0; // TODO: magic number
	cSpline->x_int = cSpline->res_factor * 0.5; // TODO: magic number (0.7)
	cSpline->y_int = cSpline->theta_set_fsm_int;
	cSpline->xi = 0.0;
	cSpline->yi = actx->jointAngleDegrees; //joint_angle
	cSpline->xf = cSpline->res_factor;
	cSpline->yf = cSpline->theta_set_fsm;
}

static void solveTridiagonalMatrix(CubicSpline *cSpline){
	float B[3], A[2], C[2], r[3];
	float x[3];
	float y[3];
	x[0] = cSpline->xi;
	x[1] = cSpline->x_int;
	x[2] = cSpline->xf;
	y[0] = cSpline->yi;
	y[1] = cSpline->y_int;
	y[2] = cSpline->yf;

	B[0] = 2.0 / (x[1] - x[0]);
	B[1] = 2.0 * ((1/(x[1]-x[0])) + (1/(x[2]-x[1])));
	B[2] = 2.0 / (x[2]-x[1]);
	A[0] = 1.0 / (x[1]-x[0]);
	A[1] = 1.0 / (x[2]-x[1]);
	C[0] = 1.0 / (x[1]-x[0]);
	C[1] = 1.0 / (x[2]-x[1]);
	r[0] = 3.0 * ((y[1]-y[0])/(pow(x[1]-x[0],2)));
	r[1] = 3.0 * (((y[1]-y[0])/(pow(x[1]-x[0],2))) + ((y[2]-y[1])/(pow(x[2]-x[1],2))));
	r[2] = 3.0 * ((y[2]-y[1])/(pow(x[2]-x[1],2)));

	float e[3], f[3], g[2];
	e[0] = 0;
	e[1] = A[0];
	e[2] = A[1];
	for(int i=0; i<3; i++){
		f[i] = B[i];
	}
	g[0] = C[0];
	g[1] = C[1];

	int n = 3; // f vector length
	// Forward elimination
	float factor;
	for(int i = 1; i < n; i++){
		factor = e[i] / f[i-1];
		f[i] = f[i] - (factor * g[i-1]);
		r[i] = r[i] - (factor * r[i-1]);
	}
	// Back substitution
	float k[3];
	k[n-1] = r[n-1] / f[n-1];
	for(int i = 1; i >= 0; i--){
		k[i] = (r[i] - (g[i] * k[i+1])) / f[i];
	}

	// ai and bi computation
	float a1, a2, b1, b2;
	a1 = k[0]*(x[1]-x[0]) - (y[1]-y[0]);
	a2 = k[1]*(x[2]-x[1]) - (y[2]-y[1]);
	b1 = -1.0*k[1]*(x[1]-x[0]) + (y[1]-y[0]);
	b2 = -1.0*k[2]*(x[2]-x[1]) + (y[2]-y[1]);
	cSpline->a1 = a1;
	cSpline->a2 = a2;
	cSpline->b1 = b1;
	cSpline->b2 = b2;
}

static void calcCubicSpline(CubicSpline *cSpline, Act_s *actx){
	float t;
	float q[2];
	float x[3];
	float y[3];
	x[0] = cSpline->xi;
	x[1] = cSpline->x_int;
	x[2] = cSpline->xf;
	y[0] = cSpline->yi;
	y[1] = cSpline->y_int;
	y[2] = cSpline->yf;

	t = (cubicSpline.time_state - x[0]) / (x[1]-x[0]);
	q[0] = (1-t)*y[0] + t*y[1] + (t*(1-t)*(cSpline->a1*(1-t)+(cSpline->b1*t)));

	t = (cubicSpline.time_state - x[1]) / (x[2]-x[1]);
	q[1] = (1-t)*y[1] + t*y[2] + (t*(1-t)*(cSpline->a2*(1-t)+(cSpline->b2*t)));

	if (actx->jointAngleDegrees > cSpline->theta_set_fsm_int)
		cSpline->Y = q[0];
	else cSpline->Y = q[1];

	cSpline->time_state++;

	// Condition to reset time_state. TODO: Correct?
	if (cSpline->Y <= (cSpline->theta_set_fsm + 3.0)){
		//cSpline->time_state = 0; // Not sure about this
		cSpline->Y = cSpline->theta_set_fsm;
	}

}

#endif //BOARD_TYPE_FLEXSEA_MANAGE

#ifdef __cplusplus
}
#endif


