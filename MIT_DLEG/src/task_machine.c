 

#include "task_machine.h"

void init_task_machine(void){



    // LEVEL GROUND WALKING CONTROLLER PARAMETERS
    // walking_param.subject_weight_centi_lbs = 20500; //WEIGHT X100
    // walking_param.early_stance_imp_k = 300; //STIFFNESS (300 TO 1300)
    // walking_param.early_stance_imp_b = 20; //STANCE DAMPING (5 TO 40)
    // walking_param.late_stance_imp_k = 3; //???
    // walking_param.slow_gain = 20;//90; // SLOW POWER (0 TO 400)
    // walking_param.fast_gain = 80;//110; //FAST POWER (0 TO 400)
    // walking_param.slow_exp = 0.9; //1.65; //SLOW TIMING (1 TO 5)
    // walking_param.fast_exp = 2.0; //2.35; //2.75; //FAST TIMING (1 TO 5)

    // //STAIR ASCENT
    // nf_param.us_gain_multiplier = 0.9;
    // nf_param.us_exp_multiplier = 1.5;
    
    // //STAIR DESCENT
    // nf_param.ds_early_stance_setpoint = -12.0;
    // nf_param.ds_early_stance_imp_k = 20.0;//40.0;
    // nf_param.ds_early_stance_imp_b = 50.0;//50.0;
    
    // //RAMP ASCENT
    // nf_param.ur_gain_multiplier = 0.7;
    // nf_param.ur_exp_multiplier = 1.0;

    // //RAMP DESCENT
    // nf_param.dr_est_k_multiplier = 1.5;
    // nf_param.dr_stance_b_multiplier = 2.0;
    // nf_param.dr_lst_k_multiplier = 1.5;
    // nf_param.ds_plantarflexion_delay_ms = 0;




    tm.mode = MODE_DS;
    tm.task = TASK_DS;
    tm.state = STATE_WALKING_ENTRY_STATE;

    tm.latestFootStaticSamples = 0;
    tm.latestFootOffTime = 10000;

    tm.foundFirstZvupAfterFootOn = 0;
    tm.taskPredicted = 0;
    tm.inSwing = 0;
    tm.achievedMinTorqueForStance = 0;

    tm.resetTrigger = 0;

    tm.feats = (float*)calloc(NFEATURES, sizeof(float));
}



 static void updateGaitEvents(void){

    
    // if (tm.state == STATE_LATE_SWING_DS || sensors.mc_sm_state == MCSM_TRAJECTORY_IMP_MODE_FOR_SWING_W_RES){
    //     return;
    // }
    // if (tm.state == STATE_LATE_SWING_DS){
    //     return;
    // }



    sensors.tqDot = 0.200f*sensors.tqDot + 0.800f*(sensors.EstAnkleTorque_f - sensors.prevEstAnkleTorque_f);
    

    if (sensors.EstAnkleTorque_f >= tm.maxTq){
        tm.maxTq = sensors.EstAnkleTorque_f;
        tm.maxTqInd = tm.loopcount;
    }

    if (tm.inSwing){
        //Detect if foot on has occurred
        if (tm.loopcount - tm.latestFootOffTime >= MIN_SWING_SAMPLES && tm.state != STATE_LATE_SWING_DS){
            if (sensors.EstAnkleTorque_f >= MIN_TQ_FOR_FOOT_ON)
            {
                tm.loopcount = 0;
                tm.foundFirstZvupAfterFootOn = 0;
                tm.inSwing = 0;
                tm.achievedMinTorqueForStance = 0;
                tm.maxTq = sensors.EstAnkleTorque_f;
                tm.maxTqInd = 0;
                tm.calculateTranslations = 0;
                tm.taskPredicted = 0;
                resetFeatures();
        }
      }
  }
  else{

        if (sensors.EstAnkleTorque_f > MIN_TQ_FOR_FOOT_STATIC && 
            sensors.tqDot > MIN_TQDOT_FOR_FOOT_STATIC &&
                sensors.EstAnkleTorque_f > tm.maxTq - 20.000f){

            if (tm.loopcount - tm.latestFootStaticTime > DEFAULT_STANCE_RESET_SAMPLES && 
                    !tm.foundFirstZvupAfterFootOn){
                tm.resetTrigger = 1;
                resetKinematics();
            }

            if (tm.accNormSq < UPPER_ACCNORM_THRESH_SQ && 
                tm.accNormSq > LOWER_ACCNORM_THRESH_SQ){
                    tm.foundFirstZvupAfterFootOn = 1;
                    tm.resetTrigger = 2;
                    resetKinematics();
            }  
        }
        

        if (tm.loopcount - tm.latestFootStaticTime > STANCE_RESET_EXPIRY_SAMPLES){
                tm.foundFirstZvupAfterFootOn = 0;
                tm.resetTrigger = 0;
                resetKinematics();
        }
        
        if (tm.loopcount > MIN_STANCE_SAMPLES &&
            sensors.EstAnkleTorque_f < MIN_TQ_FOR_FOOT_ON &&
            sensors.EstAnkleTorque_f > N_MIN_TQ_FOR_FOOT_ON){
            
            tm.inSwing = 1;
            tm.latestFootOffTime = tm.loopcount;
            tm.prediction = 1;
            tm.taskMachineEnabled = 1;
        }
    }

}

static int task_machine_demux_state = 0;

ERRCODE_T task_machine_demux(struct rigid_s* rigid){
switch (task_machine_demux_state){
  case INIT_TASK_MACHINE:
      init_task_machine();
      task_machine_demux_state++;
  break;
  case INIT_LEARNING:
      init_learner(&(tm.lrn));
      init_classifier(&(tm.lda));
      task_machine_demux_state++;
  break;
  case RUN_TASK_MACHINE:
      update_kinematics();
      update_features(get_kinematics());
      update_gait_events();
      // if (tm.loopcount - tm.latestFootStaticTime > 600){
      //   tm.kin.calculateTranslations = 0;
      // }
  break;
}

  if (sensors.acc2_raw != sensors.acc2_raw_prev){
    tm.startflag = 1;
}

    if (tm.startflag){
    if (tm.iter1){
        //17 flops
        updateLocalAcc();
        updateLocalOmega();
        if (!tm.taskPredicted){
            //9 flops
            updateIntegralsAndDerivatives();

        }else{
            solveDiscriminantFunction();
        }
        tm.iter1 = 0;
        tm.iter2 = 1;
  } else if (tm.iter2){

    if (!tm.taskPredicted){
        //33 flops
        updateTranslations();
         if (tm.taskMachineEnabled && tm.inSwing){        
              if (tm.loopcount - tm.latestFootOffTime >= PREDICTION_CUTOFF){
                 tm.taskPredicted = 1;     
                  solveDiscriminantFunction(); 
              }
          }
      }else{
           solveDiscriminantFunction(); 
      }

      tm.iter2 = 0;
      tm.iter3 = 1;
    } else if (tm.iter3){ 
        //8 swing no transition, +20 assignmetns with transition 
        updateGaitEvents();
        if (!tm.taskPredicted){
            //9 flops
            updateFeatures();
        }else{
            solveDiscriminantFunction();
        }
        tm.iter1 = 1;
        tm.iter3 = 0;
    }
        
    }

    tm.loopcount = tm.loopcount + 1;
  walking_statemachine();
}