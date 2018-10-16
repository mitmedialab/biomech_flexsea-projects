/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-manage' Mid-level computing, and networking
	Copyright (C) 2017 Dephy, Inc. <http://dephy.com/>
*****************************************************************************
	[Lead developper] Jean-Francois (JF) Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] emg: external emg Processor
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2018-03-12 | syeon | Initial release
	* 2018-03-23 | syeon | Add muscle activation calculation and calibration
****************************************************************************/

#include "user-mn.h"
#ifdef USE_MIT_EMG_I2C

//****************************************************************************
// Include(s)
//****************************************************************************
#include "main.h"
#include "i2c.h"
#include <i2c.h>
#include "user-mn-MIT-EMG.h"
#include "flexsea_global_structs.h"
#include "flexsea_user_structs.h"
#include "flexsea.h"

//****************************************************************************
// Variable(s)
//****************************************************************************
volatile uint8_t emg_peripheral_state = EMG_PERIPH_READY;
volatile uint8_t emg_on_flag = 1;
volatile uint8_t emg_activation_flag = 1;
volatile uint8_t emg_state = EMG_STATE_DISABLE;
volatile uint16_t emg_timer = 0; //1tick represent 1ms
volatile uint16_t emg_prsc = 0;
volatile uint16_t emg_reset_timer = 0;
volatile uint8_t emg_ready_flag=0;
//volatile uint8_t emg_active_flag;
uint16_t emg_timestamp = 0;

int16_t emg_data[8] = {0,0,0,0,0,0,0,0};
int16_t emg_processed_data[8] = {0,0,0,0,0,0,0,0};

float emg_activation[8] = {0,0,0,0,0,0,0,0};
float emg_threshold[8][2] = {
	{USER_MIN_CH1,USER_MAX_CH1},
	{USER_MIN_CH2,USER_MAX_CH2},
	{USER_MIN_CH3,USER_MAX_CH3},
	{USER_MIN_CH4,USER_MAX_CH4},
	{USER_MIN_CH5,USER_MAX_CH5},
	{USER_MIN_CH6,USER_MAX_CH6},
	{USER_MIN_CH7,USER_MAX_CH7},
	{USER_MIN_CH8,USER_MAX_CH8}
};
int16_t emg_misc[3] = {0,0,0};
extern uint8_t i2c2_dma_tx_buf[24]; //from i2c.c
extern uint8_t i2c2_dma_rx_buf[24]; //from i2c.c
extern I2C_HandleTypeDef hi2c2;

uint8_t emg_calibration_muscle = 0xFF;
uint8_t emg_calibration_state = EMG_CALSTATE_OFF;
//****************************************************************************
// Function(s)
//****************************************************************************
void MIT_EMG_start_calibration_min(void)
{
	for(uint8_t i=0;i<8;i++)
	{
		emg_threshold[i][MIT_EMG_MUSCLE_MIN] = 0;
	}

	emg_calibration_muscle = 0xFF;
	emg_calibration_state = EMG_CALSTATE_MIN;
	return;
}

void MIT_EMG_start_calibration_max(uint8_t muscle_ch)
{
	if(muscle_ch >=0 && muscle_ch<8)
	{
		emg_calibration_muscle = muscle_ch;
		emg_threshold[emg_calibration_muscle][MIT_EMG_MUSCLE_MAX] = MIT_EMG_BASELINE_THRESHOLD;
	}
	else if(muscle_ch == MIT_EMG_MUSCLE_ALL)
	{
		emg_calibration_muscle = muscle_ch;
	}

	emg_calibration_state = EMG_CALSTATE_MAX;
	return;
}

void MIT_EMG_stop_calibration(void)
{
	if(emg_calibration_state == EMG_CALSTATE_MIN)
	{
		for(uint8_t i=0;i<8;i++)
			emg_threshold[i][MIT_EMG_MUSCLE_MIN] +=MIT_EMG_BASELINE_THRESHOLD;
	}

	emg_calibration_muscle = 0xFF;
	emg_calibration_state = EMG_CALSTATE_OFF;
	return;
}

void MIT_EMG_calibration_fsm(void)
{
	switch(emg_calibration_state)
	{
		case EMG_CALSTATE_OFF:
			break;
		case EMG_CALSTATE_MIN:
			MIT_EMG_calibrate_activation_min();
			break;
		case EMG_CALSTATE_MAX:
			MIT_EMG_calibrate_activation_max();
			break;
		default:
			break;
	}
	return;
}

void MIT_EMG_calibrate_activation_max(void)
{
	if(emg_calibration_muscle == MIT_EMG_MUSCLE_ALL)
	{
		for(int i = 0; i<8;i++)
		{
			if(emg_threshold[i][MIT_EMG_MUSCLE_MAX] < (float)emg_data[i] )
			{
				emg_threshold[i][MIT_EMG_MUSCLE_MAX] = (float)emg_data[i];
			}
		}
	}
	else
	{
		if(emg_threshold[emg_calibration_muscle][MIT_EMG_MUSCLE_MAX] < (float)emg_data[emg_calibration_muscle] )
		{
			emg_threshold[emg_calibration_muscle][MIT_EMG_MUSCLE_MAX] = (float)emg_data[emg_calibration_muscle];
		}
	}

	return;
}

void MIT_EMG_calibrate_activation_min(void)
{
	for(int i = 0; i<8;i++)
	{
		if(emg_threshold[i][MIT_EMG_MUSCLE_MIN] < (float)emg_data[i] )
		{
			emg_threshold[i][MIT_EMG_MUSCLE_MIN] = (float)emg_data[i];
		}
	}

	return;
}

void MIT_EMG_calculate_activation(void)
{
	for(uint8_t i=0;i<8;i++)
	{
		emg_activation[i] = ((float)emg_data[i] - emg_threshold[i][MIT_EMG_MUSCLE_MIN]) /(emg_threshold[i][MIT_EMG_MUSCLE_MAX]-emg_threshold[i][MIT_EMG_MUSCLE_MIN]);

		if(emg_activation[i]<0)
			emg_activation[i] = 0;
		else if(emg_activation[i]>MIT_EMG_ACTIVATION_MAX)
			emg_activation[i] = MIT_EMG_ACTIVATION_MAX;

		emg_processed_data[i] = (int16_t)(emg_activation[i]*10000);
	}

	return;
}


void MIT_EMG_update_status(void)
{

#ifdef EMG_LINE_READY
	if( rigid1.mn.analog[EMG_LINE_READY] > EMG_LINE_THRESHOLD)
		emg_ready_flag =1;
	else
		emg_ready_flag = 0;

#else
	emg_ready_flag =1;

#endif

	return;
}


void MIT_EMG_decode(void)
{
	//ToDo: need to include packet check function
		emg_timestamp = (uint16_t)i2c2_dma_rx_buf[5] + (uint16_t)i2c2_dma_rx_buf[6]*100;
	  memcpy(emg_data, i2c2_dma_rx_buf+8,16);
		memcpy(emg_misc, i2c2_dma_rx_buf+2,6);
		/*
		for(uint8_t i=0;i<8;i++)
			rigid1.mn.genVar[i] = emg_data[i];
*/
		return;
}

void MIT_EMG_read(void)
{
	static HAL_StatusTypeDef retVal;
	retVal = HAL_I2C_Master_Receive_DMA(&hi2c2, I2C_SLAVE_ADDR_EMG, i2c2_dma_rx_buf, 24);
	if(retVal == HAL_OK)
	{
		i2c2FsmState = I2C_FSM_RX_DATA;
	}
	else
	{
		i2c2FsmState = I2C_FSM_PROBLEM;
	}
}

void MIT_EMG_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	return;
}
//Include this function in the I2C Return callback
void MIT_EMG_I2C_RxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == I2C2)
	{
		if(emg_state == EMG_STATE_WAIT)
		{
			emg_state = EMG_STATE_READ;
		}

		else if(emg_state == EMG_STATE_READ)
		{
			MIT_EMG_decode();
			emg_peripheral_state = EMG_PERIPH_RECEIVE_COMPLETE;
			emg_timer = 0;
		}
	}

	return;
}

void MIT_EMG_i2c2_fsm(void)
{
	MIT_EMG_update_status();

	switch(emg_state) //emg state machine
	{
		case EMG_STATE_DISABLE:
			if(emg_on_flag ==1)
			{
				emg_state=EMG_STATE_INACTIVE;
				emg_timer = 0;
			}
			break;

		case EMG_STATE_INACTIVE:
			if(emg_on_flag!=1)
			{
				emg_state = EMG_STATE_DISABLE;
			}
			else if(emg_ready_flag ==1)
			{
				emg_state = EMG_STATE_WAIT;
				emg_timer = 0;
			}

			break;

		case EMG_STATE_WAIT:
		// periodic reset
			if(emg_reset_timer > EMG_I2C_RESET_PERIOD)
			{
				emg_state = EMG_STATE_DEINIT;
				emg_timer = 0;
				emg_reset_timer = 0;
				break;
			}

			if(emg_on_flag!=1)
			{
				emg_state = EMG_STATE_DISABLE;
				break;
			}
			else if(emg_ready_flag!=1)
				emg_state = EMG_STATE_INACTIVE;

			if(emg_timer>EMG_TIMER_THRESHOLD)
				emg_timer = 0;

			if(emg_timer ==0)
				MIT_EMG_read(); //try to read, 5Hz

			emg_reset_timer++;
			break;

		case EMG_STATE_READ:
			if(emg_on_flag!=1)
			{
				emg_timer = 0;
				emg_state = EMG_STATE_DISABLE;
				break;
			}

			else if(emg_ready_flag ==1) // line on
			{
				if(emg_prsc ==0)
				{
						MIT_EMG_read();
						emg_peripheral_state = EMG_PERIPH_RECEIVE_WAIT;
				}

				if(emg_timer>EMG_TIMER_PRESCALER*5)
				{
					//emg_state = EMG_STATE_WAIT; //just go back to wait state
					emg_state = EMG_STATE_DEINIT; //Reboot I2C peripheral
					emg_timer = 0;
					emg_peripheral_state = EMG_PERIPH_READY;
				}

				emg_prsc++;
				if(emg_prsc>= EMG_TIMER_PRESCALER)
					emg_prsc =0;
			}

			else if(emg_ready_flag!=1)
			{
				emg_timer = 0;
				emg_state = EMG_STATE_INACTIVE;

				emg_peripheral_state == EMG_PERIPH_READY;
			}
			break;

		case EMG_STATE_DEINIT:
			if(emg_timer >=EMG_DEINIT_PERIOD )
			{
				emg_timer = 0;
				disable_i2c2();
				emg_state = EMG_STATE_RECOVER;
			}
			break;
		case EMG_STATE_RECOVER:
			if(emg_timer == EMG_DEINIT_PERIOD)
			{
				init_i2c2();
			}
			else if(emg_timer>= EMG_DEINIT_PERIOD*2)
			{
				emg_timer = 0;
				emg_state = EMG_STATE_WAIT;
			}
			break;
		default:
			break;
	}

	MIT_EMG_calibration_fsm();

	if(emg_activation_flag>=1)
		MIT_EMG_calculate_activation();

	emg_timer++;
}

uint8_t MIT_EMG_getState(void) //read value when only 1 is returned
{
	if( emg_ready_flag ==1 && emg_state == EMG_STATE_READ && emg_on_flag ==1 )
	{
		return 1;
	}
	else
	{
		return 0;
	}

}

void MIT_EMG_changeState(uint8_t on)
{
	if( on >= 0 && on <= 1 )
		emg_on_flag = on;

	return;
}


#endif // USE_MIT_EMG_I2C
