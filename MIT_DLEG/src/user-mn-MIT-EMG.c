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
	*
****************************************************************************/



//****************************************************************************
// Include(s)
//****************************************************************************
#include "user-mn-MIT-EMG.h"

#ifdef USE_MIT_EMG_I2C
//****************************************************************************
// Variable(s)
//****************************************************************************
volatile uint8_t emgPeripheralState = EMG_PERIPH_READY;
volatile uint8_t emgOnFlag = 1;			// This must be one in order to work!!! ;)

volatile uint8_t emgState = EMG_STATE_DISABLE;
volatile uint16_t emgTimer = 0; //1tick represent 1ms
volatile uint16_t emgPrsc = 0;
volatile uint16_t emgResetTimer = 0;
volatile uint8_t emgReadyFlag=0;
//volatile uint8_t emg_active_flag
uint16_t emgTimestamp = 0;

int16_t emgData[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int16_t emgMisc[3] = {0,0,0};
extern uint8_t i2c2_dma_tx_buf[24]; //from i2c.c
extern uint8_t i2c2_dma_rx_buf[24]; //from i2c.c
extern I2C_HandleTypeDef hi2c2;

int16_t emgScalers[16] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
//****************************************************************************
// Function(s)
//****************************************************************************

//scales raw EMG data from EMG board before ever transmitting to Plan
//adjust using user writes
void scaleEMGMultipacket(void) {

	//update emgScalers with user write values
	if (sizeof(emgScalers)/sizeof(int16_t) <= sizeof(user_data_1.w)/sizeof(int32_t)) {
		for (uint i = 0; i < sizeof(emgScalers)/sizeof(int16_t); i++) {

			//pull scalers from user writes
			if (user_data_1.w[i] > 0) {
				emgScalers[i] = user_data_1.w[i];
			}

			//1 is no scaling
			if (emgScalers[i] != 1) {
				float act_norm = ((float) emgData[i])/((float) emgScalers[i]);
				emgData[i] = (int16_t) (act_norm*10000); //this value sent to odroid to be divided by 10,000 later
			}


			//catch leaks and negative scalers
			if (emgData[i] < 0) {
				emgData[i] = 0;
			} else if (emgData[i] > 10000) {
				emgData[i] = 10000;
			}

		}
	}
}

/*
 * Updates the value of emgReadyFlag based off of input from rigid1
 */
void mitEmgUpdateStatus(void)
{

	#ifdef EMG_LINE_READY
		if( rigid1.mn.analog[EMG_LINE_READY] > EMG_LINE_THRESHOLD)
			emgReadyFlag =1;
		else
			emgReadyFlag = 0;

	#else
		emgReadyFlag =1;

	#endif

	return;
}

/*
 * Decoder for EMG
 * TODO: find out what this does
 */
void mitEmgDecode(void)
{
	//ToDo: need to include packet check function
		emgTimestamp = (uint16_t)i2c2_dma_rx_buf[5] + (uint16_t)i2c2_dma_rx_buf[6]*100;
		memcpy(emgTimestamp, i2c2_dma_rx_buf+5,2);
		memcpy(emgData, i2c2_dma_rx_buf+7,16);
		scaleEMGMultipacket();
//		memcpy(emgMisc, i2c2_dma_rx_buf+2,6);
		/*
		for(uint8_t i=0;i<8;i++)
			rigid1.mn.genVar[i] = emgData[i];
*/
		return;
}

/*
 * Description TODO: find out what this does
 */
void mitEmgRead(void)
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

/*
 * Seems to do absolutely nothing, may delete.
 */
void mitEmgI2CErrorCallback(I2C_HandleTypeDef *hi2c)
{
	return;
}

/*
 *  Include this function in the I2C Return callback
 *  TODO: find out what this does
 */
void mitEmgI2CRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == I2C2)
	{
		if(emgState == EMG_STATE_WAIT)
		{
			emgState = EMG_STATE_READ;
		}

		else if(emgState == EMG_STATE_READ)
		{
			mitEmgDecode();
			emgPeripheralState = EMG_PERIPH_RECEIVE_COMPLETE;
			emgTimer = 0;
		}
	}

	return;
}

/*
 *  Finite State Machine with multiple states for the EMG.
 *  The possible states are as follows:
 *  	EMG_STATE_DISABLE
 *  	EMG_STATE_INACTIVE
 *  	EMG_STATE_WAIT
 *  	EMG_STATE_READ
 *  	EMG_STATE_DEINIT
 *  	EMG_STATE_RECOVER
 */
void mitEmgI2c2Fsm(void)
{
	mitEmgUpdateStatus();


	switch(emgState) //emg state machine
	{
		case EMG_STATE_DISABLE:
			if(emgOnFlag ==1)
			{
				emgState=EMG_STATE_INACTIVE;
				emgTimer = 0;
			}
			break;

		case EMG_STATE_INACTIVE:
			if(emgOnFlag!=1)
			{
				emgState = EMG_STATE_DISABLE;
			}
			else if(emgReadyFlag ==1)
			{
				emgState = EMG_STATE_WAIT;
				emgTimer = 0;
			}

			break;

		case EMG_STATE_WAIT:
		// periodic reset
			if(emgResetTimer > EMG_I2C_RESET_PERIOD)
			{
				emgState = EMG_STATE_DEINIT;
				emgTimer = 0;
				emgResetTimer = 0;
				break;
			}

			if(emgOnFlag!=1)
			{
				emgState = EMG_STATE_DISABLE;
				break;
			}
			else if(emgReadyFlag!=1)
				emgState = EMG_STATE_INACTIVE;

			if(emgTimer>EMG_TIMER_THRESHOLD)
				emgTimer = 0;

			if(emgTimer ==0)
				mitEmgRead(); //try to read, 5Hz

			emgResetTimer++;
			break;

		case EMG_STATE_READ:
			if(emgOnFlag!=1)
			{
				emgTimer = 0;
				emgState = EMG_STATE_DISABLE;
				break;
			}

			else if(emgReadyFlag ==1) // line on
			{
				if(emgPrsc ==0)
				{
						mitEmgRead();
						emgPeripheralState = EMG_PERIPH_RECEIVE_WAIT;
				}

				if(emgTimer>EMG_TIMER_PRESCALER*5)
				{
//					emgState = EMG_STATE_WAIT; //just go back to wait state
					emgState = EMG_STATE_DEINIT; //Reboot I2C peripheral
					emgTimer = 0;
					emgPeripheralState = EMG_PERIPH_READY;
				}

				emgPrsc++;
				if(emgPrsc>= EMG_TIMER_PRESCALER)
					emgPrsc =0;
			}

			else if(emgReadyFlag!=1)
			{
				emgTimer = 0;
				emgState = EMG_STATE_INACTIVE;

				emgPeripheralState == EMG_PERIPH_READY;
			}
			break;

		case EMG_STATE_DEINIT:
			if(emgTimer >=EMG_DEINIT_PERIOD )
			{
				emgTimer = 0;
				disable_i2c2();
				emgState = EMG_STATE_RECOVER;
			}
			break;
		case EMG_STATE_RECOVER:
			if(emgTimer == EMG_DEINIT_PERIOD)
			{
				init_i2c2();
			}
			else if(emgTimer>= EMG_DEINIT_PERIOD*2)
			{
				emgTimer = 0;
				emgState = EMG_STATE_WAIT;
			}
			break;
		default:
			break;
	}
		emgTimer++;
}

/*
 * get an integer value representation of the state of the emg.
 * if emgReadyFlag and emgOnFlag both equal 1, and the emgState is EMG_STATE_READ, then the state is 1.
 * Otherwise it is 0.
 * Returns state(uint8_t): an integer value representation of the state
 */
uint8_t mitEmgGetState(void) //read value when only 1 is returned
{
	if( emgReadyFlag ==1 && emgState == EMG_STATE_READ && emgOnFlag ==1 )
	{
		return 1;
	}
	else
	{
		return 0;
	}

}
/*
 * If on is between 0 and 1, set emgOnFlag to the value on. Otherwise do nothing.
 * Param: on(uint8_t): value to flag
 */
void mitEmgChangeState(uint8_t on)
{
	if( on >= 0 && on <= 1 )
		emgOnFlag = on;

	return;
}


#endif // USE_MIT_EMG_I2C
