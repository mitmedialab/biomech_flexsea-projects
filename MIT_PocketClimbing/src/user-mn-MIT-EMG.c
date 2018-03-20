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

volatile uint8_t emg_state = EMG_STATE_DISABLE;
volatile uint16_t emg_timer = 0; //1tick represent 1ms
volatile uint16_t emg_prsc = 0;

volatile uint8_t emg_ready_flag=0;
volatile uint16_t emg_error_cnt=0;
//volatile uint8_t emg_active_flag;
uint16_t emg_timestamp = 0;
uint8_t emg_packet = EMG_DATA_MAC;

int16_t emg_data[8] = {0,0,0,0,0,0,0,0};
extern uint8_t i2c2_dma_tx_buf[24]; //from i2c.c
extern uint8_t i2c2_dma_rx_buf[24]; //from i2c.c
extern I2C_HandleTypeDef hi2c2;
//****************************************************************************
// Function(s)
//****************************************************************************

void MIT_EMG_update_status(void)
{
	// ToDo: Change code later
	// emg_ready_flag = ( rigid1.mn.analog[EMG_LINE_READY] > EMG_LINE_THRESHOLD);
	// emg_active_flag = ( rigid1.mn.analog[EMG_LINE_ACTIVE] > EMG_LINE_THRESHOLD);
#ifdef EMG_LINE_READY
	if( rigid1.mn.analog[EMG_LINE_READY] > EMG_LINE_THRESHOLD)
		emg_ready_flag =1;
	else
		emg_ready_flag = 0;
#else
	emg_ready_flag =1;
#endif

/*
	if( rigid1.mn.analog[EMG_LINE_ACTIVE] > EMG_LINE_THRESHOLD)
		emg_active_flag =1;
	else
		emg_active_flag = 0;
*/
	return;
}

/*
void MIT_EMG_command(uint8_t mode, uint8_t packet)
{
	static HAL_StatusTypeDef retVal;

	i2c2_dma_tx_buf[0] = 0xFE; //Communication header
	i2c2_dma_tx_buf[1] = mode; // Communication mode
	i2c2_dma_tx_buf[2] = 1; //comm prescaler -> not applicable
	i2c2_dma_tx_buf[3] = packet; // PacketType
	i2c2_dma_tx_buf[4] = 0;
	i2c2_dma_tx_buf[5] = 0;
	i2c2_dma_tx_buf[6] = 0;
	i2c2_dma_tx_buf[7] = 0;
	i2c2_dma_tx_buf[8] = 0;
	i2c2_dma_tx_buf[9] = 0;


	retVal = HAL_I2C_Master_Transmit_DMA(&hi2c2, I2C_SLAVE_ADDR_EMG, i2c2_dma_tx_buf, 10);
	if(retVal == HAL_OK)
	{
		i2c2FsmState = I2C_FSM_TX_ADDR;
	}
	else
	{
		i2c2FsmState = I2C_FSM_PROBLEM;
	}
}
*/

void MIT_EMG_decode(void)
{
	//ToDo: need to include packet check function
		emg_timestamp = (uint16_t)i2c2_dma_rx_buf[5] + (uint16_t)i2c2_dma_rx_buf[6]*100;
	  memcpy(emg_data, i2c2_dma_rx_buf+8,16);

	//	for(uint8_t i=0;i<8;i++)
		//	rigid1.mn.genVar[i] = emg_data[i];

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
	/* Can use the JF prototype if he update this on his function
	if(hi2c->Instance == I2C2)
		{
			//Indicate that it's done receiving:
			i2c2FsmState = I2C_FSM_RX_DATA_DONE;
		}
	*/

	if(hi2c->Instance == I2C2)
	{
		//Indicate that it's done receiving:
		if(emg_state != EMG_STATE_READ)
			emg_state = EMG_STATE_READ;

		emg_peripheral_state = EMG_PERIPH_RECEIVE_COMPLETE;
	}
	return;
}

void MIT_EMG_i2c2_fsm(void)
{
	MIT_EMG_update_status();

	if(emg_peripheral_state == EMG_PERIPH_RECEIVE_COMPLETE)
	{
		MIT_EMG_decode();
		emg_peripheral_state = EMG_PERIPH_READY;
		emg_timer = 0;
	}

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

			break;

		case EMG_STATE_READ:
			if(emg_on_flag!=1)
			{
				emg_timer = 0;
				emg_state = EMG_STATE_DISABLE;
				break;
			}

			else if(emg_ready_flag ==1) //attempt ready
			{
				if(emg_prsc ==0)
				{
					if(emg_error_cnt>3)
					emg_peripheral_state = EMG_PERIPH_READY;

					if( emg_peripheral_state == EMG_PERIPH_READY)
					{
						MIT_EMG_read();
						emg_peripheral_state = EMG_PERIPH_RECEIVE_WAIT;
						emg_error_cnt=0;
					}
					else
					{
						emg_error_cnt++;
					}
				}

				if(emg_timer>EMG_TIMER_THRESHOLD)
				{
					emg_state = EMG_STATE_WAIT;
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
		default:
			break;
	}

		emg_timer++;
}
//External 8-ch EMG Amplifier:

/*
void get_8ch_emg(void)
{
	static uint8_t cnt = 0;
	HAL_StatusTypeDef retVal;

	i2c2_dma_tx_buf[0] = 10;
	i2c2_dma_tx_buf[1] = 20;
	i2c2_dma_tx_buf[2] = 30;
	i2c2_dma_tx_buf[3] = cnt++;

	retVal = HAL_I2C_Master_Transmit_DMA(&hi2c2, I2C_SLAVE_ADDR_EMG, i2c2_dma_tx_buf, 20);
	/*
	retVal = HAL_I2C_Mem_Read_DMA(&hi2c2, I2C_SLAVE_ADDR_EMG, (uint16_t) MEM_R_CH1_H,
								I2C_MEMADD_SIZE_8BIT, i2c2_dma_rx_buf, 20);


	if(retVal == HAL_OK){i2c2FsmState = I2C_FSM_RX_DATA;}
	else{i2c2FsmState = I2C_FSM_PROBLEM;}
}
*/

//Unpack from buffer -->int16_t emg_buf[8];
/*
void unpackCompressed8ch(uint8_t *buf, int16_t *emg_buf )
{
	memcpy(emg_buf, buf, 16);
	return;
}*/
/*
void decode8chEmg(void)
{
	uint8_t retVal = 0;
//	unpackCompressed8ch(i2c2_dma_rx_buf, ext_emg);

	//Copy to genVar (GUI):
//	for(int i = 0; i < 8; i++){rigid1.mn.genVar[i] = ext_emg[i];}

	retVal = HAL_I2C_Master_Receive_DMA(&hi2c2, I2C_SLAVE_ADDR_EMG, i2c2_dma_rx_buf, 20);

}
*/
#endif // USE_MIT_EMG_I2C
