#ifdef BOARD_TYPE_FLEXSEA_PLAN

#include <stdlib.h>
#include <flexsea.h>
#include <dynamic_user_structs.h>
#include <flexsea_system.h>
#include "flexsea_dataformats.h"
#include <string.h>
#include <flexsea_cmd_user.h>
#include <stdio.h>
#ifdef __cplusplus
	extern "C" {
#endif

/* This works for Plan - Execute only
	Requirements:

	- send labels to GUI
	- send data to GUI
	- keep msg under max size

*/
uint8_t newMetaDataAvailable = 0;
uint8_t packAndSendOffsetRequest = 0;
uint8_t newDataAvailable = 0;
uint8_t waitingOnFieldFlags = 0;

int dynamicUser_slaveId = -1;
uint8_t dynamicUser_numFields = 0;
uint8_t* dynamicUser_data = NULL;
uint8_t* dynamicUser_fieldTypes = NULL;
uint8_t* dynamicUser_labelLengths = NULL;
char** dynamicUser_labels = NULL;

uint8_t* dynamicUser_fieldFlagsPlan = NULL;
uint8_t* dynamicUser_fieldFlagsExec = NULL;

void* getMemory(void* ptr, int size)
{
	if(ptr)
        ptr = realloc(ptr, size);
	else
	{
		ptr = malloc(size);
		memset(ptr, 0, size);
	}

	return ptr;
}

uint8_t sizeOfFieldType(uint8_t format)
{
	if(format > FORMAT_8S) //for unknown format we just over allocate I guess
		return 8;

	return FORMAT_SIZE_MAP[format];
}

static int lastOffsetReceived = -1;
static int receivingMetaData = 0;
void rx_metaDataStart(uint8_t* buf, uint16_t index)
{
	dynamicUser_slaveId = buf[P_XID];
	uint8_t numFields = buf[index++];
	uint16_t totalBytes = buf[index++];

	if(numFields != dynamicUser_numFields)
	{
		if(dynamicUser_labels)
		{
			int i;
			for(i=0; i<dynamicUser_numFields;i++)
			{
				char* label = dynamicUser_labels[i];
				if(label) { free(label); }
				label = NULL;
			}
		}

		dynamicUser_fieldTypes =    (uint8_t*) getMemory(dynamicUser_fieldTypes, sizeof(uint8_t)*numFields);
		dynamicUser_labelLengths =  (uint8_t*) getMemory(dynamicUser_labelLengths, sizeof(uint8_t)*numFields);
		dynamicUser_labels =        (char**)   getMemory(dynamicUser_labels, numFields*sizeof(char*));
		int i;
		for(i=0; i < numFields; i++)
			dynamicUser_labels[i] = NULL;

		dynamicUser_fieldFlagsExec =  (uint8_t*) getMemory(dynamicUser_fieldFlagsExec, sizeof(uint8_t)*numFields);
		dynamicUser_fieldFlagsPlan =  (uint8_t*) getMemory(dynamicUser_fieldFlagsPlan, sizeof(uint8_t)*numFields);

		if(numFields > dynamicUser_numFields)
		{
			memset(dynamicUser_fieldTypes + dynamicUser_numFields, 0, numFields - dynamicUser_numFields);
			memset(dynamicUser_labelLengths + dynamicUser_numFields, 0, numFields - dynamicUser_numFields);
			memset(dynamicUser_fieldFlagsExec + dynamicUser_numFields, 0, numFields - dynamicUser_numFields);
			memset(dynamicUser_fieldFlagsPlan + dynamicUser_numFields, 0, numFields - dynamicUser_numFields);
		}
	}
	dynamicUser_numFields = numFields;

	static int lastNumDataBytes = -1;
	if(totalBytes != lastNumDataBytes)
	{
		dynamicUser_data =  (uint8_t*) getMemory(dynamicUser_data, totalBytes);
		memset(dynamicUser_data, 0, totalBytes);
		lastNumDataBytes = totalBytes;
	}

	receivingMetaData = 2;
	lastOffsetReceived = -1;
	return;
}

int rx_metaDataOffset(uint8_t* buf, uint16_t index)
{
	if(buf[P_XID] != dynamicUser_slaveId) return -1;
	uint8_t offset = buf[index++];
	if(offset >= dynamicUser_numFields) return -1;

	if(offset != lastOffsetReceived + 1)
	{
		return -1;
	}
	lastOffsetReceived++;

	//parse field type
	uint8_t fieldType = buf[index++];
	if(fieldType < FORMAT_FILLER)
	{
		dynamicUser_fieldTypes[offset] = fieldType;
	}

	//parse label length, and allocate for label string
	uint8_t labelLength = buf[index++];
	if(labelLength != dynamicUser_labelLengths[offset] || !dynamicUser_labels[offset])
	{
		dynamicUser_labelLengths[offset] = labelLength;
		dynamicUser_labels[offset] = (char*) getMemory(dynamicUser_labels[offset], labelLength);
	}

	int j;
	//parse  label
	for(j = 0; j < labelLength; j++)
	{
		dynamicUser_labels[offset][j] = buf[index++];
	}

	return 0;
}

void rx_data(uint8_t *shBuf, uint16_t index)
{
	if(!dynamicUser_fieldTypes) return;
	if(!dynamicUser_data) return;

	uint8_t totalBytesToRead = shBuf[index++];

	int i, j, fieldLength, fieldOffset = 0, totalBytesRead = 0;
	for(i = 0; i < dynamicUser_numFields; i++)
	{
		fieldLength = 1;
		if(dynamicUser_fieldTypes[i] < 8 && FORMAT_SIZE_MAP[dynamicUser_fieldTypes[i]] > 0)
			fieldLength = FORMAT_SIZE_MAP[dynamicUser_fieldTypes[i]];

		if(dynamicUser_fieldFlagsExec[i])
		{
			for(j = 0; j < fieldLength && totalBytesRead < totalBytesToRead; j++)
			{
				dynamicUser_data[fieldOffset + j] = shBuf[index++];
				totalBytesRead++;
			}
		}
		else
		{
			for(j = 0; j < fieldLength; j++)
			{
				dynamicUser_data[fieldOffset + j] = 0;
			}
		}

		fieldOffset += fieldLength;
	}

	newDataAvailable = 1;
}

void tx_cmd_user_dyn_request(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, uint16_t *len, \
						uint8_t sendMetaData, uint8_t offset)
{
	*cmd = CMD_USER_DYNAMIC;
	*cmdType = CMD_READ;

	uint16_t index = 0;

	if(sendMetaData)
	{
		shBuf[index++] = sendMetaData;
		shBuf[index++] = offset;
	}
	else
	{
		shBuf[index++] = sendMetaData;
	}

	*len = index;
}

void rx_cmd_user_dyn_rr(uint8_t *buf, uint8_t *info)
{
	(void)info;

	uint16_t index = P_DATA1;

	uint8_t flag = buf[index++];
	static uint8_t retryCount = 0;

	if(flag == SEND_METADATA)
	{
		if(receivingMetaData == 1)
		{
			rx_metaDataStart(buf, index);
			lastOffsetReceived = -1;
			tx_cmd_user_dyn_request(TX_N_DEFAULT, SEND_METADATA, 0);
			receivingMetaData = 2;
			packAndSendOffsetRequest = 1;
		}
		else if(receivingMetaData == 2)
		{
			int o = lastOffsetReceived;
			rx_metaDataOffset(buf, index);
			if(o == lastOffsetReceived)
				retryCount++;
			else
				retryCount = 0;

			if(lastOffsetReceived < dynamicUser_numFields-1 && retryCount < 10)
			{
				tx_cmd_user_dyn_request(TX_N_DEFAULT, SEND_METADATA, lastOffsetReceived+1);
				packAndSendOffsetRequest = 1;
			}
			else
			{
				packAndSendOffsetRequest = 0;
				waitingOnFieldFlags = 1;
				receivingMetaData = 0;
				newMetaDataAvailable = retryCount < 10;
			}
		}
	}
	else if(flag == SEND_FIELD_FLAGS)
	{
		if(!unpackFieldFlags(buf+index, dynamicUser_fieldFlagsExec, dynamicUser_numFields))
			waitingOnFieldFlags = 0;
	}
	else
	{
		rx_data(buf, index);
	}
}

void tx_cmd_user_dyn_r(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, uint16_t *len, \
						uint8_t sendMetaData)
{
	tx_cmd_user_dyn_request(shBuf, cmd, cmdType, len, sendMetaData, 0xFF);
	receivingMetaData = 1;
}

void tx_cmd_user_dyn_w(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, uint16_t *len)
{

	*cmd = CMD_USER_DYNAMIC;
	*cmdType = CMD_WRITE;

	uint16_t index = packFieldFlags(shBuf, dynamicUser_numFields, dynamicUser_fieldFlagsPlan);
	waitingOnFieldFlags = 1;
	*len = index;
}

void init_flexsea_payload_ptr_dynamic()
{
	flexsea_payload_ptr[CMD_USER_DYNAMIC][RX_PTYPE_REPLY] = &rx_cmd_user_dyn_rr;
}

#ifdef __cplusplus
	}
#endif

#endif 	//BOARD_TYPE_FLEXSEA_PLAN
