#include "../inc/dynamic_user_structs.h"

#ifdef __cplusplus
	extern "C" {
#endif

uint16_t packFieldFlags(uint8_t* shBuf, uint8_t numFields, uint8_t* fieldFlags)
{
	// We pack so that the least significant bit is the first flag
	// Since we are left shifting that means we have to start with the last flag

	if(!fieldFlags) return 0;

	uint8_t numBytes = numFields / 8 + (numFields % 8 != 0);

	int fieldIndex, byteIndex, bufIndex=0;
	uint8_t packedByte = 0, shouldSend;

	shBuf[bufIndex++] = numFields;

	for(byteIndex = 0; byteIndex < numBytes; byteIndex++)
	{
		for(fieldIndex = (byteIndex+1) * 8 - 1; fieldIndex >= (byteIndex*8); fieldIndex--)
		{
			if(fieldIndex >= numFields)
				continue;

			packedByte = packedByte << 1;
			shouldSend = fieldFlags[fieldIndex] > 0 ? 1 : 0;
			packedByte |= shouldSend;
		}
		shBuf[bufIndex++] = packedByte;
	}

	return bufIndex;
}

int unpackFieldFlags(uint8_t* buf, uint8_t* fieldFlags, uint16_t len)
{
	int bufIndex = 0;
	uint8_t numFields = buf[bufIndex++];
	if(numFields != len)	return -1;
	
	uint8_t numBytes = numFields / 8 + (numFields % 8 != 0);
	int byteIndex, fieldIndex;
	for(byteIndex = 0; byteIndex < numBytes; byteIndex++)
	{
		uint8_t byte = buf[bufIndex++];
		uint8_t i = 0;
		for(fieldIndex = 8*byteIndex; fieldIndex < 8*(byteIndex+1) && fieldIndex < numFields; fieldIndex++)
		{
			
			uint8_t shouldSend = (byte >> i) & 0x01;
			fieldFlags[fieldIndex] = shouldSend;
			i++;
		}
	}

	return 0;	
}

#ifdef __cplusplus
	}
#endif

