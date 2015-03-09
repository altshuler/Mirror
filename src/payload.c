#include <stddef.h>
#include <stdint.h>
#include "payload.h"



int validPayload(PAYLOAD_HEADER *payload, size_t actualSize)
{
	size_t payloadSize;
	
	if (payload==NULL)
		return 0;
	
	/*if (payload->magicNumber!=PAYLOAD_MAGIC_NUMBER)
		return 0;*/
	// Check for size inconsistency if actual size specified
	if (actualSize)
	{
		if (actualSize<sizeof(PAYLOAD_HEADER))
			return 0; // payload record with incomplete header
		/*payloadSize=payload->length+sizeof(PAYLOAD_HEADER);
		
		if (actualSize<payloadSize)
			return 0;*/ // the payload record is truncated,  hence invalid. 
	}
	return 1;
}


