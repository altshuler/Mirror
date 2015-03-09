/*******************************************************************************
 * @file hostframe.c
 * @ Host frame 
 *
 * @author Andrei Mamtsev
 *
 * @version 0.0.1
 * @date 09.02.2013
 *
*/
/**************************************************************************/
/* Standard Includes */
/**************************************************************************/

#include <stddef.h>
#include <stdint.h>
#include <string.h>
/**************************************************************************/
/* RTOS Includes */
/**************************************************************************/
#include "freertos.h"

/**************************************************************************/
/* Library Includes */
/**************************************************************************/

/**************************************************************************/
/* Driver includes */
/**************************************************************************/

/**************************************************************************/
/* Src includes */
/**************************************************************************/
#include "membuf.h"
#include "hostframe.h"
#include "payload.h"
#include "pbuffer.h"
//#include "crc16.h"
#include "hostcomm.h"
//#include "data.h"

#ifdef KUKU
int validHostFrame(HOST_FRAME_HEADER *hostFrame, size_t max_size)
{
	size_t rec_size;
	long tmp;

	if (hostFrame==NULL)
		return 0;
	// Check for size overflow if maximal size specified
	if (max_size)
	{
		// Maximal frame size specified.
		rec_size=hostFrame->length;
		if(*(HOST_FRAME_TYPE(hostFrame))==HOST_ESC)
		{
			if (max_size<=rec_size)
				return 0; // the frame is truncated,  hence invalid. 
			if(encoding.type)
				return checkHostFrameCCS(hostFrame,encoding.pw_csum);
			else
				return checkHostFrameCCS(hostFrame,0);
		}
		else if(*(HOST_FRAME_TYPE(hostFrame))==HOST_ENQ)
		{
			if(encoding.type)
				{
					return checkHostFrameCCS(hostFrame,encoding.pw_csum);
				}
				else
					return 1;
		}
		else if(*(HOST_FRAME_TYPE(hostFrame))==HOST_SYNC0)
			return 1;
		else
			return 0;
	}
}

PAYLOAD_BUFFER_HEADER *extractPayloadsFromHostFrame(HOST_FRAME_HEADER *frame, PAYLOAD_BUFFER_HEADER *cmd_reminder, PAYLOAD_BUFFER_HEADER **pcmd_reminder, MEMBUF_POOL *payloadPool)
{
	PAYLOAD_BUFFER_HEADER *pb=NULL;
	char *dst=NULL;
	char *src=NULL;
	
	uint16_t rec_len=0;
	uint16_t payload_len=0;
	

	if (frame==NULL)
	{
		#ifdef KUKU
		// if no frame, extraction is not in sequence
		if (cmd_reminder)
		{
			// abort reassembly of reminder payload (if any) and return it
			*pcmd_reminder=NULL;
			retMemBuf(cmd_reminder);
		}
		#endif
		return NULL;
	}

	#ifdef KUKU
	if (cmd_reminder)
	{
		rec_len=frame->length;
		rec_copied=0;
		pb=cmd_reminder;
		dst=&(PAYLOAD_DATA(pb))[pb->length];
		src=HOST_FRAME_DATA(frame);
		if (pb->length<sizeof(PAYLOAD_HEADER))
		{
			if ((pb->length+rec_len)<sizeof(PAYLOAD_HEADER))
			{
				
				// Not enough data to reconstruct payload length
				memcpy(dst,src,rec_len);
				pb->length+=rec_len;
				*pcmd_reminder=pb;
				return NULL;
			}
			else 
			{
				// reassemble payload header
				rec_copied=sizeof(PAYLOAD_HEADER)-pb->length;
				memcpy(dst,src,rec_copied);
				pb->length+=rec_copied;
			}
		}
		payload_len=((PAYLOAD_HEADER *)PAYLOAD_DATA(pb))->length;
		if ((rec_len-rec_copied)<((sizeof(PAYLOAD_HEADER)+payload_len)-pb->Length))
		{
			memcpy(dst,src,rec_len-rec_copied);
			pb->length+=rec_len-rec_copied;
			*pcmd_reminder=pb;
			return NULL;
		}
		else
		{
			rec_copied=(sizeof(PAYLOAD_HEADER)+payload_len)-pb->length;
			memcpy(dst,src,rec_copied);
			pb->length+=rec_copied;
			*pcmd_reminder=NULL;
			return pb;
		}
	}
	else
	#endif
	{
	
		// For now all payloads will start at beginning of a frame.
		// A payload may span over one or more records. There will not be multiple payloads in a frame.
		rec_len=frame->length;
		pb=getPayloadBufferWithWait(payloadPool, portMAX_DELAY);
		dst=PAYLOAD_BUFFER_DATA(pb);
		src=HOST_SFRAME_START(frame);
		if (rec_len<(sizeof(PAYLOAD_HEADER)-2))
		{
			#ifdef KUKU
			// there is a reminder
			memcpy(dst, src, rec_len);
			pb->length=rec_len;
			*pcmd_reminder=pb;
			#else
			*pcmd_reminder=NULL;
			#endif
			return NULL;
		}
		else 
		{
			payload_len=((PAYLOAD_HEADER *)frame)->length+sizeof(uint8_t/*PAYLOAD_HEADER*/);
			if (payload_len>payloadPool->bufSize)
			{
				#ifdef KUKU
				// there is a reminder
				memcpy(dst, src, rec_len);
				pb->length=rec_len;
				*pcmd_reminder=pb;
				#endif
				return NULL;
			}
			else
			{
				// the whole payload is in the frame
				memcpy(dst,frame, payload_len);
				pb->length=payload_len;
				*pcmd_reminder=NULL;
				return pb;
			}
		}
	}
}
#endif


