/*******************************************************************************
 * @file Hostcomm.c
 * @ Hostcomm 
 *
 * @author Andrei Mamtsev
 *
 * @version 0.0.1
 * @date 09.02.2013
 *

**************************************************************************/
/* Standard Includes */
/**************************************************************************/
#include <stdint.h>
#include <stddef.h>
#include <string.h>
/**************************************************************************/
/* RTOS Includes */
/**************************************************************************/
#include "freertos.h"
#include "queue.h"
#include "task.h"
/**************************************************************************/
/* Library Includes */
/**************************************************************************/
//#include "stm32f10x_map.h"
/**************************************************************************/
/* Driver includes */
/**************************************************************************/


/**************************************************************************/
/* Src includes */
/**************************************************************************/
#include "timestamp.h"
#include "packetbuf.h"
#include "hostcomm.h"
#include "msg_type.h"
#include "inthost.h"
#include "hcmdtask.h"
//#include "data.h"
/**************************************************************************/
/*Declaration of global variables*/
/**************************************************************************/

char 		net_addr=48;
uint32_t 	SN_L=0;
uint16_t	SN_H=0;


//static void hostPutInBuffer(struct sFromHost_packetizer *p, char rxChar, TIMESTAMP rxTS);


/**
* @fn PACKETBUF_HDR *handleRxFromHost(char rxChar, TIMESTAMP rxTS, struct sFromHost_packetizer *p)
*
* Receive packet statemachine
*
* @author Evgeny Altshuler
*
* @param char rxChar-Receive char
* @param TIMESTAMP rxTS-timestamp
* @param struct sFromHost_packetizer *p-pointer to rx packetizer
*
* @return PACKETBUF_HDR * - Receive packet
*
* @date 30.01.2014
*/
PACKETBUF_HDR *handleRxFromHost(char rxChar, TIMESTAMP rxTS, struct sFromHost_packetizer *p)
{
	volatile PACKETBUF_HDR *fromHostPacket=NULL;
	uint8_t temp;
	
	if (p->buf==NULL)
	{
		switch (p->rxState)
		{
		 case HOST_RX_SYNC_LO:
			// Expecting first sync byte, so still can allocate a fresh buffer
			p->buf=getPacketBuffer(p->pool,FIRST_PACKET_SEGMENT|LAST_PACKET_SEGMENT, HOST_NETWORK, UNDEFINED_FORMAT, 0);
			if (p->buf==NULL)
				p->stat.no_buffers++;
			break;
		}
	}
	if (p->buf)
	{
		// Do packet reception
		switch (p->rxState)
		{
		 case HOST_RX_SYNC_LO:
			if (rxChar==SYNC_LO) 	//0x55
			{
				p->FieldLen=0;
				p->FieldCnt=0;
				p->chksum=0;
				p->ReceivedChksum=0;
				p->FrameSize=0;
				p->rxIdx=0;
				p->buf->dlen=0;
				p->rxPayloadLength=p->rxPayloadReceived=0;
				hostPutInBuffer(p, rxChar, rxTS);
				p->prevRxState=p->rxState;
				p->CommProtType=SYNC_FORMAT;
				p->rxState=HOST_RX_SYNC_HI;
			#ifdef COMM_TIMEOUT
				MSS_TIM1_load_immediate( 1000000uL );//10msec
				MSS_TIM1_start();
			#endif
			}
			break;
		 case HOST_RX_SYNC_HI:
			if (p->prevRxChar!=(PACKETBUF_DATA(p->buf))[0])
				p->stat.err_frame++;
			else if(p->CommProtType==SYNC_FORMAT)
			{
				if (rxChar==SYNC_HI)		//0xAA
				{
					if (p->prevRxChar==((char)SYNC_LO))
					{
						hostPutInBuffer(p, rxChar, rxTS);
						p->FieldLen=SHORT_LEN;
						p->FieldCnt=0;
						p->prevRxState=p->rxState;
						p->rxState=HOST_CHECKSM;
					}
					else
					{
						// No frame sync
						p->FieldLen=0;
						p->FieldCnt=0;
						p->chksum=0;
						p->ReceivedChksum=0;
						p->FrameSize=0;
						p->rxIdx=0;
						p->buf->dlen=0;
						p->rxPayloadLength=p->rxPayloadReceived=0;
						p->prevRxState=p->rxState;
						p->rxState=HOST_RX_SYNC_LO;
					}
				}
				else
				{
					// No frame sync
					p->FieldLen=0;
					p->FieldCnt=0;
					p->chksum=0;
					p->ReceivedChksum=0;
					p->FrameSize=0;
					p->rxIdx=0;
					p->buf->dlen=0;
					p->rxPayloadLength=p->rxPayloadReceived=0;
					p->prevRxState=p->rxState;
					p->rxState=HOST_RX_SYNC_LO;
				}
			}
			else
			{
			// No frame sync
			p->FieldLen=0;
			p->FieldCnt=0;
			p->chksum=0;
			p->ReceivedChksum=0;
			p->FrameSize=0;
			p->rxIdx=0;
			p->buf->dlen=0;
			p->rxPayloadLength=p->rxPayloadReceived=0;
			p->prevRxState=p->rxState;
			p->rxState=HOST_RX_SYNC_LO;
			}
			
			break;

		 case HOST_CHECKSM:
			
			hostPutInBuffer(p, rxChar, rxTS);
			temp=p->FieldCnt*8;
			p->ReceivedChksum|=((((uint16_t)rxChar)<<temp)&((uint16_t)(0xff<<temp)));
			p->FieldCnt++;
			
			if(p->FieldCnt==p->FieldLen)
			{
				p->FieldLen=LONG_LEN;
				p->FieldCnt=0;
				p->prevRxState=p->rxState;
				p->rxState=HOST_FRAME_SIZE;
			}
			break;
			
			
		 case HOST_FRAME_SIZE:
		
			hostPutInBuffer(p, rxChar, rxTS);
			temp=p->FieldCnt*8;
			p->FrameSize |=((((uint16_t)rxChar)<<temp)&((uint16_t)(0xff<<temp)));
			p->FieldCnt++;
			
			if(p->FieldCnt==p->FieldLen)
			{
				p->buf->rsvd=0;
				p->FieldLen=LONG_LEN;
				p->FieldCnt=0;
				p->prevRxState=p->rxState;
				p->rxState=HOST_OPCODE;
			}
			break;


		case HOST_OPCODE:
		
			hostPutInBuffer(p, rxChar, rxTS);
			temp=p->FieldCnt*8;
			p->buf->rsvd|=((((long)rxChar)<<temp)&((long)(0xff<<temp)));
			p->FieldCnt++;
			
			if(p->FieldCnt==p->FieldLen)
			{
				p->FieldLen=LONG_LEN;
				p->FieldCnt=0;
				p->prevRxState=p->rxState;
				p->rxState=HOST_SOURCE_ID;
			}
			break;	

		case HOST_SOURCE_ID:
		
			hostPutInBuffer(p, rxChar, rxTS);
			p->FieldCnt++;
			
			if(p->FieldCnt==p->FieldLen)
			{
				p->FieldLen=LONG_LEN;
				p->FieldCnt=0;
				p->prevRxState=p->rxState;
				p->rxState=HOST_DEST_ID;
			}
			break;

		case HOST_DEST_ID:
			
			hostPutInBuffer(p, rxChar, rxTS);
			p->FieldCnt++;
			
			if(p->FieldCnt==p->FieldLen)
			{
				p->FieldLen=LONG_LEN;
				p->FieldCnt=0;
				p->prevRxState=p->rxState;
				p->rxState=HOST_RESERVED_1;
			}
			break;

		case HOST_RESERVED_1:
			
			hostPutInBuffer(p, rxChar, rxTS);
			p->FieldCnt++;
			
			if(p->FieldCnt==p->FieldLen)
			{
				p->FieldLen=LONG_LEN;
				p->FieldCnt=0;
				p->prevRxState=p->rxState;
				p->rxState=HOST_RESERVED_2;
			}
			break;	

		case HOST_RESERVED_2:
			
			hostPutInBuffer(p, rxChar, rxTS);
			p->FieldCnt++;
			
			if(p->FieldCnt==p->FieldLen)
			{
				p->FieldLen=LONG_LEN;
				p->FieldCnt=0;
				p->prevRxState=p->rxState;
				p->rxState=HOST_MESSAGE_CNT;
			}
			break;	

		case HOST_MESSAGE_CNT:
			
			hostPutInBuffer(p, rxChar, rxTS);
			p->FieldCnt++;
			
			if(p->FieldCnt==p->FieldLen)
			{
				p->FieldLen=LONG_LEN;
				p->FieldCnt=0;
				p->prevRxState=p->rxState;
				p->rxState=HOST_RESERVED_3;
			}
			break;	
			
		case HOST_RESERVED_3:
			
			hostPutInBuffer(p, rxChar, rxTS);
			p->FieldCnt++;
			
			if(p->FieldCnt==p->FieldLen)
			{
				p->FieldLen=LONG_LEN;
				p->FieldCnt=0;
				p->prevRxState=p->rxState;
				p->rxState=HOST_RESERVED_4;
			}
			break;	


		case HOST_RESERVED_4:
			
			hostPutInBuffer(p, rxChar, rxTS);
			p->FieldCnt++;
			
			if(p->FieldCnt == p->FieldLen)
			{
				if(p->rxIdx == p->FrameSize)
				{
					
					fromHostPacket=p->buf;
					p->buf=NULL;
					p->FieldLen=0;
					p->FieldCnt=0;
					p->chksum=0;
					p->ReceivedChksum=0;
					p->rxIdx=0;
					p->rxPayloadLength=p->rxPayloadReceived=0;
					p->prevRxState=p->rxState;
					p->rxState=HOST_RX_SYNC_LO;	
				}
				else if(p->rxIdx < p->FrameSize)
				{
					p->prevRxState=p->rxState;
					p->rxState=HOST_MESSAGE_BODY;
				}	
				else 
				{
					// Checksum error
					p->FieldLen=0;
					p->FieldCnt=0;
					p->chksum=0;
					p->ReceivedChksum=0;
					p->rxIdx=0;
					p->buf->dlen=0;
					p->rxPayloadLength=p->rxPayloadReceived=0;
					p->prevRxState=p->rxState;
					p->rxState=HOST_RX_SYNC_LO;
				}
				
			}
			break;	


		case HOST_MESSAGE_BODY:
			
			hostPutInBuffer(p, rxChar, rxTS);
			p->chksum+=rxChar;

			if(p->rxIdx>= p->FrameSize)
				{
					if(p->chksum==p->ReceivedChksum)
					{
					   fromHostPacket=p->buf;
					   p->buf=NULL;
					   p->rxIdx=0;
					   p->chksum=0;
					   p->ReceivedChksum=0;
					   p->rxPayloadLength=p->rxPayloadReceived=0;
					   p->prevRxState=p->rxState;
					   p->rxState=HOST_RX_SYNC_LO;
					}
					else
					{
						// Checksum error
						p->FieldLen=0;
						p->FieldCnt=0;
						p->chksum=0;
						p->ReceivedChksum=0;
						p->rxIdx=0;
						p->buf->dlen=0;
						p->rxPayloadLength=p->rxPayloadReceived=0;
						p->prevRxState=p->rxState;
						p->rxState=HOST_RX_SYNC_LO;
					}
					
				}

			
			
			break;	
		}
	}
	else
	{
		// Throw the packet
	}
	p->prevRxChar=rxChar;
	return (PACKETBUF_HDR *)fromHostPacket;
}




void hostPutInBuffer(struct sFromHost_packetizer *p, char rxChar, TIMESTAMP rxTS)
{
	p->lastPutInBuf=p->buf;
	p->lastPutInBufIdx=p->rxIdx;
	if ((p->rxIdx+1)<p->bufSize)	// leave one space in the end for terminating '\0'
	{
		//p->buf->endTimestamp=rxTS;
		(PACKETBUF_DATA(p->buf))[p->rxIdx]=rxChar;
		p->rxIdx++;
		p->buf->dlen++;
	}

}

void initHostTxStat(struct sToHost_packetizerStat *stat)
{
	memset(stat, 0, sizeof(struct sToHost_packetizerStat));
}


