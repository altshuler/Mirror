/*******************************************************************************
 * @file drive_comm.c
 * @ drive_comm 
 *
 * @author Evgeny Altshuler
 *
 * @version 0.0.1
 * @date 09.03.2014
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
#include "stm32f2xx.h"
#include "drive_comm.h"
#include "drive_task.h"
//#include "inthost.h"
//#include "hcmdtask.h"
//#include "data.h"
/**************************************************************************/
/*Declaration of global variables*/
/**************************************************************************/
extern xQueueHandle 	DriveIntQueue;

static void putInBuffer(struct sFromCtl_packetizer *p, char rxChar, TIMESTAMP rxTS);
static void rxFrameResync(struct sFromCtl_packetizer *p);
static void rxFrameSync(struct sFromCtl_packetizer *p, char rxChar, TIMESTAMP rxTS);
static void rxEofFrameResync(struct sFromCtl_packetizer *p);


int handleRxTimeoutFromCtl(struct sFromCtl_packetizer *p)
{
	if (p->rxState==CTL_RX_STATE_SYNC_1)
		return 0;
	p->stat.timeout++;
	rxFrameResync(p);
	if (p->buf)
	{
		//memset(&p->buf->startTimestamp, 0, sizeof(TIMESTAMP));
		//memset(&p->buf->endTimestamp, 0, sizeof(TIMESTAMP));
	}

	return 1;
}



PACKETBUF_HDR *handleRxFromCtl(char rxChar, TIMESTAMP rxTS, struct sFromCtl_packetizer *p)
{
	volatile PACKETBUF_HDR *retPacket=NULL;

	if (p->buf==NULL)
	{
		switch (p->rxState)
		{
		 case CTL_RX_STATE_SYNC_1:
			// Expecting first sync byte, so still can allocate a fresh buffer
			p->buf=getPacketBuffer(p->pool,FIRST_PACKET_SEGMENT|LAST_PACKET_SEGMENT, p->packetType, UNDEFINED_FORMAT, 0);
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
		 case CTL_RX_STATE_SYNC_1:
		 	rxFrameSync(p, rxChar, rxTS);
		 	break;

		 case CTL_RX_STATE_SYNC_2:
			putInBuffer(p, rxChar, rxTS);
			p->rxPayloadReceived++;
			p->buf->format=PI_FORMAT;
			
			if(rxChar==0x0A) //< LF>    end of RX  Packet
			{
				p->prevRxState=p->rxState;
				p->stat.rbin++;
				retPacket=p->buf;
				p->buf=NULL;
				rxFrameResync(p);

			}
			else if (p->rxPayloadReceived>=11)
			{
				p->stat.err_frame++;
				rxFrameResync(p);
				rxFrameSync(p, rxChar, rxTS);
			}

			
			break;

		    default:
			rxFrameResync(p);
			rxFrameSync(p, rxChar, rxTS);
			break;
		}
	}
	else
	{
		// Throw the packet
	}
	p->prevRxChar=rxChar;
	return (PACKETBUF_HDR *)retPacket;
}



void rxFrameResync(struct sFromCtl_packetizer *p)
{
	p->rxIdx=0;
	if (p->buf)
		p->buf->dlen=0;
	p->rxPayloadLength=p->rxPayloadReceived=0;
	p->prevRxState=p->rxState;
	p->rxState=CTL_RX_STATE_SYNC_1;
}

void rxFrameSync(struct sFromCtl_packetizer *p, char rxChar, TIMESTAMP rxTS)
{
//	if (rxChar==PREAMBLE_LO) 
//	{
		// UBX sync 1 
		//p->buf->startTimestamp=p->buf->endTimestamp=rxTS;
		p->rxIdx=0;
		p->rxPayloadLength=p->rxPayloadReceived=0;
		putInBuffer(p, rxChar, rxTS);
		p->prevRxState=p->rxState;
		p->rxState=CTL_RX_STATE_SYNC_2;
//	}
}

void putInBuffer(struct sFromCtl_packetizer *p, char rxChar, TIMESTAMP rxTS)
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

void initCtlTxStat(struct sToCtl_packetizerStat *stat)
{
	memset(stat, 0, sizeof(struct sToCtl_packetizerStat));
}

void updateCtlTxStat(struct sToCtl_packetizerStat *stat, PACKETBUF_HDR*p)
{
	if (p)
	{
		switch (p->format)
		{
		 case PI_FORMAT:
			stat->rbin++;
			break;
		 default:
		 	stat->other++;
		}
	}
}



int handleEofRxTimeoutFromCtl(struct sFromCtl_packetizer *p)
{
	if (p->rxState==CTL_RX_STATE_SYNC_1)
		return 0;
	p->stat.timeout++;
	rxFrameResync(p);
	if (p->buf)
	{
		//memset(&p->buf->startTimestamp, 0, sizeof(TIMESTAMP));
		//memset(&p->buf->endTimestamp, 0, sizeof(TIMESTAMP));
	}
	return 1;
}

void rxEofFrameResync(struct sFromCtl_packetizer *p)
{
	p->rxIdx=0;
	p->rxPayloadLength=p->rxPayloadReceived=0;
	p->prevRxState=p->rxState;
	p->rxState=CTL_RX_STATE_SYNC_1;
}









//static void DriveRxHandler(USART_TypeDef *dev);

/**************************************************************************/
/*Extern Declarations*/
/**************************************************************************/
extern struct sHostInterface intDrive2;
extern struct sHostInterface intDrive3;


/**
* @fn void uart1_RxHandler(void)
*
*   UART interrupts
*
* @author Andrei Mamtsev
*
* @param void
*
* @return void
*
* @date 17.02.2013
*/

#ifdef KUKU
void uart1_RxHandler(void)
{
	DriveRxHandler(USART1);

}

void uart2_RxHandler(void)
{
	DriveRxHandler(USART2);
}


void uart3_RxHandler(void)
{
	DriveRxHandler(USART3);
}

/**
* @fn void driveRxHandler(USART_TypeDef *dev)
*
*  Receive packet Handler
*
* @author Evgeny Altshuler
*
* @param USART_TypeDef *dev-pointer to usart device
*
* @return void
*
* @date 9.03.2014
*/
void DriveRxHandler(USART_TypeDef *dev)
{
	PACKETBUF_HDR *pktBuf=NULL;
	portBASE_TYPE xHigherPriorityTaskWoken= pdFALSE;
	MSG_HDR msg;
	size_t idx;
	size_t rxLen;
	uint8_t localRxBuffer[SERIAL_RX_BUFFER_SIZE];


	
	rxLen=UART_get_rx(dev, localRxBuffer, sizeof(localRxBuffer));
	
	for (idx=0;idx<rxLen;idx++)
	{
		if(dev==USART2)
			pktBuf=handleRxFromDrive(localRxBuffer[idx], 0, &intDrive2.rxPack);
		else if(dev==USART3)
			pktBuf=handleRxFromDrive(localRxBuffer[idx], 0, &intDrive3.rxPack);
		
		if(pktBuf)
		{
			
			
			if(dev==USART2)
			{
				pktBuf->usartnum=DRIVER_1_ID;
				msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_ISR_RX, MSG_TYPE_DRV_1);
			}		
			else if(dev==USART3)
			{
				pktBuf->usartnum=DRIVER_2_ID;
				msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_ISR_RX, MSG_TYPE_DRV_2);
			}
			
			msg.data=0;

			msg.buf=pktBuf;
			
			xQueueSendFromISR(DriveIntQueue,&msg,&xHigherPriorityTaskWoken);
			
			if( xHigherPriorityTaskWoken )
			{
				// Actual macro used here is port specific.
				taskYIELD();
			}
		}
	}
	
}







/**
* @fn PACKETBUF_HDR *handleRxFromDrive(char rxChar, TIMESTAMP rxTS, struct sFromHost_packetizer *p)
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
PACKETBUF_HDR *handleRxFromDrive(char rxChar, TIMESTAMP rxTS, struct sFromHost_packetizer *p)
{
	volatile PACKETBUF_HDR *fromHostPacket=NULL;
	uint8_t temp;
	uint16_t testcomm;
	
	if (p->buf==NULL)
	{
		switch (p->rxState)
		{
		 case PREAMBLE_LSB:
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
		 case PREAMBLE_LSB:
			if (rxChar==PREAMBLE_LO) 	//0x8B
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
				p->rxState=PREAMBLE_MSB;
			#ifdef COMM_TIMEOUT
				MSS_TIM1_load_immediate( 1000000uL );//10msec
				MSS_TIM1_start();
			#endif
			}
			break;
		 case PREAMBLE_MSB:
			
			if (rxChar==PREAMBLE_HI)		//0x3C
			{
				if (p->prevRxChar==((char)PREAMBLE_LO))
				{
					hostPutInBuffer(p, rxChar, rxTS);
					p->prevRxState=p->rxState;
					p->rxState=COMMAND_LSB;
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
					p->rxState=PREAMBLE_LSB;
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
				p->rxState=PREAMBLE_LSB;
			}
			
			break;

		 case COMMAND_LSB:				//Command LSByte
			
			hostPutInBuffer(p, rxChar, rxTS);
			p->prevRxState=p->rxState;
			p->rxState=HOST_FRAME_SIZE;
			
			break;
			
			
		 case COMMAND_MSB:				//Bit0-5  Command MSBits, Bit6-7 sub command LSBits

			testcomm = ((uint16_t)((rxChar&0x3F)<<8))|((uint16_t)p->prevRxChar);
			
		 	if((testcomm >= MIN_CMD)&&(testcomm <= MAX_CMD)) // test valid command 
		 	{
				hostPutInBuffer(p, rxChar, rxTS);
				p->prevRxState=p->rxState;
				p->rxState=ATTRIBUTE;
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
				p->rxState=PREAMBLE_LSB;
			}
			
			break;


		case ATTRIBUTE:		//Bit0-3  sub command MSBits, Bit4- Communication type, Bit5-Data type, Bit6-Error, Bit7- Response
		
			hostPutInBuffer(p, rxChar, rxTS);
			p->prevRxState=p->rxState;
			p->rxState=DATA_3;
			
			break;	

		case DATA_3:
		
			hostPutInBuffer(p, rxChar, rxTS);
			p->prevRxState=p->rxState;
			p->rxState=DATA_2;
			
			break;

		case DATA_2:
			
			hostPutInBuffer(p, rxChar, rxTS);
			p->prevRxState=p->rxState;
			p->rxState=DATA_1;
			
			break;

		case DATA_1:
			
			hostPutInBuffer(p, rxChar, rxTS);
			p->prevRxState=p->rxState;
			p->rxState=DATA_0;
			
			break;	

		case DATA_0:
			
			hostPutInBuffer(p, rxChar, rxTS);
			p->prevRxState=p->rxState;
			p->rxState=CRC_LSB;

			break;	

		case CRC_LSB:
			
			hostPutInBuffer(p, rxChar, rxTS);
			p->prevRxState=p->rxState;
			p->rxState=CRC_MSB;
			
			break;	
			
		case CRC_MSB:

			testcomm = ((uint16_t)rxChar<<8)|((uint16_t)p->prevRxChar);
		
			if(testcomm == calcHostFrameCrc((&(PACKETBUF_DATA(p->buf))[0]),PAYLOAD_LEN))
			{
				hostPutInBuffer(p, rxChar, rxTS);
				fromHostPacket=p->buf;
				p->buf=NULL;
				p->FieldLen=0;
				p->FieldCnt=0;
				p->chksum=0;
				p->ReceivedChksum=0;
				p->rxIdx=0;
				p->rxPayloadLength=p->rxPayloadReceived=0;
				p->prevRxState=p->rxState;
				p->rxState=PREAMBLE_LSB;	
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
				p->rxState=PREAMBLE_LSB;
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

#endif

