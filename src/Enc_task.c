/*******************************************************************************
 * @file Enc_task.c
 * @ Encoder transitive  task 
 *
 * @author Andrei Mamtsev
 *
 * @version 0.0.1
 * @date 09.02.2013
 *
*******************************************************************************/

/**************************************************************************/
/* Standard Includes */
/**************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

/**************************************************************************/
/* Driver Includes */
/**************************************************************************/

#include "stm32f2xx_usart.h"
//#include "stm32f2x_dma.h"
//#include "stm32f2x_nvic.h"


/**************************************************************************/
/* RTOS Includes */
/**************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
/**************************************************************************/
/* Src includes */
/**************************************************************************/
#include "main.h"
//#include "msg.h"
//#include "buff.h"
#include "packetbuf.h"
//#include "msgbuf.h"
//#include "msg_type.h"
#include "hcmd.h"
#include "inthost.h"
#include "data.h"
#include "AbsEncoderSSI.h"
#include "sysport.h"
#include "endianutils.h"

/**************************************************************************/
/* Extern Declarations */
/**************************************************************************/
#ifdef KUKU
#define  PEDESTAL_MOVE  	1<<18 //Pedestal Move  :Current pedestal state     0 - stop       1 - moving
#define  PEDESTAL_STOP  	0<<18 

#define  PEDESTAL_POS_DIR  1<<19 //Pedestal Move dir  :Current pedestal dir     0 - to negative edge (to left)       1 - to positive edge (to right)
#define  PEDESTAL_NEG_DIR  0<<19 

#define  PEDESTAL_NEAR_EDGE  	1<<20 //Pedestal in Sector Edge   : 0 - pedestal far from edge  , 1 - near edge (+/- 0.05 deg - TBR)
#define  PEDESTAL_FAR_EDGE  	0<<20 

#define  EDGE_PROXIMITY 	100  //100 counts= 0.0977 deg
#endif
#define READOUT_BUFFERS	5
#define READOUT_BUFFER_SIZE 8

portTickType 	EncCommDly=300; //encoder delay in msec
extern struct sPedestalParams	SysParams;
//extern struct sHostInterface intHost;
//uSSI AbsEncoderCnt;
extern uSSI AbsEncoderXData;
extern uSSI AbsEncoderYData;

extern struct sDriverStatus DriveStatus;
//extern xQueueHandle 	DriveIntQueue;
extern uint32_t AbsEncXOffset;
extern uint32_t AbsEncYOffset;

char readoutBuffersMemory[(READOUT_BUFFER_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*READOUT_BUFFERS];
MEMBUF_POOL readoutBuffers;

static int SendDataToReadout( uint32_t encoder_data);


/**
* @fn void enc_tx_task(void *para)
*
*  Host transitive  task 
*
* @author Andrei Mamtsev
*
* @param void *para
*
* @return void
*
* @date 5.03.2014
*/

//void SetNextPosition(uSSI Encoder);
//void PedestalPositionCmd(void);


void enc_tx_task(void *para)
{
	int32_t enc_data=0;
	uint32_t  tmp_data;
	uint32_t Diff;
	float 	Degrees;
	//uint32_t dir=1;
	//DMA_InitTypeDef DMA_InitStructure;

	
	#ifdef TASK_STACK_CHECK
	unsigned portBASE_TYPE uxHighWaterMark;
	
	/* Inspect our own high water mark on entering the task. */
	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
	#endif

	

	/* Initialize  memory buffers*/
	initMemBufPool(&readoutBuffers,readoutBuffersMemory,sizeof(readoutBuffersMemory), READOUT_BUFFER_SIZE+sizeof(PACKETBUF_HDR),READOUT_BUFFERS);
	
	while (1)
	{

		if(SysParams.State==SYS_STATE_OPERATE)
		{
			
			tmp_data=AbsEncoderXData.raw32Data>>3;

			if(tmp_data>=DriveStatus.TargetPosCmd)
				Diff=tmp_data-DriveStatus.TargetPosCmd;
			else
				Diff=DriveStatus.TargetPosCmd-tmp_data;		
			
			Degrees=CntToDeg(AbsEncoderXData.raw32Data, AbsEncXOffset);
			
			enc_data= (int32_t)(Degrees*100.0);
			enc_data&=0xFFE3FFFF;

			if(DriveStatus.CurrentVelocity>0)
			{
				enc_data |= PEDESTAL_MOVE;
				enc_data |= PEDESTAL_POS_DIR;
			}
			else if(DriveStatus.CurrentVelocity<0)
			{
				enc_data |= PEDESTAL_MOVE;
				enc_data |= PEDESTAL_NEG_DIR;
			}
			else
				enc_data |= PEDESTAL_STOP;

			if(Diff<EDGE_PROXIMITY)
				enc_data |= PEDESTAL_NEAR_EDGE;
			else
				enc_data |= PEDESTAL_FAR_EDGE;

			/* Enable the DMA TX Stream, USART will start sending the command code (4bytes) */
			//DMA_Cmd(USART6_TX_DMA_STREAM, ENABLE);
			SendDataToReadout(enc_data);
		}
		else if(SysParams.State==SYS_STATE_INIT)
		{
			enc_data=0x3FFEC;

			/* Enable the DMA TX Stream, USART will start sending the command code (4bytes) */
			//DMA_Cmd(USART6_TX_DMA_STREAM, ENABLE);
			SendDataToReadout(enc_data);
		}
		else if(SysParams.State==SYS_STATE_STDBY)
					enc_data=0x0;

		
		vTaskDelay(4); //Should be changed according SysParams.RS422MesRate param and/or transferred to TMR ISR.
		
		#ifdef TASK_STACK_CHECK
		/* Calling the function will have used some stack space, we would therefore now expect
		uxTaskGetStackHighWaterMark() to return a value lower than when it was called on
		entering the task. */
		HighWaterMark.uxHighWaterMark_hTxTask=uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
		#endif
		}
}


int SendDataToReadout( uint32_t encoder_data)
{
	char command[5];
	MSG_HDR msg;
	PACKETBUF_HDR *DriverPckt=NULL;


	encoder_data=longBE2LE(encoder_data);
	
	command[0]=sizeof(uint32_t);
	memcpy(&command[1],&encoder_data,sizeof(uint32_t));
	
	DriverPckt=makeSinglePacketResponse(&readoutBuffers, (PAYLOAD_HEADER*)command, RESP_BUFFER_GET_TIMEOUT);
	
	if(DriverPckt!=NULL)
	{
		msg.hdr.all=MAKE_MSG_HDRTYPE(0, MSG_SRC_ENC, MSG_TYPE_PACKET); 
		msg.data=0;
		msg.buf=DriverPckt;
		
		if(readoutOutQ)
		{
			if (pdFAIL==xQueueSend(readoutOutQ,&msg,portMAX_DELAY))
		 	{
		 		retMemBuf(DriverPckt);
				return pdFAIL;  
		 	}
			else
			 	return pdPASS;
		}
		else
	 	{
	 		retMemBuf(DriverPckt);
			return pdFAIL;  
	 	}
	}
	else
		return pdFAIL;	
}


