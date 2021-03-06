/*******************************************************************************
 * @file Motion_task.c
 * @ Motion   task 
 *
 * @author Evgeny Altshuler
 *
 * @version 0.0.1
 * @date 03.07.2014
 *
*******************************************************************************/

/**************************************************************************/
/* Standard Includes */
/**************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
#include "packetbuf.h"
#include "inthost.h"
#include "data.h"
#include "AbsEncoderSSI.h"
#include "sysport.h"
#include "Motion_task.h"

/**************************************************************************/
/* Extern Declarations */
/**************************************************************************/


#define TARGET_RADIUS	200 //Target Radius in counts

xQueueHandle     MotionQueue;

extern struct sPedestalParams	SysParams;
uSSI AbsEncoderXCnt;
uSSI AbsEncoderYCnt;
uSSI AbsEncoderXData;
uSSI AbsEncoderYData;

	
extern struct sDriverStatus DriveStatus;
extern xQueueHandle 	DriveIntQueue;
extern uint32_t AbsEncXOffset;
extern uint32_t AbsEncYOffset;

struct sEmergencyStatus	EmergStatus;



/**************************************************************************/
/* Creating Motion task for RTOS */
/**************************************************************************/

/**
* @fn void motion_task(void *para)
*
*  Motion Calc   task
*
* @author Evgeny Altshuler
*
* @param void *para
*
* @return void
*
* @date 03.07.2014
*/




void motion_task(void *para)
{
	MSG_HDR read_in_msg;
	MSG_HDR  msg;
	//uint32_t err;
	//uint16_t VelocityCnt=0;
	//float PrevPosition;
	uint32_t key;
	uint8_t SendEmrgCmd=0;

	
	#ifdef TASK_STACK_CHECK
	unsigned portBASE_TYPE uxHighWaterMark;
	
	/* Inspect our own high water mark on entering the task. */
	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
	#endif


	#ifdef KUKU
	/* Initialize  Readout queue*/
	MotionQueue = xQueueCreate( MOTION_QUEUE_SIZE, sizeof(MSG_HDR));
	if( MotionQueue == NULL )
	{
		// Failed to create the queue.
		while(1)
		{
			vTaskSuspend(NULL);
			vTaskDelay(10);
		}
	}
	#endif
	
	//PrevPosition=SysParams.AxisYPosition;
	
		
	memset((uint8_t *)(&EmergStatus),0,sizeof(struct sEmergencyStatus));
	
	while (1)
	{
		if (xQueueReceive( MotionQueue, &read_in_msg, ( portTickType ) portMAX_DELAY ) )
		{
			
			if (read_in_msg.hdr.bit.type==MSG_TYPE_X_ENC)
				AbsEncoderXData=AbsEncoderXCnt;
			else if (read_in_msg.hdr.bit.type==MSG_TYPE_Y_ENC)
				AbsEncoderYData=AbsEncoderYCnt;
			
		
			if(AbsEncoderXData.ssiData.Error!=0x0) //Ranishow enc Err-0, OK-1
			{
			
				key=__disableInterrupts();
				SysParams.AbsEncXStatus=STATUS_OK;
				__restoreInterrupts(key);
			}
			else
			{
				key=__disableInterrupts();
				SysParams.AbsEncXStatus=STATUS_FAIL;
				SysParams.AllOkFlag=STATUS_FAIL;
				__restoreInterrupts(key);

			}

			if(AbsEncoderYData.ssiData.Error!=0x0) //Ranishow enc Err-0, OK-1
			{
			
				key=__disableInterrupts();
				SysParams.AbsEncYStatus=STATUS_OK;
				__restoreInterrupts(key);
			}
			else
			{
				key=__disableInterrupts();
				SysParams.AbsEncYStatus=STATUS_FAIL;
				SysParams.AllOkFlag=STATUS_FAIL;
				__restoreInterrupts(key);

			}				
				PedestalPositionCmd();
			
				
		}

		#ifdef TASK_STACK_CHECK
		/* Calling the function will have used some stack space, we would therefore now expect
		uxTaskGetStackHighWaterMark() to return a value lower than when it was called on
		entering the task. */
		HighWaterMark.uxHighWaterMark_hTxTask=uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
		#endif
		}
}

void PedestalPositionCmd(void)
{
	uint32_t key;

	if(SysParams.State==SYS_STATE_OPERATE)
	{
		if(SysParams.SubState!=SYS_STPR_JOYSTICK)
		{
			key=__disableInterrupts();
			SysParams.ActionXComplete = DriveStatus.MotionXStatus;
			SysParams.ActionYComplete = DriveStatus.MotionYStatus;
			__restoreInterrupts(key);
		}
		
		//if(DriveStatus.MotionXStatus==MOTION_ACTIVE)
		//	SetNextPosition(AbsEncoderXData);
	}
	else if(SysParams.State==SYS_STATE_MAINT)
	{

	}

}


#ifdef KUKU
void SetNextPosition(uSSI Encoder)
{
uint32_t EncoderData;
uint32_t Diff;
uint32_t key;
MSG_HDR  msg;
uint32_t TargetRadius;
uint8_t  EncErrFlag=STATUS_OK;


	if(Encoder.ssiData.Error!=0x0) //Ranishow enc Err-0, OK-1
	{
		EncErrFlag=STATUS_OK;
		if(SysParams.AbsEncStatus!=EncErrFlag)
		{		
			key=__disableInterrupts();
			SysParams.AbsEncStatus=STATUS_OK;
			__restoreInterrupts(key);
		}

		
		TargetRadius=/*((uint32_t)DriveStatus.VelocityCmd)*OVERSHOOT_FACTOR+*/TARGET_RADIUS;
		EncoderData=Encoder.raw32Data>>3;

		
		if(EncoderData>=DriveStatus.TargetPosCmd)
			Diff=EncoderData-DriveStatus.TargetPosCmd;
		else
			Diff=DriveStatus.TargetPosCmd-EncoderData;

		if(Diff<TargetRadius)
		{
		
		}	
	}
	else
	{

		EncErrFlag=STATUS_FAIL;
		if(SysParams.AbsEncStatus!=EncErrFlag)
		{		
			key=__disableInterrupts();
			//DriveStatus.MotionStatus=MOTION_NOT_ACTIVE;
			SysParams.AbsEncStatus=STATUS_FAIL;
			SysParams.AllOkFlag=STATUS_FAIL;
			__restoreInterrupts(key);
		#ifdef KUKU
			msg.data=DRV_STATE_MOTOR_OFF;
			msg.buf=NULL;
		
			msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_1,MSG_TYPE_CMD);
			xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);
			
			msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_2,MSG_TYPE_CMD);
			xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);
		#endif	
		}	
	}
}
#endif
