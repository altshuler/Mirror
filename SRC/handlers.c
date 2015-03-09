/*******************************************************************************
 * @file handlers.c
 * @ handlers 
 *
 * @author Evgeny Altshuler
 *
 * @version 0.0.1
 * @date 24.07.2014
 *
*/
/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_it.h"
#include "main.h"
#include "stm32f2x7_eth.h"
#include "data.h"
#include "handlers.h"

/* Scheduler includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* lwip includes */
#include "lwip/sys.h"



/**************************************************************************/
/*Declaration of global variables*/
/**************************************************************************/
extern xSemaphoreHandle s_xSemaphore;
extern xSemaphoreHandle Timer_3_Sem ;
extern xSemaphoreHandle Timer_4_Sem ;

extern __IO uint8_t Tx_Data;				// E.A. Line added
extern __IO uint8_t RxBuffer [10];			// E.A. Line added
extern __IO uint8_t Rx_Idx;			    	// E.A. Line added

uint16_t capture_4 = 0;
uint16_t capture_3 = 0;

extern __IO uint16_t T3_CCR1_Val;
extern __IO uint16_t T4_CCR1_Val;
extern __IO uint16_t CCR2_Val;
extern __IO uint16_t CCR3_Val;
extern __IO uint16_t CCR4_Val;
extern __IO uint16_t Brake_PWM_Val;

extern struct sPedestalParams	SysParams;
extern xQueueHandle 	DriveIntQueue;
extern xQueueHandle     MotionQueue;
extern struct sDriverStatus DriveStatus;
extern uSSI AbsEncoderXCnt;
extern uSSI AbsEncoderYCnt;



uint8_t EncXIntFlag=0;
uint8_t EncYIntFlag=0;
uint8_t Spi_Enc_Flag=0;

extern struct sEmergencyStatus	EmergStatus;
uint32_t enc;

int __eXTI15_10_IRQHandler(void * arg)
{
	int doYield=0;

	  if(EXTI_GetITStatus(ETH_LINK_EXTI_LINE) != RESET)
	  {
		Eth_Link_ITHandler(DP83848_PHY_ADDRESS);
		/* Clear interrupt pending bit */
		EXTI_ClearITPendingBit(ETH_LINK_EXTI_LINE);
	  }

	  return doYield;
}


int __eXTI9_5_IRQHandler(void * arg)
{
	MSG_HDR msg;
	portBASE_TYPE xHigherPriorityTaskWoken= pdFALSE;
	uint8_t EmergFlag=0;
	int doYield=0;
	uint8_t state;
	
	  if(EXTI_GetITStatus(EXTI_Line7) != RESET)
	  {
		#ifdef KUKU
			/* Clear the EXTI line 7 pending bit */
			EXTI_ClearITPendingBit(EXTI_Line7);
			EmergFlag=1;
			SysParams.SafetyState |= EMERGENCY_2_ON;
			SysParams.AllOkFlag=STATUS_FAIL;
		#else
			/* Clear the EXTI line 7 pending bit */
			EXTI_ClearITPendingBit(EXTI_Line7);
			state=GPIO_ReadInputDataBit(EMERGENCY_INTERRUPT2_GPIO_PORT,EMERGENCY_INTERRUPT2_PIN);
			if(state==Bit_SET)
  				EmergStatus.EmergencyFlagsState |= EMERGENCY_2_ON;
			else
				EmergStatus.EmergencyFlagsState &= EMERGENCY_2_OFF;
			
			EmergStatus.Emerg2DebounceTmr=0;
			EmergStatus.Emergency2Flag=EMERGENCY_ACTIVE;
		#endif
	  }
	  else if(EXTI_GetITStatus(EXTI_Line8) != RESET)
	  {
		#ifdef KUKU
		/* Clear the EXTI line 8 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line8);
		EmergFlag=1;
		SysParams.SafetyState |= EMERGENCY_1_ON;
		SysParams.AllOkFlag=STATUS_FAIL;
		#else
			/* Clear the EXTI line 8 pending bit */
			EXTI_ClearITPendingBit(EXTI_Line8);
			state=GPIO_ReadInputDataBit(EMERGENCY_INTERRUPT1_GPIO_PORT,EMERGENCY_INTERRUPT1_PIN);
			if(state==Bit_SET)
  				EmergStatus.EmergencyFlagsState |= EMERGENCY_1_ON;
			else
				EmergStatus.EmergencyFlagsState &= EMERGENCY_1_OFF;
			
			EmergStatus.Emerg1DebounceTmr=0;
			EmergStatus.Emergency1Flag=EMERGENCY_ACTIVE;
		#endif
	  }
	
		if(EmergFlag==1)
		{
			Brake_1_Control(DISABLE);
			Brake_2_Control(DISABLE);
	
			msg.hdr.all=MAKE_MSG_HDRTYPE(0, MSG_SRC_ISR_EMERG_1, MSG_TYPE_CMD);
			msg.data=DRV_STATE_MOTOR_OFF;
			msg.buf=NULL;
			xQueueSendFromISR(DriveIntQueue,&msg,&xHigherPriorityTaskWoken);
			
	
			msg.hdr.all=MAKE_MSG_HDRTYPE(0, MSG_SRC_ISR_EMERG_2, MSG_TYPE_CMD);
			xQueueSendFromISR(DriveIntQueue,&msg,&xHigherPriorityTaskWoken);
			if( xHigherPriorityTaskWoken )
			{
				// Actual macro used here is port specific.
				doYield=1;
			}
		}

	return doYield;
}



int __eTH_IRQHandler(void * arg)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	/* Frame received */
	if ( ETH_GetDMAFlagStatus(ETH_DMA_FLAG_R) == SET) 
	{
	  /* Give the semaphore to wakeup LwIP task */
	  xSemaphoreGiveFromISR( s_xSemaphore, &xHigherPriorityTaskWoken );   
	}
	  
	/* Clear the interrupt flags. */
	/* Clear the Eth DMA Rx IT pending bits */
	ETH_DMAClearITPendingBit(ETH_DMA_IT_R);
	ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);
	  
	/* Switch tasks if necessary. */ 
	return ( xHigherPriorityTaskWoken != pdFALSE ) ? 1 : 0 ;
}




int __sPI1_IRQHandler (void * arg)
{
	MSG_HDR msg;
	portBASE_TYPE xHigherPriorityTaskWoken= pdFALSE;
	int doYield=0;
	
	if (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) == SET)
	  {
	  	SPI_ClearITPendingBit(SPI1,SPI_I2S_IT_RXNE);
		
		
		#ifdef RANISHOW
			
			if(EncXIntFlag==2)
			{
				AbsEncoderXCnt.rawData[3-EncXIntFlag]=(uint16_t)SPI_I2S_ReceiveData(SPI1);
				//AbsEncoderCnt.raw32Data=((AbsEncoderCnt.raw32Data&0xFFFFFFFC)>>3);
				EncXIntFlag=0;
				msg.hdr.all=MAKE_MSG_HDRTYPE(0, MSG_SRC_ISR_TIM, MSG_TYPE_X_ENC);
				xQueueSendFromISR(MotionQueue,&msg,&xHigherPriorityTaskWoken);
				
				if( xHigherPriorityTaskWoken )
				{
					// Actual macro used here is port specific.
					doYield=1;
				}	
			}
			else
			{
				AbsEncoderXCnt.rawData[3-EncXIntFlag]=(uint16_t)SPI_I2S_ReceiveData(SPI1);
				SPI_I2S_SendData(SPI1,0x5555);
				EncXIntFlag++;
			}	
		#else
		EncXIntFlag++;
		
		if(EncXIntFlag==1)
		{
			AbsEncoderXCnt.raw32Data=(uint32_t)SPI_I2S_ReceiveData(SPI1);
			AbsEncoderXCnt.raw32Data=AbsEncoderXCnt.raw32Data<<16;
			SPI_I2S_SendData(SPI1,0x5555);
		}
		else
		{
			EncXIntFlag=0;
			AbsEncoderXCnt.raw32Data|= (uint32_t)SPI_I2S_ReceiveData(SPI1);
			AbsEncoderXCnt.raw32Data=((AbsEncoderXCnt.raw32Data&0x7FFFFFFF)>>3);
			msg.hdr.all=MAKE_MSG_HDRTYPE(0, MSG_SRC_ISR_TIM, MSG_TYPE_ENC);
			xQueueSendFromISR(MotionQueue,&msg,&xHigherPriorityTaskWoken);
			
			if( xHigherPriorityTaskWoken )
			{
				// Actual macro used here is port specific.
				doYield=1;
			}	
		}
		#endif
	  }
	return doYield;
}




int __sPI3_IRQHandler (void * arg)
{
	MSG_HDR msg;
	portBASE_TYPE xHigherPriorityTaskWoken= pdFALSE;
	int doYield=0;
	
	if (SPI_I2S_GetITStatus(SPI3, SPI_I2S_IT_RXNE) == SET)
	  {
	  	SPI_ClearITPendingBit(SPI3,SPI_I2S_IT_RXNE);
		
		
		#ifdef RANISHOW
			
			if(EncYIntFlag==2)
			{
				AbsEncoderYCnt.rawData[3-EncYIntFlag]=(uint16_t)SPI_I2S_ReceiveData(SPI3);
				//AbsEncoderYCnt.raw32Data=AbsEncoderYCnt.raw32Data>>4;
				//AbsEncoderCnt.raw32Data=((AbsEncoderCnt.raw32Data&0xFFFFFFFC)>>3);
				EncYIntFlag=0;
				msg.hdr.all=MAKE_MSG_HDRTYPE(0, MSG_SRC_ISR_TIM, MSG_TYPE_Y_ENC);
				xQueueSendFromISR(MotionQueue,&msg,&xHigherPriorityTaskWoken);
				
				if( xHigherPriorityTaskWoken )
				{
					// Actual macro used here is port specific.
					doYield=1;
				}	
			}
			else
			{
				AbsEncoderYCnt.rawData[3-EncYIntFlag]=(uint16_t)SPI_I2S_ReceiveData(SPI3);
				SPI_I2S_SendData(SPI3,0x5555);
				EncYIntFlag++;
			}	
		#else
		EncYIntFlag++;
		
		if(EncYIntFlag==1)
		{
			AbsEncoderYCnt.raw32Data=(uint32_t)SPI_I2S_ReceiveData(SPI3);
			AbsEncoderYCnt.raw32Data=AbsEncoderYCnt.raw32Data<<16;
			SPI_I2S_SendData(SPI3,0x5555);
		}
		else
		{
			EncYIntFlag=0;
			AbsEncoderYCnt.raw32Data|= (uint32_t)SPI_I2S_ReceiveData(SPI3);
			AbsEncoderYCnt.raw32Data=((AbsEncoderYCnt.raw32Data&0x7FFFFFFF)>>3);
			//msg.hdr.all=MAKE_MSG_HDRTYPE(0, MSG_SRC_ISR_TIM, MSG_TYPE_ENC);
			//xQueueSendFromISR(MotionQueue,&msg,&xHigherPriorityTaskWoken);
			
			if( xHigherPriorityTaskWoken )
			{
				// Actual macro used here is port specific.
				doYield=1;
			}	
		}
		#endif
	  }
	return doYield;
}





int __tIM4_IRQHandler(void * arg)
{

	MSG_HDR msg;
	portBASE_TYPE xHigherPriorityTaskWoken= pdFALSE;
	int doYield=0;
	
	if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)
	{
	  TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
	
	  capture_4 = TIM_GetCapture1(TIM4);
	  TIM_SetCompare1(TIM4, capture_4 + T4_CCR1_Val);
	}
	else if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)
	{
	  TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
	 
	  //capture_4 = TIM_GetCapture2(TIM4);
	  //TIM_SetCompare2(TIM4, capture_4 + CCR2_Val);
	  TIM_ITConfig(TIM4, TIM_IT_CC2, DISABLE);
	}
	else if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET)
	{
	  TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);
	
	  GPIO_ToggleBits(BREAK_PWM_M1_GPIO_PORT, BREAK_PWM_M1_PIN);
	  capture_4 = TIM_GetCapture3(TIM4);
	  TIM_SetCompare3(TIM4, capture_4 + Brake_PWM_Val);
	}
	else
	{
	  TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);
	
	  GPIO_ToggleBits(BREAK_PWM_M1_GPIO_PORT, BREAK_PWM_M2_PIN);
	  capture_4 = TIM_GetCapture4(TIM4);
	  TIM_SetCompare4(TIM4, capture_4 + Brake_PWM_Val);
	}
	
return doYield;
}




int __tIM3_IRQHandler(void * arg)
{
  MSG_HDR msg;
  portBASE_TYPE xHigherPriorityTaskWoken= pdFALSE;
  int doYield=0;
  
  if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
	if(DriveStatus.Drive1PacketSent==0)
	{	
		msg.hdr.all=MAKE_MSG_HDRTYPE(0, MSG_SRC_ISR_TIM, MSG_TYPE_EVENT);
		msg.data=DRV_STATE_MOTOR_GET;
		//xQueueSendFromISR(DriveIntQueue,&msg,&xHigherPriorityTaskWoken);
		if( xHigherPriorityTaskWoken )
		{
			// Actual macro used here is port specific.
			doYield=1;
		}
	}

	capture_3 = TIM_GetCapture1(TIM3);
    TIM_SetCompare1(TIM3, capture_3 + T3_CCR1_Val);
  }
  else if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
	TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);
	xSemaphoreGiveFromISR(Timer_3_Sem,&xHigherPriorityTaskWoken);
	if( xHigherPriorityTaskWoken )
	{
		// Actual macro used here is port specific.
		doYield=1;
	}

  }
  else if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);

    GPIO_ToggleBits(ONE_SHOT1_TRIGN_GPIO_PORT, ONE_SHOT1_TRIGN_PIN | ONE_SHOT2_TRIGN_PIN);
    capture_3 = TIM_GetCapture3(TIM3);
    TIM_SetCompare3(TIM3, capture_3 + CCR3_Val);
  }
  else
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);

	if(Spi_Enc_Flag)
	{
		SPI_I2S_SendData(SPI1,0x5555);
		Spi_Enc_Flag=0x0;
	}
	else
	{
		SPI_I2S_SendData(SPI3,0x5555);
		Spi_Enc_Flag=0x1;
	}		

	capture_3 = TIM_GetCapture4(TIM3);
    TIM_SetCompare4(TIM3, capture_3 + CCR4_Val);
  }

  return doYield;
}




