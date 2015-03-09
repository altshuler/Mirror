/**
  ******************************************************************************
  * @file    stm32f2xx_it.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    07-October-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_it.h"
#include "main.h"
#include "stm32f2x7_eth.h"
#include "data.h"
#include "irqhndl.h"
#include "sysport.h"

/* Scheduler includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* lwip includes */
#include "lwip/sys.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

extern xSemaphoreHandle s_xSemaphore;
#ifdef KUKU

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
#endif

uint32_t isrLevel=0;


/* Private function prototypes -----------------------------------------------*/
extern void xPortSysTickHandler(void); 
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
	int i=1;
  /* Go to infinite loop when Hard Fault exception occurs */
  while (i)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
//void SVC_Handler(void)
//{
//}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//void SysTick_Handler(void)
//{
//  xPortSysTickHandler();
//}




/**
  * @brief  This function handles External line 10 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{

	ENTER_ISR();
	if ((*irqHandler[EXTI15_10_IRQn])(irqHandlerArg[EXTI15_10_IRQn]))
	{
		/* Clear the EXTI lines 15-10 pending bits */
		//EXTI_ClearITPendingBit(EXTI_Line15|EXTI_Line14|EXTI_Line13|EXTI_Line12|EXTI_Line11|EXTI_Line10);
		taskYIELD();
	}
	else
	{
		/* Clear the EXTI lines 15-10 pending bits */
		//EXTI_ClearITPendingBit(EXTI_Line15|EXTI_Line14|EXTI_Line13|EXTI_Line12|EXTI_Line11|EXTI_Line10);
		//taskYIELD();
	}
	EXIT_ISR();

}



/**
  * @brief  This function handles External lines 5 to 9 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{

	ENTER_ISR();
	if ((*irqHandler[EXTI9_5_IRQn])(irqHandlerArg[EXTI9_5_IRQn]))
	{
		/* Clear the EXTI lines 9-5 pending bits */
		//EXTI_ClearITPendingBit(EXTI_Line9|EXTI_Line8|EXTI_Line7|EXTI_Line6|EXTI_Line5);
		taskYIELD();
	}
	else
	{
		/* Clear the EXTI lines 9-5 pending bits */
		//EXTI_ClearITPendingBit(EXTI_Line9|EXTI_Line8|EXTI_Line7|EXTI_Line6|EXTI_Line5);
	}
	EXIT_ISR();
}





/**
  * @brief  This function handles ethernet DMA interrupt request.
  * @param  None
  * @retval None
  */
void ETH_IRQHandler(void)
{
//#ifdef KUKU
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
  if( xHigherPriorityTaskWoken != pdFALSE )
  {
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
  }
//#else
#ifdef KUKU
	ENTER_ISR();
	if ((*irqHandler[ETH_IRQn])(irqHandlerArg[ETH_IRQn]))
		taskYIELD();
	EXIT_ISR();
#endif
//#endif

}

/******************************************************************************/
/*                 STM32F2xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f2xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/


/**
  * @brief  This function handles USART1 interrupt request.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[USART1_IRQn])(irqHandlerArg[USART1_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles USART2 interrupt request.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[USART2_IRQn])(irqHandlerArg[USART2_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles USART3 interrupt request.
  * @param  None
  * @retval None
  */
void USART3_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[USART3_IRQn])(irqHandlerArg[USART3_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles UART4 interrupt request.
  * @param  None
  * @retval None
  */
void UART4_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[UART4_IRQn])(irqHandlerArg[UART4_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles UART5 interrupt request.
  * @param  None
  * @retval None
  */
void UART5_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[UART5_IRQn])(irqHandlerArg[UART5_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles USART6 interrupt request.
  * @param  None
  * @retval None
  */
void USART6_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[USART6_IRQn])(irqHandlerArg[USART6_IRQn]))
		taskYIELD();
	EXIT_ISR();
}



void SPI1_IRQHandler (void)
{
	ENTER_ISR();
	if ((*irqHandler[SPI1_IRQn])(irqHandlerArg[SPI1_IRQn]))
		taskYIELD();
	EXIT_ISR();
}



/**
  * @brief  This function handles SPI1 interrupt request.
  * @param  None
  * @retval None
  */
void SPI2_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[SPI2_IRQn])(irqHandlerArg[SPI2_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles SPI1 interrupt request.
  * @param  None
  * @retval None
  */
void SPI3_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[SPI3_IRQn])(irqHandlerArg[SPI3_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

////==================== I2C interrupt handler ====================================//
void I2C1_EV_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[I2C1_EV_IRQn])(irqHandlerArg[I2C1_EV_IRQn]))
		taskYIELD();
	EXIT_ISR();
}
void I2C1_ER_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[I2C1_ER_IRQn])(irqHandlerArg[I2C1_ER_IRQn]))
		taskYIELD();
	EXIT_ISR();
}
void I2C2_EV_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[I2C2_EV_IRQn])(irqHandlerArg[I2C2_EV_IRQn]))
		taskYIELD();
	EXIT_ISR();
}
void I2C2_ER_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[I2C2_ER_IRQn])(irqHandlerArg[I2C2_ER_IRQn]))
		taskYIELD();
	EXIT_ISR();
}
void I2C3_EV_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[I2C3_EV_IRQn])(irqHandlerArg[I2C3_EV_IRQn]))
		taskYIELD();
	EXIT_ISR();
}
void I2C3_ER_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[I2C3_ER_IRQn])(irqHandlerArg[I2C3_ER_IRQn]))
		taskYIELD();
	EXIT_ISR();
}
////==================== I2C interrupt handler  end ====================================//


/**
  * @brief  This function handles DMA1 stream 0 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Stream0_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA1_Stream0_IRQn])(irqHandlerArg[DMA1_Stream0_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA1 stream 1 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Stream1_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA1_Stream1_IRQn])(irqHandlerArg[DMA1_Stream1_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA1 stream 2 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Stream2_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA1_Stream2_IRQn])(irqHandlerArg[DMA1_Stream2_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA1 stream 3 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Stream3_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA1_Stream3_IRQn])(irqHandlerArg[DMA1_Stream3_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA1 stream 4 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Stream4_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA1_Stream4_IRQn])(irqHandlerArg[DMA1_Stream4_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA1 stream 5 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Stream5_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA1_Stream5_IRQn])(irqHandlerArg[DMA1_Stream5_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA1 stream 6 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Stream6_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA1_Stream6_IRQn])(irqHandlerArg[DMA1_Stream6_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA1 stream 7 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Stream7_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA1_Stream7_IRQn])(irqHandlerArg[DMA1_Stream7_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA2 stream 0 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream0_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA2_Stream0_IRQn])(irqHandlerArg[DMA2_Stream0_IRQn]))
		taskYIELD();
	EXIT_ISR();
}


/**
  * @brief  This function handles DMA2 stream 1 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream1_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA2_Stream1_IRQn])(irqHandlerArg[DMA2_Stream1_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA2 stream 2 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream2_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA2_Stream2_IRQn])(irqHandlerArg[DMA2_Stream2_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA2 stream 3 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream3_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA2_Stream3_IRQn])(irqHandlerArg[DMA2_Stream3_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA1 stream 4 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream4_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA2_Stream4_IRQn])(irqHandlerArg[DMA2_Stream4_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA2 stream 5 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream5_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA2_Stream5_IRQn])(irqHandlerArg[DMA2_Stream5_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA2 stream 6 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream6_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA2_Stream6_IRQn])(irqHandlerArg[DMA2_Stream6_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA2 stream 7 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream7_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA2_Stream7_IRQn])(irqHandlerArg[DMA2_Stream7_IRQn]))
		taskYIELD();
	EXIT_ISR();
}




/**
  * @brief  This function handles TIM4 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM4_IRQHandler(void)
{

	ENTER_ISR();
	if ((*irqHandler[TIM4_IRQn])(irqHandlerArg[TIM4_IRQn]))
		taskYIELD();
	EXIT_ISR();
}





/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[TIM3_IRQn])(irqHandlerArg[TIM3_IRQn]))
		taskYIELD();
	EXIT_ISR();
}


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
