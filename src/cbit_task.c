/*******************************************************************************
 * @file Cbit_task.c
 * @ CBIT  task 
 *
 * @author Evgeny Altshuler
 *
 * @version 0.0.1
 * @date 19.05.2014
 *
*******************************************************************************/

/**************************************************************************/
/* Standard Includes */
/**************************************************************************/

#include <stdio.h>
#include <stdlib.h>


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
#include "AbsEncoderSSI.h"
#include "sysport.h"
#include "data.h"
#include "handlers.h"
#include "voltage_check.h"
#include "production_data.h"
#include "flashcsum.h"



extern xQueueHandle 	DriveIntQueue;
extern struct sDriverStatus DriveStatus;
extern struct sPedestalParams	SysParams;
extern uint16_t Channel3Pulse;
extern uint16_t Channel4Pulse;
extern uint8_t Travel_Pin_Cmd;
extern uint8_t Travel_Pin_Pos;
extern __IO uint16_t ADC1ConvertedValue[2];


//extern uSSI AbsEncoderData;


uint32_t AbsEncXOffset=0x57878; 
uint32_t AbsEncYOffset=0x57878;

#define APPLICATION_ADDRESS   (uint32_t)0x08020000
#define USER_FLASH_FIRST_PAGE_ADDRESS 0x08020000 /* Only as example see comment */
#define USER_FLASH_LAST_PAGE_ADDRESS  0x080E0000
#define USER_FLASH_END_ADDRESS        0x080FFFFF

int validApplication(void);


/**
  * @brief  CBITTask
  * @param  None
  * @retval None
  */
void CBITTask(void * pvParameters)
{
	uint8_t  CbitCnt=0;
	uint16_t Brakes_Pwm_period;

	
	SysParams.State=SYS_STATE_STDBY;

	PedestalStatus(NULL);

	#ifdef KUKU
	if(ERROR==InitAbsolutePosition())
	{	
		// Failed to Initialize Absolute positions
		while(1)
		{
			vTaskSuspend(NULL);
			vTaskDelay(10);
		}
	}
	#endif
	
	while(1)
	{
		CbitCnt++;

		if(CbitCnt>5)	//Perform CBIT every second
		{
			CbitCnt=0;
			CheckTemperature();
			
			RamCheck();
			flashCheck();
		}	
			check_voltage();
			

		
	    CheckDiscrets();

		vTaskDelay(200);				
	}	
		
}



/**
  * @brief  Emergency_Int 1 and Emergency_Int 2 configuration.
  * @param None  
  * @retval None
  */
void Emergency_Int_EXTIConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  uint32_t key;  
  uint8_t state;
  
  key=__disableInterrupts();
  
  /* Enable the INT (PE7 PE8) Clock */
  RCC_AHB1PeriphClockCmd(EMERGENCY_INTERRUPT1_GPIO_CLK, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Configure INT pin as input */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = EMERGENCY_INTERRUPT1_PIN | EMERGENCY_INTERRUPT2_PIN;
  GPIO_Init(EMERGENCY_INTERRUPT1_GPIO_PORT, &GPIO_InitStructure);

  state=GPIO_ReadInputDataBit(EMERGENCY_INTERRUPT1_GPIO_PORT,EMERGENCY_INTERRUPT1_PIN);

  if(state==Bit_SET)
  {
  	SysParams.SafetyState |= EMERGENCY_1_ON;
  	SysParams.AllOkFlag=STATUS_FAIL;
  }

  state=GPIO_ReadInputDataBit(EMERGENCY_INTERRUPT1_GPIO_PORT,EMERGENCY_INTERRUPT2_PIN);

  if(state==Bit_SET)
  {
	SysParams.SafetyState |= EMERGENCY_2_ON;
	SysParams.AllOkFlag=STATUS_FAIL;
  }

  /* Connect EXTI Line to INT Pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource7);
  /* Connect EXTI Line to INT Pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource8);
  
  /* Configure Emergency_Int 2 EXTI line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line7 | EXTI_Line8;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  installInterruptHandler(EXTI9_5_IRQn,__eXTI9_5_IRQHandler,NULL);
  
  /* Enable and set the EXTI interrupt to the highest priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_LOWEST_INTERRUPT_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  EXTI_ClearITPendingBit(EXTI_Line7);
  EXTI_ClearITPendingBit(EXTI_Line8);
  __restoreInterrupts(key);
}


uint32_t DegToCnt(float angle, uint32_t offset)
{
	uint32_t ResultCnt;
		
	angle+= 90.0;
	ResultCnt=(uint32_t)(angle*1024.0);
	ResultCnt+=offset;

	if(ResultCnt>ABS_ENC_MAX_VAL)
				ResultCnt=ResultCnt-ABS_ENC_MAX_VAL+1;
			
			
		
return ResultCnt;
}

int32_t deltaCalc(uint32_t Prev, uint32_t Current,uint32_t Max)
{
	int32_t num;
	int32_t half=(Max/2);
	num=(Current-Prev);

	if (num>half)
		num-=Max;
	else if (num<(-half))
		num+=Max;

	return num;
}

void RamCheck(void)
{
	uint32_t RamTestVar = 0;
	uint8_t RamTestRetVal = 0xFF;

	RamTestVar = 0x55555555;
	if(RamTestVar == 0x55555555)
	{
		RamTestVar = ~RamTestVar;
		if(RamTestVar == 0xAAAAAAAA)
			RamTestRetVal = 0;
		else
			RamTestRetVal = 1;
	}
	else
		RamTestRetVal = 1;

	portENTER_CRITICAL();
	SysParams.RamTest = RamTestRetVal;
	portEXIT_CRITICAL();
}

void flashCheck(void)
{
	uint32_t csum;
	//union uBootProductionDataArea tempProductionData;
	uint8_t FlashTestRetVal = 0xFF;

	//memcpy((void *)&tempProductionData,&bootProductionData,sizeof(tempProductionData));
	/* Check validity of factory ids */

	if (isBlank((uint8_t *)&bootProductionData.s.factoryIds, sizeof(bootProductionData.s.factoryIds)))
	{
		//serialPutString(USARTx," Blank\n\r ");
		FlashTestRetVal = 1;
		//return;
	}
	else if ((bootProductionData.s.factoryIds.header.magicNumber!=PRODUCTION_MAGIC_NUMBER) ||
		(bootProductionData.s.factoryIds.header.size!=(sizeof(bootProductionData.s.factoryIds.serialNumber)+sizeof(bootProductionData.s.factoryIds.boardRevision))) ||
		( bootProductionData.s.factoryIds.trailer.checksum!=(csum=calcFlashChecksum(&bootProductionData.s.factoryIds.header,(offsetof(struct sFactoryIds,trailer)+offsetof(struct sProductionInfoTrailer,checksum))))))
	{
		//serialPutString(USARTx," Corrupted\n\r ");
		if (validApplication() == 1)
			FlashTestRetVal = 1;
		else
			FlashTestRetVal = 0;
		//return;
	}
	else
		FlashTestRetVal = 0;
	portENTER_CRITICAL() ;
	SysParams.FlashTest = FlashTestRetVal;
	portEXIT_CRITICAL();
	return;

}



int validApplication(void)
{
	#ifdef APPLICATION_ADDRESS
	uint32_t jumpAddress;
	uint32_t calculatedFlash_checksum;

	/* Check if valid stack address (RAM address) then jump to user application */
	if (((*(__IO uint32_t*)USER_FLASH_FIRST_PAGE_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
	{
		jumpAddress = *(__IO uint32_t *) (APPLICATION_ADDRESS + 4);
		if ((jumpAddress == 0xffffffff) || ((jumpAddress&0x1) == 0))
			return 0;	/* Jump address is empty or even (hence not THUMB-2 instruction address */
		calculatedFlash_checksum = calcFlashChecksum((void *)APPLICATION_ADDRESS,(size_t)&flashChecksum-(size_t)APPLICATION_ADDRESS);
		if (flashChecksum == 0xffffffff)
			return 0; /* flash checksum is blank, no valid application */
		if ((flashChecksum != 0) && (flashChecksum != calculatedFlash_checksum))
			return 0;
	}
	else
		return 0;/* invalid stack address (RAM address)*/
	return 1;
	#else
	return 0;
	#endif
}



int isBlank(uint8_t *p, size_t len)
{
	while (len--)
	{
		if (*p!=0xff)
			return 0;
		p++;
	}
	return 1;
}


void CheckDiscrets(void)
{
	union uDiscrete DiscreteData;
	uint32_t key;
	MSG_HDR msg;	
	MSG_HDR msg1;	

	DiscreteData.bit.discrete_1_4= (uint8_t)((GPIO_ReadInputData(DISCRETE_1_GPIO_PORT)>>9)&0xf);
	DiscreteData.bit.discrete_5_6=(uint8_t)((GPIO_ReadInputData(DISCRETE_5_GPIO_PORT)>>5)&0x3);
	DiscreteData.bit.spare=0x0;
	
	if(SysParams.LimitSwState!=DiscreteData.all)
	{
		vTaskDelay(5);
		DiscreteData.bit.discrete_1_4= (uint8_t)((GPIO_ReadInputData(DISCRETE_1_GPIO_PORT)>>9)&0xf);
		DiscreteData.bit.discrete_5_6=(uint8_t)((GPIO_ReadInputData(DISCRETE_5_GPIO_PORT)>>5)&0x3);
		
		
		if(SysParams.LimitSwState!=DiscreteData.all)
		{
			key=__disableInterrupts();
			SysParams.LimitSwState=DiscreteData.all;
			__restoreInterrupts(key);
		}
	
		
		if(DiscreteData.all!=0x3f)
			msg.data=msg1.data=DRV_STATE_MOTOR_OFF;
		else
			msg.data=msg1.data=DRV_STATE_MOTOR_ON;
		
		msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_1,MSG_TYPE_CMD);
		msg.buf=NULL;
		//xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);
		
		msg1.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_2,MSG_TYPE_CMD);
		msg1.buf=NULL;
		//xQueueSend(DriveIntQueue,&msg1,portMAX_DELAY);
	}		
}




void GetTravelPinPosition(void)
{
uint8_t i;

//	for(i=0;i<100;i++)
//	{
		if(ADC1ConvertedValue[1]>PIN_CLOSED_POSITION)
			Travel_Pin_Pos=PIN_POS_CLOSE;
		
		else if(ADC1ConvertedValue[1]<PIN_OPENED_POSITION)
			Travel_Pin_Pos=PIN_POS_OPEN;

		else
		{
			Travel_Pin_Cmd=PIN_CMD_OPEN;
			GPIO_SetBits(PIN_ON_GPIO_PORT, PIN_ON_PIN); //Enable Travel Pin Bridge
			TIM4->CCR3 = 0x0;			//Open Travel Pin
			TIM4->CCR4 = Channel4Pulse;
		}

//	}	
}


