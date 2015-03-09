/*
 * voltage_check.c
 *
 *  Created on: April 7, 2014
 *      Author: David Anidjar
 */

// Temp sensor
//http://www.embedds.com/introducing-to-stm32-adc-programming-part2/
//https://mbed.org/teams/SDK-Development/code/mbed-src/docs/9655231f5786/group__ADC__Group4.html#_details

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "stm32f2xx.h"
#include "board.h"
#include "data.h"
#include "portmacro.h"
#include "voltage_check.h"
#include "AbsEncoderSSI.h"
#include "sysport.h"
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define ADC3_DR_ADDRESS    ((uint32_t)0x4001224C)
#define ADC1_DR_ADDRESS    ((uint32_t)0x4001204C)

#define RESISTOR_RATIO_5V 			((10000.0+1000.0)/1000.0)
#define RESISTOR_RATIO_12V 			((10000.0+1000.0)/1000.0)
#define RESISTOR_RATIO_VM 			((43200.0+1000.0)/1000.0)
#define RESISTOR_RATIO_V_PROTECTED 	((10000.0+470.0)/470.0)

#define ADC_RES_RATIO				3300.0/4095.0

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint16_t ADC3ConvertedValue[5];
__IO uint16_t ADC1ConvertedValue[2];

extern struct sPedestalParams	SysParams;
extern uint16_t Channel1Pulse;
extern uint16_t Channel2Pulse;
extern uint16_t Timer_9_Period;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  check_voltage function
  * @param  None
  * @retval uint32_t Voltage
  */
int check_voltage(void)
{
	uint32_t key;
	uint16_t Vmotor;
	uint8_t  Brake_1;
	uint8_t  Brake_2;
	uint16_t MinPwmVal;

	MinPwmVal = (uint16_t) (((uint32_t) 5 * (Timer_9_Period - 1)) / 10);
	
	key=__disableInterrupts();

	SysParams.V12volt 		= (((float)ADC3ConvertedValue[0] * ADC_RES_RATIO) * RESISTOR_RATIO_12V) / (float)1000;
	SysParams.Vmotor 		= (((float)ADC3ConvertedValue[1] * ADC_RES_RATIO) * RESISTOR_RATIO_VM) / (float)1000;
	SysParams.Vprotected 	= (((float)ADC3ConvertedValue[2] * ADC_RES_RATIO) * RESISTOR_RATIO_V_PROTECTED) / (float)1000;
	SysParams.V5volt 		= (((float)ADC3ConvertedValue[3] * ADC_RES_RATIO) * RESISTOR_RATIO_5V) / (float)1000;
	Vmotor = (uint16_t)(SysParams.Vmotor + 0.5);
	Brake_1 = SysParams.Brake1State;
	Brake_2 = SysParams.Brake2State;
	__restoreInterrupts(key);

	Channel1Pulse = 12000-375*Vmotor;
	
	if(Channel1Pulse>Timer_9_Period)
		Channel1Pulse = Timer_9_Period;
	else if(Channel1Pulse<MinPwmVal)
		Channel1Pulse = MinPwmVal;

	Channel2Pulse = 12000-375*Vmotor;
	
	if(Channel2Pulse>Timer_9_Period)
		Channel2Pulse = Timer_9_Period;
	else if(Channel2Pulse<MinPwmVal)
		Channel2Pulse = MinPwmVal;

	if(!Brake_1)
		TIM9->CCR1 = Channel1Pulse;
	if(!Brake_2)
		TIM9->CCR2 = Channel2Pulse;
	return 0;

	
	
}

/**
  * @brief  ADC3 configuration function
  * @param  None
  * @retval None
  */
void ADC3_Config(void)
{
	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef       DMA_InitStructure;
	GPIO_InitTypeDef      GPIO_InitStructure;



	/* Enable peripheral clocks *************************************************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 , ENABLE);

	/* Configure ADC3 Channel14 pin as analog input ******************************/
	GPIO_InitStructure.GPIO_Pin = CHK_5V_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(CHK_5V_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = CHK_VM_PIN;
	GPIO_Init(CHK_VM_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = CHK_12V_PIN;
	GPIO_Init(CHK_12V_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = V_PROTECTED_PIN;
	GPIO_Init(V_PROTECTED_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_CURRENT_PIN;
	GPIO_Init(PIN_CURRENT_GPIO_PORT, &GPIO_InitStructure);


	/* DMA2 Stream0 channel2 configuration **************************************/
	DMA_InitStructure.DMA_Channel = DMA_Channel_2;
	DMA_InitStructure.DMA_PeripheralBaseAddr =  (uint32_t)ADC3_DR_ADDRESS;//(uint32_t)&ADC3->DR;//
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValue[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = 5;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream0, ENABLE);

	/* ADC Common Init **********************************************************/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* ADC3 Init ****************************************************************/
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 5;
	ADC_Init(ADC3, &ADC_InitStructure);

	// 5V ADC  ==> ADC3  ch 14
	// VM ADC  ==> ADC3  ch 9
	// 12V ADC ==> ADC3  ch 8

	/* ADC3 regular channels configuration ******************************/
	ADC_RegularChannelConfig(ADC3, ADC_Channel_8 , 1, ADC_SampleTime_3Cycles); // 12V
	ADC_RegularChannelConfig(ADC3, ADC_Channel_9 , 2, ADC_SampleTime_3Cycles); // VM
	ADC_RegularChannelConfig(ADC3, ADC_Channel_10, 3, ADC_SampleTime_3Cycles); // VProtected
	ADC_RegularChannelConfig(ADC3, ADC_Channel_14, 4, ADC_SampleTime_3Cycles); // 5V
	ADC_RegularChannelConfig(ADC3, ADC_Channel_7,  5, ADC_SampleTime_3Cycles); // Pin Current
	
	/* Enable DMA request after last transfer (Single-ADC mode) */
	ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);

	/* Enable ADC3 DMA */
	ADC_DMACmd(ADC3, ENABLE);

	/* Enable ADC3 **************************************************************/
	ADC_Cmd(ADC3, ENABLE);

	/* Start ADC3 Software Conversion */
	ADC_SoftwareStartConv(ADC3);
}

void ADC1_Config(void)
{
	ADC_InitTypeDef       	ADC_InitStructure;
	ADC_CommonInitTypeDef 	ADC_CommonInitStructure;
	DMA_InitTypeDef       	DMA_InitStructure;
	GPIO_InitTypeDef		GPIO_InitStructure;

	/* Enable peripheral clocks *************************************************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* Configure ADC1 Channel4 pin as analog input ******************************/
	GPIO_InitStructure.GPIO_Pin = PIN_POS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(PIN_POS_GPIO_PORT, &GPIO_InitStructure);

	/* DMA2 Stream4 channel0 configuration **************************************/
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr =  (uint32_t)ADC1_DR_ADDRESS;//
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ADC1ConvertedValue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = 2;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream4, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream4, ENABLE);


	//ADC_DeInit();
	/* ADC Common Init */
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* ADC1 Init */
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 2;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular channel16 configuration */
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_3Cycles);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 1, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 2, ADC_SampleTime_3Cycles); // PinPos
	ADC_TempSensorVrefintCmd(ENABLE);

	/* Enable DMA request after last transfer (Single-ADC mode) */
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);

	//ADC_TempSensorVrefintCmd(ENABLE);
	//ADC_EOCOnEachRegularChannelCmd(ADC1, ENABLE);
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/* Start ADC1 Software Conversion */
	ADC_SoftwareStartConv(ADC1);
}


void CheckTemperature(void)
{
	float adcVolt;
	float diffVolt;
	float temp;


	adcVolt  = ADC1ConvertedValue[0]*3.3/4095.0;
	diffVolt =   adcVolt - 0.76 ;
	temp = diffVolt*400.0 + 25.0;//0.00025;

	portENTER_CRITICAL() ;
	SysParams.ControllerTemp = temp;
	portEXIT_CRITICAL();
}




//static void delay(__IO uint32_t nCount)
//{
//	__IO uint32_t index = 0;
//	for(index = (100000 * nCount); index != 0; index--)
//	{
//	}
//}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
