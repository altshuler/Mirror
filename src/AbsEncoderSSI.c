/*
 * AbsEncoderSSI.c
 *
 *  Created on: Apr 1, 2014
 *      Author: David Anidjar
 */

//#include <stdio.h>
#include <string.h>
#include "board.h"
#include "stm32f2xx_spi.h"
#include "AbsEncoderSSI.h"
#include "sysport.h"
#include "FreeRTOSConfig.h"
#include "handlers.h"
#include "irqhndl.h"



void Sleep(uint32_t microseconds);
SPI_InitTypeDef initSSI;





void init_SSI()
{
	GPIO_InitTypeDef      GPIO_InitStructure1;

	//SPI1 configuration
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* Enable GPIO clocks */
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG|SPI1_SCK_GPIO_CLK | SPI1_MISO_GPIO_CLK, ENABLE);

	GPIO_PinAFConfig(SPI1_SCK_GPIO_PORT, SPI1_SCK_PIN_SOURCE, GPIO_AF_SPI1);
	GPIO_PinAFConfig(SPI1_MISO_GPIO_PORT, SPI1_MISO_PIN_SOURCE, GPIO_AF_SPI1);

	GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure1.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure1.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure1.GPIO_Pin = SPI1_SCK_PIN;
	GPIO_Init(SPI1_SCK_GPIO_PORT, &GPIO_InitStructure1);


	GPIO_InitStructure1.GPIO_Pin = SPI1_MISO_PIN;
	//GPIO_InitStructure1.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	//GPIO_InitStructure1.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(SPI1_MISO_GPIO_PORT, &GPIO_InitStructure1);


	SPI_I2S_DeInit(SPI1);

	initSSI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	initSSI.SPI_DataSize = SPI_DataSize_16b;
	initSSI.SPI_CPOL = SPI_CPOL_High;
	initSSI.SPI_CPHA = SPI_CPHA_1Edge;
	initSSI.SPI_FirstBit = SPI_FirstBit_MSB;
	initSSI.SPI_NSS = SPI_NSS_Soft;
	initSSI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
	initSSI.SPI_Mode = SPI_Mode_Master;
	initSSI.SPI_CRCPolynomial = 7; //??????

	SPI_Init(SPI1,&initSSI);
	//SPI1->CR1 |= SPI_CR1_RXONLY;

	installInterruptHandler(SPI1_IRQn,__sPI1_IRQHandler,NULL);
	
	NVIC_Configuration(SPI1_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY/*0*/, 0);
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
	
	SPI_Cmd(SPI1, ENABLE);
	GPIO_SetBits(ENCODER_CLK_EN_GPIO_PORT,ENCODER_CLK_EN_PIN);



	//SPI3 configuration
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

	GPIO_PinAFConfig(SPI3_CLK_GPIO_PORT, SPI3_CLK_PIN_SOURCE, GPIO_AF_SPI3);
	GPIO_PinAFConfig(SPI3_MISO_GPIO_PORT, SPI3_MISO_PIN_SOURCE, GPIO_AF_SPI3);

	GPIO_InitStructure1.GPIO_Pin = SPI3_CLK_PIN;
	GPIO_Init(SPI3_CLK_GPIO_PORT, &GPIO_InitStructure1);


	GPIO_InitStructure1.GPIO_Pin = SPI3_MISO_PIN;
	GPIO_Init(SPI3_MISO_GPIO_PORT, &GPIO_InitStructure1);


	SPI_I2S_DeInit(SPI3);
	initSSI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPI_Init(SPI3,&initSSI);
	
	installInterruptHandler(SPI3_IRQn,__sPI3_IRQHandler,NULL);
	
	NVIC_Configuration(SPI3_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY/*0*/, 0);
	SPI_I2S_ITConfig(SPI3, SPI_I2S_IT_RXNE, ENABLE);
	
	SPI_Cmd(SPI3, ENABLE);
	GPIO_SetBits(ENCODER_1_CLK_EN_GPIO_PORT,ENCODER_1_CLK_EN_PIN);
}

uint32_t getAngle()
{
	uSSI u_SSI;
	uint32_t key;
	
		memset(u_SSI.rawData,0,4);
	//key=__disableInterrupts();
#ifdef KUKU
		while(SPI_GetFlagStatus(SPI1, SPI_FLAG_TXE)==RESET);
		SPI_I2S_SendData(SPI1,0x55);

		while(SPI_GetFlagStatus(SPI1, SPI_FLAG_RXNE)==RESET);
		u_SSI.rawData[3] = SPI_I2S_ReceiveData(SPI1);

		while(SPI_GetFlagStatus(SPI1, SPI_FLAG_TXE)==RESET);
		SPI_I2S_SendData(SPI1,0x55);

		while(SPI_GetFlagStatus(SPI1, SPI_FLAG_RXNE)==RESET);
		u_SSI.rawData[2] = SPI_I2S_ReceiveData(SPI1);

		while(SPI_GetFlagStatus(SPI1, SPI_FLAG_TXE)==RESET);
		SPI_I2S_SendData(SPI1,0x55);

		while(SPI_GetFlagStatus(SPI1, SPI_FLAG_RXNE)==RESET);
		u_SSI.rawData[1] = SPI_I2S_ReceiveData(SPI1);

		while(SPI_GetFlagStatus(SPI1, SPI_FLAG_TXE)==RESET);
		SPI_I2S_SendData(SPI1,0x55);

		while(SPI_GetFlagStatus(SPI1, SPI_FLAG_RXNE)==RESET);
		u_SSI.rawData[0] = SPI_I2S_ReceiveData(SPI1);
	//__restoreInterrupts(key);
#else
		key=__disableInterrupts();
		while(SPI_GetFlagStatus(SPI1, SPI_FLAG_TXE)==RESET);
		SPI_I2S_SendData(SPI1,0x5555);

		while(SPI_GetFlagStatus(SPI1, SPI_FLAG_RXNE)==RESET);
		u_SSI.raw32Data = (uint32_t)SPI_I2S_ReceiveData(SPI1);
		u_SSI.raw32Data= u_SSI.raw32Data<<16;
		
		while(SPI_GetFlagStatus(SPI1, SPI_FLAG_TXE)==RESET);
		SPI_I2S_SendData(SPI1,0x5555);

		while(SPI_GetFlagStatus(SPI1, SPI_FLAG_RXNE)==RESET);
		u_SSI.raw32Data |= (uint32_t)SPI_I2S_ReceiveData(SPI1);
		__restoreInterrupts(key);
#endif

		//if((u_SSI.ssiData.Error == 1) || (u_SSI.ssiData.Warning == 1))
		//	return -1;

	return ((u_SSI.raw32Data&0x7FFFFFFF)>>3);
}




uSSI getGPIO_SSI_Angle()
{
	int idx;
	uint8_t data;
	uSSI u_SSI;
	uint32_t key;
	
	//printf("Testing");
//	while(1)
//	{
//		Sleep(35000);

		// Set Data Line to "0"
		key=__disableInterrupts();		
		GPIO_ResetBits(SPI1_SCK_GPIO_PORT,SPI1_SCK_PIN);
		
		Sleep(2);
		GPIO_SetBits(SPI1_SCK_GPIO_PORT,SPI1_SCK_PIN);
		Sleep(2);
		u_SSI.raw32Data = 0;
		
		for(idx=0;idx<SSI_NUM_OF_BITS;idx++)
		{		
			u_SSI.raw32Data = u_SSI.raw32Data << 1;
			GPIO_ResetBits(SPI1_SCK_GPIO_PORT,SPI1_SCK_PIN);
			Sleep(2);
			GPIO_SetBits(SPI1_SCK_GPIO_PORT,SPI1_SCK_PIN);
			Sleep(2);
			data = GPIO_ReadInputDataBit(SPI1_MISO_GPIO_PORT, SPI1_MISO_PIN);
			u_SSI.raw32Data |= (uint32_t)data;	
		}
		__restoreInterrupts(key);
		u_SSI.raw32Data = u_SSI.raw32Data >> 1;
//	}
	return u_SSI;
}

void Sleep(uint32_t microseconds)
{
	uint32_t LoopCounter = 0;

	while(microseconds > LoopCounter)
	{
		LoopCounter++;
	}
	//startBootTimer(milliseconds * 2);
	//while(bootTimerExpired() != 1);
}

void NVIC_Configuration(uint8_t ch, uint8_t prio, uint8_t subprio)
{
  NVIC_InitTypeDef  NVIC_InitStructure;  

  NVIC_InitStructure.NVIC_IRQChannel = ch;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = prio;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = subprio;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

