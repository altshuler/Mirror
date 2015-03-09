/**
* @file st_usart.c
* @brief usart IO driver
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 08.05.2012
*/

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <iodev.h>
#include <sysport.h>
#include <freertos.h>
#include <task.h>
#include <uart.h>
#include "st_usart.h"
#include <board.h>
#include <irqhndl.h>
#ifdef USE_USART_DRIVER_TIMESTAMPS
#include <timebase.h>
#endif
#include <dma.h>
#include "../st_dma/st_dma.h"

#if ((defined(USART1_RX_PIN) && defined(USART1_RX_GPIO_PORT) && defined(USART1_RX_GPIO_CLK)  && defined(USART1_RX_PIN_SOURCE)) && (defined(USART1_TX_PIN) && defined(USART1_TX_GPIO_PORT) && defined(USART1_TX_GPIO_CLK)  && defined(USART1_TX_PIN_SOURCE)))
	#define USE_USART1_GPIO
#endif

#if ((defined(USART2_RX_PIN) && defined(USART2_RX_GPIO_PORT) && defined(USART2_RX_GPIO_CLK)  && defined(USART2_RX_PIN_SOURCE)) && (defined(USART2_TX_PIN) && defined(USART2_TX_GPIO_PORT) && defined(USART2_TX_GPIO_CLK)  && defined(USART2_TX_PIN_SOURCE)))
	#define USE_USART2_GPIO
#endif

#if ((defined(USART3_RX_PIN) && defined(USART3_RX_GPIO_PORT) && defined(USART3_RX_GPIO_CLK)  && defined(USART3_RX_PIN_SOURCE)) && (defined(USART3_TX_PIN) && defined(USART3_TX_GPIO_PORT) && defined(USART3_TX_GPIO_CLK)  && defined(USART3_TX_PIN_SOURCE)))
	#define USE_USART3_GPIO
#endif

#if ((defined(USART4_RX_PIN) && defined(USART4_RX_GPIO_PORT) && defined(USART4_RX_GPIO_CLK)  && defined(USART4_RX_PIN_SOURCE)) && (defined(USART4_TX_PIN) && defined(USART4_TX_GPIO_PORT) && defined(USART4_TX_GPIO_CLK)  && defined(USART4_TX_PIN_SOURCE)))
	#define USE_USART4_GPIO
#endif

#if ((defined(USART5_RX_PIN) && defined(USART5_RX_GPIO_PORT) && defined(USART5_RX_GPIO_CLK)  && defined(USART5_RX_PIN_SOURCE)) && (defined(USART5_TX_PIN) && defined(USART5_TX_GPIO_PORT) && defined(USART5_TX_GPIO_CLK)  && defined(USART5_TX_PIN_SOURCE)))
	#define USE_USART5_GPIO
#endif

#if ((defined(USART6_RX_PIN) && defined(USART6_RX_GPIO_PORT) && defined(USART6_RX_GPIO_CLK)  && defined(USART6_RX_PIN_SOURCE)) && (defined(USART6_TX_PIN) && defined(USART6_TX_GPIO_PORT) && defined(USART6_TX_GPIO_CLK)  && defined(USART6_TX_PIN_SOURCE)))
	#define USE_USART6_GPIO
#endif


#define USART_MODULE_CLOCK	((uint32_t)120000000)

const struct Uart_Params UART_PARAMS = 
{
    0,							/* cacheEnable		*/
    0,							/* fifoEnable			*/
    Uart_OpMode_INTERRUPT_DMA,	/* opMode			*/
    0,							/* loopbackEnabled	*/
    Uart_BaudRate_9_6K,			/* baudRate			*/
    Uart_NumStopBits_1,			/* stopBits			*/
    Uart_CharLen_8,				/* charLen			*/
    Uart_Parity_NONE,			/* parity				*/
	Uart_FcType_NONE,			/* fc				*/									   
	1,							/* rxThreshold		*/
	1,							/* txThreshold		*/
    0xffffffff,					/* polledModeTimeout	*/
    1
};

/* ========================================================================== */
/*                       GLOBAL MODULE STATE                                  */
/* ========================================================================== */
/**
 *  \brief  Array which is part of Uart Module State
 */
static uint8_t inUse[N_USART];
/**
 *  \brief  Usart Module State Object
 */
static Usart_Module_State Usart_module = {&inUse[0]};
/**
 *  \brief  Array of Usart instance State objects array
 */
static Usart_Object Usart_Instances[N_USART];



static int stUsartBind(void **devp, int devid, void *devParams);
static int stUsartUnBind(void * devp);
static int stUsartControl(void * chanp, unsigned int cmd, void *args);
static int stUsartCreate(void **chanp, void *devp, char *name, int mode,
                      void *chanParams, IODEV_TiomCallback cbFxn, void *cbArg);
static int stUsartDelete(void *chanp);
static int stUsartSubmit(void *chanp, IODEV_Packet *packet);

#ifndef DRV_DISABLE_INPUT_PARAMETER_CHECK
static int usartValidateParams(const Uart_Params *openParams);
#endif

static void usartConfigure(Usart_Object *instHandle, const Usart_DevParams  *openParams);
static int usartSubmitIoReq(Usart_ChanObj *chanHandle, IODEV_Packet *ioPacket);
static void usartCancelCurrentIo(Usart_ChanObj *chanHandle, IODEV_Packet *ioPacket);
//static void usartEnableHwFlowControl(Usart_Object *instHandle, const Usart_FlowControl  *pFlowCtrl);
//static uint32_t sciXfer(Usart_ChanObj *chanHandle, uint8_t *buffer, uint32_t numBytes);

static int usartSubmitIoReqPolled(Usart_ChanObj *chanHandle, IODEV_Packet *ioPacket);
                                   
#ifdef Usart_TX_BUFFERING_ENABLE
static int usartSubmitIoReqBuffering(Usart_ChanObj *chanHandle, IODEV_Packet *ioPacket);
#endif                                      

static int usartSubmitIoReqIntDma(Usart_ChanObj *chanHandle, IODEV_Packet *ioPacket, uint32_t key);
static void usartCancelAllIo(Usart_ChanObj *chanHandle);
static int usartOsCheckTimeOut(portTickType startValue, portTickType timeout);
static void usartPoll(Usart_ChanObj *chanHandle);

static void Usart_loadPendedIops(Usart_ChanObj  *chanHandle);

static void usartSetBaudRate(Usart_Object *instHandle, uint32_t USART_BaudRate);

static int stUsartIsr(void *arg);

#ifdef Usart_DMA_ENABLE
static int usartStartDmaTxTransfer(Usart_ChanObj *chanHandle);
static int usartStartDmaRxTransfer(Usart_ChanObj *chanHandle);
static int usart_localStopDmaTransfer(Usart_ChanObj *chanHandle, void *arg);
static int stUsartDmaTxIsr(void *arg);
static int stUsartDmaRxIsr(void *arg);
#ifdef USE_USART_DRIVER_TIMESTAMPS
uint32_t calcUsartBlockTime(Usart_Object *instHandle, size_t blkLen);
#endif
#ifdef USE_USART_DRIVER_TIMESTAMPS
uint32_t calcUsartBlockWithIdleTime(Usart_Object *instHandle, size_t blkLen);
#endif
#endif



const IODEV_Fxns Usart_IODEVFXNS =
{
	stUsartBind,
	stUsartUnBind,
	stUsartControl,
	stUsartCreate,
	stUsartDelete,
	stUsartSubmit
		
};

/**
 *  \brief  Function called by OS during instance initialization
 *
 *  \return None
 */
void stUsartInit(void)
{
    int i;
    
    for (i = 0; i < N_USART; i++)
    {
        /* have to initialize statically */
        Usart_module.inUse[i] = 0;
        memset((void *)&Usart_Instances[i], 0x0, sizeof(Usart_Object));
    }
}



int stUsartBind(void **devp, int devid, void *devParams)
{
	int retVal = IODEV_COMPLETED;
	Uart_Params	*params = NULL;
	Usart_Object *instHandle = NULL;   
	GPIO_InitTypeDef gpioInit;
	NVIC_InitTypeDef nvicInit;
	
	/* Begin parameter checking 												  */
	#ifndef DRV_DISABLE_INPUT_PARAMETER_CHECK
	if ((NULL == devParams) || (N_USART <= devid))
		retVal = IODEV_EBADARGS;
	else 
	{
		params = (Uart_Params *)devParams;
		instHandle =  &Usart_Instances[devid];
		
		if ((0 != Usart_module.inUse[devid]) ||
			(Usart_DriverState_DELETED != instHandle->devState)  ||
			(IODEV_COMPLETED != usartValidateParams(params)))
		{
			/* Driver not in deleted (initial) state or in use				  */
			retVal = IODEV_EBADMODE;
		}
	}
	#endif  /* DRV_DISABLE_INPUT_PARAMETER_CHECK */
	/* End parameter checking													  */


	if (IODEV_COMPLETED == retVal)
	{
		params = (Uart_Params *)devParams;

		instHandle = &Usart_Instances[devid];
			
		/* set the status of the module as in use							  */
		Usart_module.inUse[devid] = 1;

		instHandle->rxCallback.f=NULL;
		instHandle->rxCallback.arg=NULL;
	   
		/* initialize the soc specific information							  */
		instHandle->deviceInfo.inputFrequency	 = USART_MODULE_CLOCK;

		instHandle->hDmaRes=NULL;		
		
		switch (devid) 
		{
		 case 0:
			instHandle->deviceInfo.baseAddress = USART1;
			instHandle->deviceInfo.cpuEventNumber = USART1_IRQn;
			break;
		 case 1:
			instHandle->deviceInfo.baseAddress = USART2;
			instHandle->deviceInfo.cpuEventNumber = USART2_IRQn;
			break;
		 case 2:
			instHandle->deviceInfo.baseAddress = USART3;
			instHandle->deviceInfo.cpuEventNumber = USART3_IRQn;
			break;
		 case 3:
			instHandle->deviceInfo.baseAddress = UART4;
			instHandle->deviceInfo.cpuEventNumber = UART4_IRQn;
			break;
		 case 4:
			instHandle->deviceInfo.baseAddress = UART5;
			instHandle->deviceInfo.cpuEventNumber = UART5_IRQn;
			break;
		 case 5:
			instHandle->deviceInfo.baseAddress = USART6;
			instHandle->deviceInfo.cpuEventNumber = USART6_IRQn;
			break;
		 default:
		 	break;
		}
		
		
		/* Initailize Tx Channel members									  */
		instHandle->xmtChanObj.status = Usart_DriverState_CLOSED;
		instHandle->xmtChanObj.mode = Usart_OUTPUT;
		instHandle->xmtChanObj.cbFxn = NULL;
		instHandle->xmtChanObj.cbArg = NULL;
		QUE_new(&(instHandle->xmtChanObj.queuePendingList));
		instHandle->xmtChanObj.activeIOP = NULL;
		instHandle->xmtChanObj.bytesRemaining = 0;
		instHandle->xmtChanObj.chunkSize = 0;
		instHandle->xmtChanObj.devHandle = NULL;
		instHandle->xmtChanObj.errors = 0;
		instHandle->xmtChanObj.hDma= NULL;
		instHandle->xmtChanObj.hDmaHandler= NULL;
		instHandle->xmtChanObj.hDmaHandlerArg= NULL;
		

		memset(&instHandle->xmtChanObj.iBuffer.buffer[0],0x00, Usart_MAX_BUFFER_SIZE);

		instHandle->xmtChanObj.iBuffer.bufFillPtr  = NULL;
		instHandle->xmtChanObj.iBuffer.bufReadPtr  = NULL;
		instHandle->xmtChanObj.iBuffer.bufferIndex = 0;
		instHandle->xmtChanObj.iBuffer.xferActual  = 0;

		/* Initailize Rx Channel members									  */
		instHandle->rcvChanObj.status = Usart_DriverState_CLOSED;
		instHandle->rcvChanObj.mode = Usart_INPUT;
		instHandle->rcvChanObj.cbFxn = NULL;
		instHandle->rcvChanObj.cbArg = NULL;
		QUE_new(&(instHandle->rcvChanObj.queuePendingList));
		instHandle->rcvChanObj.activeIOP = NULL;
		instHandle->rcvChanObj.bytesRemaining = 0;
		instHandle->rcvChanObj.chunkSize = 0;
		instHandle->rcvChanObj.devHandle = NULL;
		instHandle->rcvChanObj.errors = 0;
		instHandle->rcvChanObj.hDma= NULL;
		instHandle->rcvChanObj.hDmaHandler= NULL;
		instHandle->rcvChanObj.hDmaHandlerArg= NULL;

		memset(&instHandle->rcvChanObj.iBuffer.buffer[0],0x00, Usart_MAX_BUFFER_SIZE);

		instHandle->rcvChanObj.iBuffer.bufFillPtr  = NULL;
		instHandle->rcvChanObj.iBuffer.bufReadPtr  = NULL;
		instHandle->rcvChanObj.iBuffer.bufferIndex = 0;
		instHandle->rcvChanObj.iBuffer.xferActual  = 0;

		/*Inintialize Statistics members									  */
		instHandle->stats.rxBytes = 0;
		instHandle->stats.txBytes = 0;
		instHandle->stats.overrun = 0;
		instHandle->stats.rxTimeout = 0;
		instHandle->stats.rxFramingError = 0;
		instHandle->stats.rxBreakError = 0;
		instHandle->stats.rxParityError = 0;

		instHandle->devParams.fifoEnable = params->fifoEnable;
		instHandle->devParams.loopbackEnabled = params->loopbackEnabled;
		instHandle->devParams.baudRate = params->baudRate;
		instHandle->devParams.stopBits = params->stopBits;
		instHandle->devParams.charLen = params->charLen;
		instHandle->devParams.parity = params->parity;
		//instHandle->devParams.rxThreshold = params->rxThreshold;
		instHandle->devParams.fc = params->fc;
		instHandle->devParams.fifoEnable = params->fifoEnable;
		instHandle->polledModeTimeout = params->polledModeTimeout;
		instHandle->devParams.softTxFifoThreshold = params->softTxFifoThreshold;

		instHandle->opMode = params->opMode;
		instHandle->devState = Usart_DriverState_DELETED;
		//instHandle->hwiNumber = params->hwiNumber;
		//instHandle->enableCache = params->enableCache;
		instHandle->instNum = (uint32_t)devid;
		//instHandle->txTskletHandle = NULL;
		//instHandle->rxTskletHandle = NULL;

		/* Enable USART clock and configure GPIO*/
		switch (devid)
		{
		#ifdef USE_USART1_GPIO
		 case 0:	
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
			RCC_AHB1PeriphClockCmd(USART1_RX_GPIO_CLK|USART1_TX_GPIO_CLK,ENABLE);

			/* Initialize the USART Rx and Tx pins */
			GPIO_StructInit(&gpioInit);
			gpioInit.GPIO_Pin=USART1_RX_PIN;
			gpioInit.GPIO_Mode=GPIO_Mode_AF;
			gpioInit.GPIO_Speed=GPIO_Speed_100MHz;
			gpioInit.GPIO_OType=GPIO_OType_PP;
			gpioInit.GPIO_PuPd=GPIO_PuPd_UP;
			GPIO_Init(USART1_RX_GPIO_PORT, &gpioInit);
			GPIO_PinAFConfig(USART1_RX_GPIO_PORT,USART1_RX_PIN_SOURCE,GPIO_AF_USART1);

			GPIO_StructInit(&gpioInit);
			gpioInit.GPIO_Pin=USART1_TX_PIN;
			gpioInit.GPIO_Mode=GPIO_Mode_AF;
			gpioInit.GPIO_Speed=GPIO_Speed_100MHz;
			gpioInit.GPIO_OType=GPIO_OType_PP;
			gpioInit.GPIO_PuPd=GPIO_PuPd_NOPULL;
			GPIO_Init(USART1_TX_GPIO_PORT, &gpioInit);
			GPIO_PinAFConfig(USART1_TX_GPIO_PORT,USART1_TX_PIN_SOURCE,GPIO_AF_USART1);
			break;
		#endif
		
		#ifdef USE_USART2_GPIO
		 case 1:    
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
			RCC_AHB1PeriphClockCmd(USART2_RX_GPIO_CLK|USART2_TX_GPIO_CLK,ENABLE);

			/* Initialize the USART Rx and Tx pins */
			GPIO_StructInit(&gpioInit);
			gpioInit.GPIO_Pin=USART2_RX_PIN;
			gpioInit.GPIO_Mode=GPIO_Mode_AF;
			gpioInit.GPIO_Speed=GPIO_Speed_100MHz;
			gpioInit.GPIO_OType=GPIO_OType_PP;
			gpioInit.GPIO_PuPd=GPIO_PuPd_UP;
			GPIO_Init(USART2_RX_GPIO_PORT, &gpioInit);
			GPIO_PinAFConfig(USART2_RX_GPIO_PORT,USART2_RX_PIN_SOURCE,GPIO_AF_USART2);

			GPIO_StructInit(&gpioInit);
			gpioInit.GPIO_Pin=USART2_TX_PIN;
			gpioInit.GPIO_Mode=GPIO_Mode_AF;
			gpioInit.GPIO_Speed=GPIO_Speed_100MHz;
			gpioInit.GPIO_OType=GPIO_OType_PP;
			gpioInit.GPIO_PuPd=GPIO_PuPd_NOPULL;
			GPIO_Init(USART2_TX_GPIO_PORT, &gpioInit);
			GPIO_PinAFConfig(USART2_TX_GPIO_PORT,USART2_TX_PIN_SOURCE,GPIO_AF_USART2);
			break;
		#endif
				
		#ifdef USE_USART3_GPIO
		 case 2:    
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
			RCC_AHB1PeriphClockCmd(USART3_RX_GPIO_CLK|USART3_TX_GPIO_CLK,ENABLE);

			/* Initialize the USART Rx and Tx pins */
			GPIO_StructInit(&gpioInit);
			gpioInit.GPIO_Pin=USART3_RX_PIN;
			gpioInit.GPIO_Mode=GPIO_Mode_AF;
			gpioInit.GPIO_Speed=GPIO_Speed_100MHz;
			gpioInit.GPIO_OType=GPIO_OType_PP;
			gpioInit.GPIO_PuPd=GPIO_PuPd_UP;
			GPIO_Init(USART3_RX_GPIO_PORT, &gpioInit);
			GPIO_PinAFConfig(USART3_RX_GPIO_PORT,USART3_RX_PIN_SOURCE,GPIO_AF_USART3);

			GPIO_StructInit(&gpioInit);
			gpioInit.GPIO_Pin=USART3_TX_PIN;
			gpioInit.GPIO_Mode=GPIO_Mode_AF;
			gpioInit.GPIO_Speed=GPIO_Speed_100MHz;
			gpioInit.GPIO_OType=GPIO_OType_PP;
			gpioInit.GPIO_PuPd=GPIO_PuPd_NOPULL;
			GPIO_Init(USART3_TX_GPIO_PORT, &gpioInit);
			GPIO_PinAFConfig(USART3_TX_GPIO_PORT,USART3_TX_PIN_SOURCE,GPIO_AF_USART3);
			break;
		#endif
			
		#ifdef USE_USART4_GPIO
		 case 3:    
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
			RCC_AHB1PeriphClockCmd(USART4_RX_GPIO_CLK|USART4_TX_GPIO_CLK,ENABLE);

			/* Initialize the USART Rx and Tx pins */
			GPIO_StructInit(&gpioInit);
			gpioInit.GPIO_Pin=USART4_RX_PIN;
			gpioInit.GPIO_Mode=GPIO_Mode_AF;
			gpioInit.GPIO_Speed=GPIO_Speed_100MHz;
			gpioInit.GPIO_OType=GPIO_OType_PP;
			gpioInit.GPIO_PuPd=GPIO_PuPd_UP;
			GPIO_Init(USART4_RX_GPIO_PORT, &gpioInit);
			GPIO_PinAFConfig(USART4_RX_GPIO_PORT,USART4_RX_PIN_SOURCE,GPIO_AF_UART4);

			GPIO_StructInit(&gpioInit);
			gpioInit.GPIO_Pin=USART4_TX_PIN;
			gpioInit.GPIO_Mode=GPIO_Mode_AF;
			gpioInit.GPIO_Speed=GPIO_Speed_100MHz;
			gpioInit.GPIO_OType=GPIO_OType_PP;
			gpioInit.GPIO_PuPd=GPIO_PuPd_NOPULL;
			GPIO_Init(USART4_TX_GPIO_PORT, &gpioInit);
			GPIO_PinAFConfig(USART4_TX_GPIO_PORT,USART4_TX_PIN_SOURCE,GPIO_AF_UART4);
			break;
		#endif
			
		#ifdef USE_USART5_GPIO
		 case 4:    
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);
			RCC_AHB1PeriphClockCmd(USART5_RX_GPIO_CLK|USART5_TX_GPIO_CLK,ENABLE);

			/* Initialize the USART Rx and Tx pins */
			GPIO_StructInit(&gpioInit);
			gpioInit.GPIO_Pin=USART5_RX_PIN;
			gpioInit.GPIO_Mode=GPIO_Mode_AF;
			gpioInit.GPIO_Speed=GPIO_Speed_100MHz;
			gpioInit.GPIO_OType=GPIO_OType_PP;
			gpioInit.GPIO_PuPd=GPIO_PuPd_UP;
			GPIO_Init(USART5_RX_GPIO_PORT, &gpioInit);
			GPIO_PinAFConfig(USART5_RX_GPIO_PORT,USART5_RX_PIN_SOURCE,GPIO_AF_UART5);

			GPIO_StructInit(&gpioInit);
			gpioInit.GPIO_Pin=USART5_TX_PIN;
			gpioInit.GPIO_Mode=GPIO_Mode_AF;
			gpioInit.GPIO_Speed=GPIO_Speed_100MHz;
			gpioInit.GPIO_OType=GPIO_OType_PP;
			gpioInit.GPIO_PuPd=GPIO_PuPd_NOPULL;
			GPIO_Init(USART5_TX_GPIO_PORT, &gpioInit);
			GPIO_PinAFConfig(USART5_TX_GPIO_PORT,USART5_TX_PIN_SOURCE,GPIO_AF_UART5);
			break;
		#endif
			
		#ifdef USE_USART6_GPIO
		 case 5:    
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
			RCC_AHB1PeriphClockCmd(USART6_RX_GPIO_CLK|USART6_TX_GPIO_CLK,ENABLE);

			/* Initialize the USART Rx and Tx pins */
			GPIO_StructInit(&gpioInit);
			gpioInit.GPIO_Pin=USART6_RX_PIN;
			gpioInit.GPIO_Mode=GPIO_Mode_AF;
			gpioInit.GPIO_Speed=GPIO_Speed_100MHz;
			gpioInit.GPIO_OType=GPIO_OType_PP;
			gpioInit.GPIO_PuPd=GPIO_PuPd_UP;
			GPIO_Init(GPIOC,&gpioInit);
			GPIO_PinAFConfig(GPIOC,USART6_RX_PIN_SOURCE,GPIO_AF_USART6);

			GPIO_StructInit(&gpioInit);
			gpioInit.GPIO_Pin=USART6_TX_PIN;
			gpioInit.GPIO_Mode=GPIO_Mode_AF;
			gpioInit.GPIO_Speed=GPIO_Speed_100MHz;
			gpioInit.GPIO_OType=GPIO_OType_PP;
			gpioInit.GPIO_PuPd=GPIO_PuPd_NOPULL;
			GPIO_Init(GPIOC, &gpioInit);
			GPIO_PinAFConfig(GPIOC,USART6_TX_PIN_SOURCE,GPIO_AF_USART6);
			break;
		#endif
		 default:
			break;
		}

		/* Configure Usart hw for communication parameters	 */
		usartConfigure(instHandle, &instHandle->devParams);

		if (Uart_OpMode_INTERRUPT_DMA == instHandle->opMode)
		{
		
			/* Register interrupts											  */
			installInterruptHandler(instHandle->deviceInfo.cpuEventNumber,stUsartIsr,instHandle);
			//installInterruptHandler(uint16_t vecnum,void * handler,void * arg);

			/* Configure DMA											  */

			/* configure interrupts											  */
			nvicInit.NVIC_IRQChannel = instHandle->deviceInfo.cpuEventNumber;
			nvicInit.NVIC_IRQChannelPreemptionPriority = 15;
			nvicInit.NVIC_IRQChannelSubPriority = 1;
			nvicInit.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&nvicInit);
			USART_ITConfig(instHandle->deviceInfo.baseAddress, USART_IT_IDLE, DISABLE);
			USART_ITConfig(instHandle->deviceInfo.baseAddress, USART_IT_RXNE, DISABLE);
			USART_ITConfig(instHandle->deviceInfo.baseAddress, USART_IT_TXE, DISABLE);
			USART_ITConfig(instHandle->deviceInfo.baseAddress, USART_IT_TC, DISABLE);
		}
		if (Uart_OpMode_INTERRUPT == instHandle->opMode)
		{
			/* Register interrupts											  */
			installInterruptHandler(instHandle->deviceInfo.cpuEventNumber,stUsartIsr,instHandle);

			/* configure interrupts											  */
			nvicInit.NVIC_IRQChannel = instHandle->deviceInfo.cpuEventNumber;
			nvicInit.NVIC_IRQChannelPreemptionPriority = 15;
			nvicInit.NVIC_IRQChannelSubPriority = 1;
			nvicInit.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&nvicInit);
			USART_ITConfig(instHandle->deviceInfo.baseAddress, USART_IT_IDLE, DISABLE);
			USART_ITConfig(instHandle->deviceInfo.baseAddress, USART_IT_RXNE, DISABLE);
			USART_ITConfig(instHandle->deviceInfo.baseAddress, USART_IT_TXE, DISABLE);
			USART_ITConfig(instHandle->deviceInfo.baseAddress, USART_IT_TC, DISABLE);
		}
		else if (Uart_OpMode_POLLED == instHandle->opMode)
		{
			/* polled mode of operation 									  */
			instHandle->syncSem=SEM_create(1,NULL);
		}
		else
		{
			/* do nothing													  */
		}
		
		/* Enable USART */
		USART_Cmd(instHandle->deviceInfo.baseAddress, ENABLE);

		if (IODEV_COMPLETED == retVal)
		{
			instHandle->devState = Usart_DriverState_CREATED;
			*devp = (void *)instHandle;
		}
	}
	return retVal;
}

int stUsartUnBind(void * devp)
{
	return 0;
}

int stUsartControl(void * chanp, unsigned int cmd, void *args)
{
    Usart_Object *instHandle = NULL;
    Usart_ChanObj *chanHandle = NULL;
    //uint32_t tempReg = 0;
    FlagStatus lineStatus    = RESET;
    int retVal = IODEV_COMPLETED;
    //Uart_NumStopBits stopBits = Uart_NumStopBits_1;
    //Uart_CharLen charLen = Uart_CharLen_8;

	/* Begin parameter checking 												  */
	#ifndef DRV_DISABLE_INPUT_PARAMETER_CHECK
	
	if (NULL == chanp)
	{
		retVal = IODEV_EBADARGS;
	}
	else if((cmd != Uart_IOCTL_RESET_TX_FIFO)	  &&   /* These command do not*/
	(cmd != Uart_IOCTL_RESET_RX_FIFO)	  &&   /* cmdARg. Hence this  */
	(cmd != Uart_IOCTL_CANCEL_CURRENT_IO) &&   /* check 			  */ 
	(cmd != Uart_IOCTL_CANCEL_CURRENT_IO) &&
	(cmd != Uart_IOCTL_CLEAR_STATS) 	  &&
	(cmd != Uart_IOCTL_FLUSH_ALL_REQUEST))
	{
		if(NULL == args)
		{
			retVal = IODEV_EBADARGS;			  
		}
	}
	else
	{
		/* get the handle to the channel to send the ICOTL to				  */
		chanHandle = (Usart_ChanObj*)chanp;
		instHandle = (Usart_Object *)chanHandle->devHandle;

		if ((NULL == instHandle) || (Usart_DriverState_OPENED != chanHandle->status))
		{
			retVal = IODEV_EBADARGS;	 
		}
	}
	#endif  /* PSP_DISABLE_INPUT_PARAMETER_CHECK */
	/* End parameter checking													  */

	
    if (IODEV_COMPLETED == retVal)
	{
		/* get the handle to the channel to send the IOCTL to                 */
		chanHandle = (Usart_ChanObj*)chanp;
		instHandle = (Usart_Object *)chanHandle->devHandle;

		/* Get the Line status of Uart. Uart ioctl commands to change baud    *
		 * rate, stop bit, data bit, parity, flow control and other clock     *
		 * related paramters should not be allowed when IO is on progress     *
		 * i.e. TX BUFFER is not empty                                           */
		lineStatus=USART_GetFlagStatus(instHandle->deviceInfo.baseAddress,USART_FLAG_TC);

		if (Uart_IOCTL_SET_BAUD == cmd)
		{
			/* If TX FIFO is empty, then process this command                 */
			if (SET == lineStatus)
			{
				Uart_BaudRate baudRate = Uart_BaudRate_115_2K;

				/* Perform error checking on baud rate                        */
				baudRate = *((Uart_BaudRate *)args);

				switch(baudRate)
				{
				 case Uart_BaudRate_2_4K:
				 case Uart_BaudRate_4_8K:
				 case Uart_BaudRate_9_6K:
				 case Uart_BaudRate_19_2K:
				 case Uart_BaudRate_38_4K:
				 case Uart_BaudRate_57_6K:
				 case Uart_BaudRate_115_2K:
				 case Uart_BaudRate_230_4K:
				 case Uart_BaudRate_256K:
				 case Uart_BaudRate_460_8K:
				 case Uart_BaudRate_921_6K:
				 /*Note: Tested upto this baud rate */
				 default:
					instHandle->devParams.baudRate=(Uart_BaudRate)(*(uint32_t *)args);
					usartSetBaudRate(instHandle, *(uint32_t *)args);
					break;
				}
			}
			else
			{
				/* IO operation is going. Baud rate change should not         *
				* be entertained.                                            */
				retVal = IODEV_EBADMODE;
			}
		}
	}
	
    return (retVal);
}

int stUsartCreate(void **chanp, void *devp, char *name, int mode,
                      void *chanParams, IODEV_TiomCallback cbFxn, void *cbArg)
{
	Usart_Object *instHandle  = NULL;
	Usart_ChanObj *chanHandle  = NULL;
	int bFalseWhile = 1;
	uint32_t key = 0;
	int retVal = IODEV_COMPLETED;
	#ifdef Usart_DMA_ENABLE
	//uint32_t value = 0;
	Usart_ChanParams *chanparams = NULL;
	#endif
	
	/* Begin parameter checking 												  */
	#ifndef DRV_DISABLE_INPUT_PARAMETER_CHECK
	if ((NULL == cbFxn)|| 
		(NULL == cbArg) ||	
		((IODEV_INPUT != mode) && (IODEV_OUTPUT != mode))||
		(NULL == devp) || 
		(NULL == chanp))
	{
		 retVal = IODEV_EBADARGS;
	}
	#endif  /* DRV_DISABLE_INPUT_PARAMETER_CHECK */
	/* End parameter checking													  */
			
	if (IODEV_COMPLETED == retVal)
	{
		do
		{
			bFalseWhile = 0;
	
			instHandle	= (Usart_Object *)devp;
	
			#ifdef Usart_DMA_ENABLE
			if (NULL != chanParams)
			{
				chanparams = (Usart_ChanParams *)chanParams;
			}
			#endif  /* Usart_DMA_ENABLE */           
			/* check the current mode of operation and assign suitable handle */
			if (IODEV_INPUT == mode)
				chanHandle = (Usart_ChanObj *)&instHandle->rcvChanObj;
			else
				chanHandle = (Usart_ChanObj *)&instHandle->xmtChanObj;

			/* Begin parameter checking 												  */
			#ifndef DRV_DISABLE_INPUT_PARAMETER_CHECK
			/* check if the channel was already opened.if already open then   *
			 * flag error and bail out										  */
			if ((NULL == chanHandle) ||
				(Usart_DriverState_OPENED == chanHandle->status))
			{
				retVal = IODEV_EBADARGS;
				break;
			}
			#endif  /* DRV_DISABLE_INPUT_PARAMETER_CHECK */
			/* End parameter checking													  */
	
			/* enter critical section										  */
			key = __disableInterrupts();
	
			/* Mark channel status as allocated.							  */
			chanHandle->status = Usart_DriverState_OPENED;
	
			/* exit critical section										  */
			__restoreInterrupts(key);
	
			chanHandle->mode	  = (Usart_IoMode)(mode);
			chanHandle->devHandle = instHandle;
	
			/*Assign the respective callback function						  */
			chanHandle->cbFxn = cbFxn;
			chanHandle->cbArg = (void *)cbArg;
	
			chanHandle->activeBuf	 = NULL;
			chanHandle->activeIOP	 = NULL;
			chanHandle->bytesRemaining = 0;
	
			#ifdef Usart_DMA_ENABLE
			if (NULL != chanparams)
			{
				chanHandle->hDma= chanparams->hDma;
			}
			#endif
			
			#ifdef Usart_TX_BUFFERING_ENABLE
			if (Usart_OUTPUT == chanHandle->mode)
			{
				chanHandle->iBuffer.xferActual	= 0;
				chanHandle->iBuffer.bufferIndex = 0;
				chanHandle->iBuffer.bufFillPtr	= 
					(uint8_t *)chanHandle->iBuffer.buffer;
				chanHandle->iBuffer.bufReadPtr	= 
					(uint8_t *)chanHandle->iBuffer.buffer;
			}
			#endif  /* Usart_TX_BUFFERING_ENABLE */
			chanHandle->chunkSize = (uint32_t)Uart_FIFO_SIZE;

	
			if ((Uart_OpMode_POLLED != instHandle->opMode) && (Usart_INPUT == chanHandle->mode))
			{
				 /* Enabling Line Idle Interrupt if receive channel is
				  * created, in order to process Line idle event interrupts.*/
				USART_ITConfig(instHandle->deviceInfo.baseAddress, USART_IT_IDLE, ENABLE);
			}
		}while(bFalseWhile);
	}
	
	if (IODEV_COMPLETED != retVal)
		*chanp = NULL;
	else
		*chanp = (void *)chanHandle;

	return retVal;
}

int stUsartDelete(void *chanp)
{
	return 0;
}

int stUsartSubmit(void *chanp, IODEV_Packet *packet)
{
	Usart_ChanObj *chanHandle = NULL;
	Usart_Object *instHandle = NULL;
	int retVal = IODEV_COMPLETED;
		
	/* Begin parameter checking 												  */	
	#ifndef DRV_DISABLE_INPUT_PARAMETER_CHECK
	/* The read and write command should give a proper buffer				  *
	* hence check if the buffer pointer is not null						  */
	if ((NULL == chanp) || (NULL == packet) ||
			(((IODEV_ABORT != packet->cmd) && (IODEV_FLUSH != packet->cmd))
			&& ((NULL == packet->addr) || (0 == packet->size))))
	{
		retVal = IODEV_EBADARGS;
	}
	else
	{
		chanHandle = (Usart_ChanObj *)chanp;

		if ((Usart_DriverState_OPENED != chanHandle->status) ||
										(NULL == chanHandle->devHandle))
		{
			retVal = IODEV_EBADARGS;
		}
		else
		{
			instHandle = (Usart_Object *)chanHandle->devHandle;

			if (NULL == instHandle)
				retVal = IODEV_EBADARGS;
		}
	}
	#endif  /* DRV_DISABLE_INPUT_PARAMETER_CHECK */
	/* End parameter checking													  */
	
	if (IODEV_COMPLETED == retVal)
	{
		/* get the handle to the channel to which the request if posted 	  */
		chanHandle = (Usart_ChanObj *)chanp;		  
		instHandle = (Usart_Object *)chanHandle->devHandle;

		/* check the command and process it stream will take care that		  *
		 * proper command is passed hence no need to check it once again	  */
		if ((IODEV_READ == packet->cmd) || (IODEV_WRITE == packet->cmd))
		{
			/* request will be posted for processing. Now depending on		  *
			 * The mode of operation we need to either Wait for 			  *
			 * processing to complete or the callback function to be		  *
			 * called														  */
			retVal = usartSubmitIoReq((void *)chanHandle,packet);
		}
		else if ((Uart_OpMode_POLLED != instHandle->opMode) &&
			((IODEV_ABORT == packet->cmd) || (IODEV_FLUSH == packet->cmd)))
		{
			/* we will try and abort all the packets except if the driver is  *
			 * in the POLLED mode as in polled mode we will be executing in   *
			 * task context and till it return control to application, an	  *
			 * abort call cannot come to us 								  */
			usartCancelAllIo(chanHandle);
		}
		else
		{
			/* unknown command has been passed to the driver hence set the	  *
			 * status of the command as error								  */
			retVal = IODEV_EBADARGS;
		}
	}
	return retVal;
}


#ifndef DRV_DISABLE_INPUT_PARAMETER_CHECK 
/**
 *  \brief    Validating Parameters passed to the Uart
 *
 *   This function is invoked in order to statically validate the
 *   various parameters passed to the Uart like baud Rate,Stop Bits,etc.
 *
 *  \param    instHandle   [IN]    Pointer to the Uart driver object
 *  \param    openParams   [IN]    Parameters passed to be vaildated
 *
 *  \return   IODEV_COMPLETED if success
 *
 *  \enter    instHandle must be a valid pointer and should not be null.
 *            openParams must be a valid pointer and should not be null.
 *
 *  \leave  Not Implemented.
 */
static int usartValidateParams(const Uart_Params *openParams)
{
	int  retVal = IODEV_COMPLETED;

	assert(NULL != openParams);

	/* operational mode verification                                          */
	switch (openParams->opMode)
	{
	 case Uart_OpMode_POLLED:
	 case Uart_OpMode_INTERRUPT:
	 case Uart_OpMode_INTERRUPT_DMA:
		break;
	 default:
		retVal = IODEV_EBADARGS;
		break;
	}

	/* Get the stop bits from user configuration                              */
	switch (openParams->stopBits)
	{
	 case Uart_NumStopBits_1:
	 case Uart_NumStopBits_1_5:
	 case Uart_NumStopBits_2:
		break;

	 default:
		retVal = IODEV_EBADARGS;
		break;
	}

	/* Get the char len from user configuration                               */
	switch (openParams->charLen)
	{
	 case Uart_CharLen_8:
	 case Uart_CharLen_9:
		break;

	default:
		retVal = IODEV_EBADARGS;
	break;
	}

	/* Get the parity from user configuration                                 */
	switch (openParams->parity)
	{
	 case Uart_Parity_ODD:
	 case Uart_Parity_EVEN:
	 case Uart_Parity_NONE:
		break;

	 default:
		retVal = IODEV_EBADARGS;
		break;
	}


	return retVal;
}
#endif  /* DRV_DISABLE_INPUT_PARAMETER_CHECK */



/**
 *  \brief Configure Uart device HW
 *
 *   This function is used to set up the hardware for the current settings
 *  "param" points to the hardware config and other open params.
 *
 *  \param  instHandle    [IN]   Handle to the Uart driver object
 *  \param  openParams    [IN]   Parameters passed by the user
 *
 *  \return IOM_COMPLETED in case of sucess or 
 *          E_badArgs in case of error
 *
 *  \enter  instHandle must be a valid pointer and should not be null.
 *          openParams must be a valid pointer and should not be null.
 *
 *  \leave  Not Implemented.
 */
void usartConfigure(Usart_Object *instHandle, const Usart_DevParams  *openParams)
{
	USART_InitTypeDef usartInitData;
	
	USART_StructInit(&usartInitData);
	usartInitData.USART_BaudRate=openParams->baudRate;
	usartInitData.USART_WordLength=(openParams->charLen==Uart_CharLen_9)?USART_WordLength_9b:USART_WordLength_8b;
	switch (openParams->stopBits)
	{
	 case Uart_NumStopBits_0_5:
		usartInitData.USART_StopBits=USART_StopBits_0_5;
		break;
	 case Uart_NumStopBits_1_5:
		usartInitData.USART_StopBits=USART_StopBits_1_5;
		break;
	 case Uart_NumStopBits_2:
		usartInitData.USART_StopBits=USART_StopBits_2;
		break;
	 default:
		usartInitData.USART_StopBits=USART_StopBits_1;
		break;
	}
	
	switch (openParams->parity)
	{
	 case Uart_Parity_EVEN:
		usartInitData.USART_Parity=USART_Parity_Even;
		break;
	 case Uart_Parity_ODD:
		usartInitData.USART_Parity=USART_Parity_Odd;
		break;
	 default:
		usartInitData.USART_Parity=USART_Parity_No;
		break;
	}
	usartInitData.USART_HardwareFlowControl=(openParams->fc==Uart_FcType_HW)?USART_HardwareFlowControl_RTS_CTS:USART_HardwareFlowControl_None;

	USART_Init(instHandle->deviceInfo.baseAddress,&usartInitData);

}

int usartSubmitIoReq(Usart_ChanObj *chanHandle, IODEV_Packet *ioPacket)
{
	uint32_t key = 0;
	int retVal = IODEV_COMPLETED;
	Usart_Object *instHandle = NULL;
	
	/* Validating all the input parameters inputs							  */
	assert((NULL != chanHandle) && (NULL != ioPacket));
		
	/* get the handle the instance											  */
	instHandle = (Usart_Object *)chanHandle->devHandle;
	assert(NULL != instHandle);
		
	/* If polled mode is set with the active iop as the received iop call	  *
	 * the function "uartRxIntrHandler" where the control will block		  *
	 * till it gets the requested number of bytes.							  */
	
	if (Uart_OpMode_POLLED == instHandle->opMode)
	{
		/* we will protect this function with a semaphore because in case of  *
		 * multiple tasks submitting the IO requests we will be running in to *
		 * race conditions hence in polled mode we will only allow only one   *
		 * task to complete the request in polled mode. and the other task	  *
		 * will be pending on the semaphore.(until the first task has		  *
		 * completed the IO).												  */
		if (pdPASS == SEM_pend(instHandle->syncSem, instHandle->polledModeTimeout))
		{
			retVal = usartSubmitIoReqPolled(chanHandle,ioPacket);
			/* we have completed the IO processing hence post the semaphore   */
			SEM_post(instHandle->syncSem);
		}
		else
			retVal = IODEV_ETIMEOUT;
	}
	else
	{
		/* interrupt or dma mode of operation										  */

		/* disable the interrupts											  */
		key = __disableInterrupts();

		#ifdef Usart_TX_BUFFERING_ENABLE
		retVal = usartSubmitIoReqBuffering(chanHandle,ioPacket);
		#else
		retVal = usartSubmitIoReqIntDma(chanHandle,ioPacket,key);
		#endif
		__restoreInterrupts(key);
	}
	return retVal;
}

void usartCancelCurrentIo(Usart_ChanObj *chanHandle, IODEV_Packet *ioPacket)
{
}
//static void usartEnableHwFlowControl(Usart_Object *instHandle, const Usart_FlowControl  *pFlowCtrl);
//static uint32_t sciXfer(Usart_ChanObj *chanHandle, uint8_t *buffer, uint32_t numBytes);

int usartSubmitIoReqPolled(Usart_ChanObj *chanHandle, IODEV_Packet *ioPacket)
{
	int retVal = IODEV_COMPLETED;
	uint16_t lineStatus	  = 0;
	portTickType currentTick	  = 0;
	int timeoutElapsed = 0;
	Usart_Object *instHandle = NULL;
	
	assert((NULL != chanHandle) && (NULL != ioPacket));
	
	/* get the handle the instance											  */
	instHandle = (Usart_Object *)chanHandle->devHandle;
	assert(NULL != instHandle);
		
	/* This submit does not allow a second request (from different			  *
	 * task ) when we are in polled mode									  */
	if (NULL != chanHandle->activeIOP)
	{
		/* already an active iop is present hence cannot queue another		  *
		 * request in polled mode											  */
		ioPacket->status = IODEV_EBADIO;
		ioPacket->size	= 0;
		retVal			= IODEV_EBADIO;
	}
	else
	{
		/* No active iop hence can queue our request						  */
		chanHandle->errors		   = 0 ;
		chanHandle->activeIOP	   = ioPacket;
		chanHandle->activeBuf	   = ioPacket->addr;
		chanHandle->bytesRemaining = ioPacket->size;
		retVal = IODEV_EBADIO;

		/* Update current tick value to perform timeout operation			  */
		currentTick = xTaskGetTickCount();
		while ((0 == timeoutElapsed) && (0 != chanHandle->bytesRemaining))
		{
			usartPoll(chanHandle);
			/* Check whether timeout happened or not						  */
			timeoutElapsed = usartOsCheckTimeOut(currentTick, instHandle->polledModeTimeout);

			vTaskDelay((Usart_TPOLL_MSECS));
		}

		if ((0 == chanHandle->bytesRemaining) && (0 == chanHandle->errors))
		{
			do
			{
				lineStatus = (instHandle->deviceInfo.baseAddress)->SR;
			}while((USART_FLAG_TC|USART_FLAG_TXE) != (lineStatus & (USART_FLAG_TC|USART_FLAG_TXE)));			  
			
			retVal = IODEV_COMPLETED;
		} 
	}

	if (0 != timeoutElapsed)
	{
		ioPacket->status = retVal;
		ioPacket->size = ioPacket->size - chanHandle->bytesRemaining;

		/* Perform the operation to complete the IO 						  */
		Usart_localCompleteCurrentIO(chanHandle);
	}

	return retVal;
}

#ifdef Usart_TX_BUFFERING_ENABLE
int usartSubmitIoReqBuffering(Usart_ChanObj *chanHandle, IODEV_Packet *ioPacket)
{
	return 0;
}
#endif                                      

int usartSubmitIoReqIntDma(Usart_ChanObj *chanHandle, IODEV_Packet *ioPacket, uint32_t key)
{
	int retVal = IODEV_COMPLETED;
	Usart_Object *instHandle = NULL;

	assert((NULL != chanHandle) && (NULL != ioPacket));

	/* get the handle the instance											  */
	instHandle = (Usart_Object *)chanHandle->devHandle;
	assert(NULL != instHandle);

	/* To remove the compiler warning										  */
	key = key;

	/* If not in polled mode,then it may be in INT or DMA mode. 			  *
	* Disable the interrupts.Check if any IOP is active, if yes			  *
	* check if the new IOP has a timeout value. If not then post			  *
	* an error. else enque the iop into the list and return back.								  */
	if (NULL != chanHandle->activeIOP)
	{
		/* Queue it and update the return value 							  */
		QUE_put(&(chanHandle->queuePendingList),(void *)ioPacket);
		ioPacket->status = IODEV_PENDING;
		retVal = IODEV_PENDING;
	}
	else
	{
		/* No other IOP is active, We can process this IOP, set 			  *
		* it as active IOP 												  */
		chanHandle->activeIOP	   = ioPacket;
		chanHandle->activeBuf	   = ioPacket->addr;
		chanHandle->bytesRemaining = ioPacket->size;
		chanHandle->errors		   = 0 ;

		#ifdef Usart_DMA_ENABLE
		if (Uart_OpMode_INTERRUPT_DMA == instHandle->opMode)
		{
			/* If DMA mode then start the DMA								  */
			retVal = usart_localStartDmaTransfer(chanHandle, (uint32_t)ioPacket->cmd);

			if	(IODEV_COMPLETED == retVal)
				retVal = IODEV_PENDING;
		}
		#endif  /* Usart_DMA_ENABLE */
		retVal	= IODEV_PENDING;

		/* enable interrupt 												  */
		if (Uart_OpMode_INTERRUPT_DMA != instHandle->opMode)
		{
			if (Usart_INPUT == chanHandle->mode)
			{
				//USART_ITConfig(instHandle->deviceInfo.baseAddress, USART_IT_IDLE, ENABLE);
				USART_ITConfig(instHandle->deviceInfo.baseAddress, USART_IT_RXNE, ENABLE);
			}
			else
				USART_ITConfig(instHandle->deviceInfo.baseAddress, USART_IT_TXE, ENABLE);
		}
		else
		{
			USART_ITConfig(instHandle->deviceInfo.baseAddress, USART_IT_RXNE, DISABLE);
			if (Usart_INPUT == chanHandle->mode)
			{
				USART_ITConfig(instHandle->deviceInfo.baseAddress, USART_IT_IDLE, ENABLE);
			}
			#ifdef KUKU
			else
				USART_ITConfig(instHandle->deviceInfo.baseAddress, USART_IT_TC, ENABLE);
			#endif
		}
	}/*end of buffer mode check 											  */

	return retVal;
}


void usartCancelAllIo(Usart_ChanObj *chanHandle)
{
}

int usartOsCheckTimeOut(portTickType startValue, portTickType timeout)
{
    portTickType checkValue = 0;
    int retVal = 1;

    /* get the current tick value and compare with the start value            */
    checkValue = xTaskGetTickCount();

    if (checkValue < startValue)
        checkValue = (((portMAX_DELAY) - startValue) + checkValue) + ((portTickType)1) ;
    else
        checkValue = checkValue - startValue;

    /* if the difference between the current tick and start tick is greater   *
     * than start tick then set retval to TRUE to indicate time out           */
    if (checkValue < timeout)
        retVal  =   0;
    else
        retVal  =   1;
    return  retVal;
}

static void usartPoll(Usart_ChanObj *chanHandle)
{
	Usart_Object *instHandle = NULL;
    IODEV_Packet      *ioPacket   = NULL;
	uint16_t	txd;
	uint16_t	rxd;
	
	assert(NULL != chanHandle);
	
	/* get the handle the instance											  */
	instHandle = (Usart_Object *)chanHandle->devHandle;
	assert(NULL != instHandle);
	
    ioPacket = chanHandle->activeIOP;

    if (NULL  != ioPacket)
	{
		if (chanHandle->mode==Usart_INPUT)
		{
			if (USART_GetFlagStatus(instHandle->deviceInfo.baseAddress, USART_FLAG_RXNE))
			{
				if (chanHandle->bytesRemaining)
				{
					rxd=USART_ReceiveData(instHandle->deviceInfo.baseAddress);
					*chanHandle->activeBuf=rxd;
					chanHandle->bytesRemaining -= 1;
					chanHandle->activeBuf += 1;
				}
			}
		}
		else
		{
			if (USART_GetFlagStatus(instHandle->deviceInfo.baseAddress,USART_FLAG_TXE))
			{
				if (chanHandle->bytesRemaining)
				{
					txd=*chanHandle->activeBuf;
					USART_SendData(instHandle->deviceInfo.baseAddress,txd);
					chanHandle->bytesRemaining -= 1;
					chanHandle->activeBuf += 1;
				}
			}
		}
	}
}

int Usart_localCompleteCurrentIO (Usart_ChanObj *chanHandle)
{
	Usart_Object *instHandle = NULL;
	int retVal = IODEV_COMPLETED;
	uint32_t key = 0;
	int completeIo=0;
	#ifdef Usart_DMA_ENABLE
	uint32_t bytesRemain = 0;
	#endif
	key = __disableInterrupts();
	assert(NULL != chanHandle);
	instHandle = (Usart_Object*)chanHandle->devHandle;
	assert(NULL != instHandle);

	/* call the application completion callback function registered			*
	* with us during opening of the channel								  */
	if ((NULL != chanHandle->cbFxn) && (NULL != chanHandle->cbArg) && (Uart_OpMode_POLLED != instHandle->opMode))
	{
		/* Update the packet status 										  */
		if (0 != chanHandle->errors)
		{
			/* Priority is for the appropriate status set before calling this *
			 * completion function for the IOP								  */
			if(IODEV_COMPLETED == chanHandle->activeIOP->status)
				chanHandle->activeIOP->status = IODEV_EBADIO;
		}
		else
			chanHandle->activeIOP->status = IODEV_COMPLETED;

		/* Update the size													  */
		if (Uart_OpMode_INTERRUPT == instHandle->opMode)
			chanHandle->activeIOP->size -= chanHandle->bytesRemaining;
		#ifdef Usart_DMA_ENABLE
		else if (Uart_OpMode_INTERRUPT_DMA == instHandle->opMode)
		{
			/* Get the PaRAM set for default parameters 					  */
			if (chanHandle->hDma)
				bytesRemain=DMA_GetCurrDataCounter(dmaStreamInfo.dmac[((struct sDmaParam *)(chanHandle->hDma))->dmacId].pStream[((struct sDmaParam *)(chanHandle->hDma))->stream]);
			else
				bytesRemain=0;
			/* calculate the amount of bytes remaining						  */
			chanHandle->activeIOP->size -= bytesRemain;
		}
		#endif
		else
		{
			/* Do nothing - poll mode is handled in transfer function itself  */
		}

		/* switch OFF the module in the PSC 								  */
		//retVal = Uart_localLpscOff(instHandle);

		/* when setting the status of the IOP,priority is given to the error  *
		 * generated by the IOP. if the IOP has completed successfully, then  *
		 * the priority will be for the error (if any) generated by the PSC   *
		 * while switching off the module									  */
		if (IODEV_COMPLETED == chanHandle->activeIOP->status)
			chanHandle->activeIOP->status = retVal;

		/* Invoke Application callback for this channel 					  */
		completeIo=(*chanHandle->cbFxn)((void *)chanHandle->cbArg,chanHandle->activeIOP);
		//completeIo=1;
	}

	chanHandle->activeIOP = NULL;

	__restoreInterrupts(key);
	return completeIo;
}

/**
 * \brief     This function restores the driver to the original state that is it
 *            resumes the normal operation of the driver by picking the IOPs
 *            from the pending queue and putting it to the active IOP.
 *
 * \param     instHandle [IN] Handle to the device instance object
 *
 * \enter     instHandle is a valid non NULL pointer
 *
 * \leave     Not impplemented
 */
void Usart_loadPendedIops(Usart_ChanObj  *chanHandle)
{
	Usart_Object *instHandle = NULL;
	IODEV_Packet *ioPacket = NULL;

	assert(NULL != chanHandle);
	assert(NULL != chanHandle->devHandle);

	instHandle = (Usart_Object *)chanHandle->devHandle;

	if (0 == QUE_empty(&(chanHandle->queuePendingList)))
	{
		/* we have atleast one packet                                         */
		ioPacket = (IODEV_Packet *)QUE_get(&(chanHandle->queuePendingList));

		/* validate and update the iop                                        */
		if (NULL  != ioPacket)
		{
			chanHandle->activeIOP      = ioPacket;
			chanHandle->activeBuf      = ioPacket->addr;
			chanHandle->bytesRemaining = ioPacket->size;
			chanHandle->errors = 0;

			if (Uart_OpMode_INTERRUPT == instHandle->opMode)
			{
				if (Usart_INPUT == chanHandle->mode)
				{
					USART_ITConfig(instHandle->deviceInfo.baseAddress, USART_IT_RXNE, ENABLE);
				}
				else
					USART_ITConfig(instHandle->deviceInfo.baseAddress, USART_IT_TXE, ENABLE);
			}

			#ifdef Usart_DMA_ENABLE
			if (Uart_OpMode_INTERRUPT_DMA == instHandle->opMode)
			{
				usart_localStartDmaTransfer(chanHandle,ioPacket->cmd);
				if (Usart_INPUT == chanHandle->mode)
				{
					USART_ITConfig(instHandle->deviceInfo.baseAddress, USART_IT_IDLE, ENABLE);
				}
			}
			#endif
		}
	}
	else
	{
		chanHandle->activeIOP = NULL;
	}
}

int stUsartIsr(void *arg)
{
	#ifdef USE_USART_DRIVER_TIMESTAMPS
	TIMESTAMP ts=readTimestamp();
	TIMESTAMP *tsp;
	#endif
	Usart_Object *instHandle  = NULL;
	Usart_ChanObj *chanHandle  = NULL;
    IODEV_Packet *ioPacket   = NULL;
	uint16_t idleDetected=0;
	int completeIo=0;
	
    assert(NULL != arg);

    instHandle = (Usart_Object*)arg;


	if(Uart_OpMode_INTERRUPT == instHandle->opMode)
	{
	
		if (USART_GetITStatus(instHandle->deviceInfo.baseAddress, USART_IT_IDLE) != RESET)
			idleDetected=1;
		if (USART_GetITStatus(instHandle->deviceInfo.baseAddress, USART_IT_RXNE) != RESET)
		{
			chanHandle = &(instHandle->rcvChanObj);

			ioPacket = chanHandle->activeIOP;
			if (ioPacket==NULL)
			{
				/* Disable the RX interrupts before completing. If only   *
				 * there is a packet for processing it shall be re-enabled*/
				USART_ITConfig(instHandle->deviceInfo.baseAddress, USART_IT_RXNE, DISABLE);
			}
			else
			{
				*(chanHandle->activeBuf) = (uint8_t)USART_ReceiveData(instHandle->deviceInfo.baseAddress);
			
				#ifdef USE_USART_DRIVER_TIMESTAMPS
				tsp=(TIMESTAMP *)ioPacket->addr;
				if (chanHandle->bytesRemaining<ioPacket->size)
				{
					tsp[-1]=ts; /* mark end of frame timestamp */
				}
				else
				{
					tsp[-2]=tsp[-1]=ts; /* mark start and end of frame timestamps */
				}
				#endif	

				chanHandle->bytesRemaining	  -= 1;
				chanHandle->activeBuf		  += 1;
				instHandle->stats.rxBytes	  += 1;
				USART_ITConfig(instHandle->deviceInfo.baseAddress, USART_IT_IDLE, ENABLE);

				if ((chanHandle->bytesRemaining==0)||idleDetected)
				{
					/* Disable the RX interrupts before completing. If only   *
					 * there is a packet for processing it shall be re-enabled*/
					USART_ITConfig(instHandle->deviceInfo.baseAddress, USART_IT_RXNE, DISABLE);
					
					/* Perform the operation to complete the IO 			  */
					completeIo|=Usart_localCompleteCurrentIO(chanHandle);
					Usart_loadPendedIops(chanHandle);
				}
			}
		}
		else if (idleDetected)
		{
			chanHandle = &(instHandle->rcvChanObj);
			ioPacket = chanHandle->activeIOP;
			if (ioPacket!=NULL)
			{
				if  (chanHandle->bytesRemaining<chanHandle->activeIOP->size)
				{
					/* Disable the RX interrupts before completing. If only   *
					 * there is a packet for processing it shall be re-enabled*/
					USART_ITConfig(instHandle->deviceInfo.baseAddress, USART_IT_RXNE, DISABLE);
					USART_ITConfig(instHandle->deviceInfo.baseAddress, USART_IT_IDLE, DISABLE);
					
					/* Perform the operation to complete the IO 			  */
					completeIo|=Usart_localCompleteCurrentIO(chanHandle);
					Usart_loadPendedIops(chanHandle);
				}
			}
		}
		
		if(USART_GetITStatus(instHandle->deviceInfo.baseAddress, USART_IT_TXE) != RESET)
		{
			chanHandle = &(instHandle->xmtChanObj);
			ioPacket = chanHandle->activeIOP;
			if (ioPacket==NULL)
			{
				USART_ITConfig(instHandle->deviceInfo.baseAddress, USART_IT_TXE, DISABLE);
			}
			else
			{
				/* Write one byte to the transmit data register */
				USART_SendData(instHandle->deviceInfo.baseAddress, *(chanHandle->activeBuf));
				chanHandle->bytesRemaining	  -= 1;
				chanHandle->activeBuf		  += 1;
				instHandle->stats.txBytes	  += 1;

				if (chanHandle->bytesRemaining==0)
				{
					/* Disable the TX interrupts before completing. If only   *
					 * there is a packet for processing it shall be re-enabled*/
					USART_ITConfig(instHandle->deviceInfo.baseAddress, USART_IT_TXE, DISABLE);
					
					/* Perform the operation to complete the IO 			  */
					completeIo|=Usart_localCompleteCurrentIO(chanHandle);
					Usart_loadPendedIops(chanHandle);
				}
			}
		}
	}
	#ifdef Usart_DMA_ENABLE
	else if(Uart_OpMode_INTERRUPT_DMA== instHandle->opMode)
	{
		if (USART_GetITStatus(instHandle->deviceInfo.baseAddress, USART_IT_IDLE) != RESET)
		{
			USART_ITConfig(instHandle->deviceInfo.baseAddress, USART_IT_IDLE, DISABLE);
			
			/* If there is a packet reception in progress, terminate DMA */
			chanHandle = &(instHandle->rcvChanObj);
			ioPacket = chanHandle->activeIOP;
			if (ioPacket!=NULL)
			{
				/* Terminate DMA */
				#ifdef USE_USART_DRIVER_TIMESTAMPS
				usart_localStopDmaTransfer(chanHandle,&ts);
				#else
				usart_localStopDmaTransfer(chanHandle,NULL);
				#endif
				if (USART_GetITStatus(instHandle->deviceInfo.baseAddress, USART_IT_RXNE) == RESET)
					USART_ReceiveData(instHandle->deviceInfo.baseAddress);
					
			}
			
		}
		#ifdef KUKU
		if(USART_GetITStatus(instHandle->deviceInfo.baseAddress, USART_IT_TC) != RESET)
		{
			USART_ClearITPendingBit(instHandle->deviceInfo.baseAddress,USART_IT_TC);
		}
		#endif
	}
	#endif
	return completeIo;
}

void usartSetBaudRate(Usart_Object *instHandle, uint32_t USART_BaudRate)
{
	uint32_t tmpreg = 0x00, apbclock = 0x00;
	uint32_t integerdivider = 0x00;
	uint32_t fractionaldivider = 0x00;
	RCC_ClocksTypeDef RCC_ClocksStatus;

	/*---------------------------- USART BRR Configuration -----------------------*/
	/* Configure the USART Baud Rate */
	RCC_GetClocksFreq(&RCC_ClocksStatus);

	if ((instHandle->deviceInfo.baseAddress == USART1) || (instHandle->deviceInfo.baseAddress == USART6))
	{
		apbclock = RCC_ClocksStatus.PCLK2_Frequency;
	}
	else
	{
		apbclock = RCC_ClocksStatus.PCLK1_Frequency;
	}

	/* Determine the integer part */
	if ((instHandle->deviceInfo.baseAddress->CR1 & USART_CR1_OVER8) != 0)
	{
		/* Integer part computing in case Oversampling mode is 8 Samples */
		integerdivider = ((25 * apbclock) / (2 * (USART_BaudRate)));    
	}
	else /* if ((USARTx->CR1 & USART_CR1_OVER8) == 0) */
	{
		/* Integer part computing in case Oversampling mode is 16 Samples */
		integerdivider = ((25 * apbclock) / (4 * (USART_BaudRate)));    
	}
	tmpreg = (integerdivider / 100) << 4;

	/* Determine the fractional part */
	fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

	/* Implement the fractional part in the register */
	if ((instHandle->deviceInfo.baseAddress->CR1 & USART_CR1_OVER8) != 0)
	{
		tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t)0x07);
	}
	else /* if ((USARTx->CR1 & USART_CR1_OVER8) == 0) */
	{
		tmpreg |= ((((fractionaldivider * 16) + 50) / 100)) & ((uint8_t)0x0F);
	}

	/* Write to USART BRR register */
	instHandle->deviceInfo.baseAddress->BRR = (uint16_t)tmpreg;

	

}


#ifdef Usart_DMA_ENABLE

int usart_localStartDmaTransfer(Usart_ChanObj *chanHandle, uint32_t cmd)
{
	int	   retVal		 = IODEV_COMPLETED;

	assert(NULL != chanHandle);

	chanHandle->ioCompleted = 0;

	if (IODEV_READ == cmd)
		retVal = usartStartDmaRxTransfer(chanHandle);
	else if (IODEV_WRITE == cmd)
		retVal = usartStartDmaTxTransfer(chanHandle);
	else
		retVal = IODEV_ENOTIMPL;

	return retVal;
}


int usartStartDmaTxTransfer(Usart_ChanObj *chanHandle)
{
	int retVal = IODEV_COMPLETED;
	DMA_Stream_TypeDef	*hDmaStream = NULL;
	Usart_Object *instHandle = NULL;
	DMA_InitTypeDef dmaInit;
	//struct sDmaResSet *hDmaResSet=NULL;
	DMA_RESOURCE *hDmaRes=NULL;
	struct sDmaParam *hDmaParam=NULL;
	uint32_t key;
	NVIC_InitTypeDef nvicInit;

	assert(NULL != chanHandle);

	
	instHandle = (Usart_Object*)chanHandle->devHandle;

	assert(NULL != instHandle);

	#ifdef KUKU
	if (instHandle->enableCache)
	{
		/* Move the data to physical memory and invalidate the cache line	  */
	}
	#endif
	hDmaParam=(struct sDmaParam *)chanHandle->hDma;
	if (instHandle->hDmaRes==NULL)
	{
		/* Obtain DMA resource */
		hDmaRes=allocDmaResource(hDmaParam->dmacId,hDmaParam->stream,hDmaParam->channel,portMAX_DELAY,NULL);
		key=__disableInterrupts();
		instHandle->hDmaRes=hDmaRes;
		instHandle->xmtChanObj.hDmaHandler=NULL;
		instHandle->xmtChanObj.hDmaHandlerArg=NULL;
		instHandle->rcvChanObj.hDmaHandler=NULL;
		instHandle->rcvChanObj.hDmaHandlerArg=NULL;
		__restoreInterrupts(key);
	}
	else 
		hDmaRes=(DMA_RESOURCE *)instHandle->hDmaRes;

	if (hDmaRes==NULL)
		return IODEV_EBADIO;

	/* install interrupt handler */
	if ((chanHandle->hDmaHandler!=(void *)stUsartDmaRxIsr)||(chanHandle->hDmaHandlerArg!=chanHandle))
	{
		nvicInit.NVIC_IRQChannel = dmaStreamInfo.dmac[hDmaParam->dmacId].nIrq[hDmaParam->stream];
		nvicInit.NVIC_IRQChannelPreemptionPriority = 15;
		nvicInit.NVIC_IRQChannelSubPriority = 1;
		nvicInit.NVIC_IRQChannelCmd = ENABLE;
		installInterruptHandler((uint16_t)nvicInit.NVIC_IRQChannel,stUsartDmaTxIsr,chanHandle);
		NVIC_Init(&nvicInit);
		
	}

	hDmaStream=dmaStreamInfo.dmac[hDmaParam->dmacId].pStream[hDmaParam->stream];

	DMA_StructInit(&dmaInit);
	dmaInit.DMA_Channel =  dmaChanTable[hDmaParam->channel];
	dmaInit.DMA_PeripheralBaseAddr = (uint32_t)&instHandle->deviceInfo.baseAddress->DR;
	dmaInit.DMA_Memory0BaseAddr = (uint32_t)chanHandle->activeBuf;
	dmaInit.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	dmaInit.DMA_MemoryInc=DMA_MemoryInc_Enable;
	dmaInit.DMA_Priority=DMA_Priority_Medium;
	if (chanHandle->bytesRemaining <= 0xFFFF)
		dmaInit.DMA_BufferSize = chanHandle->bytesRemaining;
	else
	{
		retVal = IODEV_EBADIO;
	}
	if (IODEV_COMPLETED == retVal)
	{
		DMA_Init(hDmaStream,&dmaInit);
		DMA_Cmd(hDmaStream,ENABLE);
		DMA_ITConfig(hDmaStream,DMA_IT_TC,ENABLE);
		USART_DMACmd(instHandle->deviceInfo.baseAddress,USART_DMAReq_Tx,ENABLE);
	}

	return (retVal);
}


int usartStartDmaRxTransfer(Usart_ChanObj *chanHandle)
{
	int retVal = IODEV_COMPLETED;
	DMA_Stream_TypeDef	*hDmaStream = NULL;
	Usart_Object *instHandle = NULL;
	DMA_InitTypeDef dmaInit;
	//struct sDmaResSet *hDmaResSet=NULL;
	DMA_RESOURCE *hDmaRes=NULL;
	struct sDmaParam *hDmaParam=NULL;
	uint32_t key;
	NVIC_InitTypeDef nvicInit;
	
	assert(NULL != chanHandle);

	
	instHandle = (Usart_Object*)chanHandle->devHandle;

	assert(NULL != instHandle);

	#ifdef KUKU
	if (instHandle->enableCache)
	{
		/* Move the data to physical memory and invalidate the cache line	  */
	}
	#endif
	hDmaParam=(struct sDmaParam *)chanHandle->hDma;
	if (instHandle->hDmaRes==NULL)
	{
		/* Obtain DMA resource */
		hDmaRes=allocDmaResource(hDmaParam->dmacId,hDmaParam->stream,hDmaParam->channel,portMAX_DELAY,NULL);
		key=__disableInterrupts();
		instHandle->hDmaRes=hDmaRes;
		instHandle->xmtChanObj.hDmaHandler=NULL;
		instHandle->xmtChanObj.hDmaHandlerArg=NULL;
		instHandle->rcvChanObj.hDmaHandler=NULL;
		instHandle->rcvChanObj.hDmaHandlerArg=NULL;
		__restoreInterrupts(key);
	}
	else 
		hDmaRes=(DMA_RESOURCE *)instHandle->hDmaRes;
	if (hDmaRes==NULL)
		return IODEV_EBADIO;

	/* install interrupt handler */
	if ((chanHandle->hDmaHandler!=(void *)stUsartDmaRxIsr)||(chanHandle->hDmaHandlerArg!=chanHandle))
	{
		nvicInit.NVIC_IRQChannel = dmaStreamInfo.dmac[hDmaParam->dmacId].nIrq[hDmaParam->stream];
		nvicInit.NVIC_IRQChannelPreemptionPriority = 15;
		nvicInit.NVIC_IRQChannelSubPriority = 1;
		nvicInit.NVIC_IRQChannelCmd = ENABLE;
		installInterruptHandler((uint16_t)nvicInit.NVIC_IRQChannel,stUsartDmaRxIsr,chanHandle);
		chanHandle->hDmaHandler=stUsartDmaRxIsr;
		chanHandle->hDmaHandlerArg=chanHandle;
		NVIC_Init(&nvicInit);
		
	}
	hDmaStream=dmaStreamInfo.dmac[hDmaParam->dmacId].pStream[hDmaParam->stream];

	DMA_StructInit(&dmaInit);
	dmaInit.DMA_Channel = dmaChanTable[hDmaParam->channel];
	dmaInit.DMA_PeripheralBaseAddr = (uint32_t)&instHandle->deviceInfo.baseAddress->DR;
	dmaInit.DMA_Memory0BaseAddr = (uint32_t)chanHandle->activeBuf;
	dmaInit.DMA_DIR = DMA_DIR_PeripheralToMemory;
	dmaInit.DMA_MemoryInc=DMA_MemoryInc_Enable;
	dmaInit.DMA_Priority=DMA_Priority_Medium;
	if (chanHandle->bytesRemaining <= 0xFFFF)
		dmaInit.DMA_BufferSize = chanHandle->bytesRemaining;
	else
	{
		retVal = IODEV_EBADIO;
	}
	if (IODEV_COMPLETED == retVal)
	{
		DMA_Init(hDmaStream,&dmaInit);
		DMA_Cmd(hDmaStream,ENABLE);
		DMA_ITConfig(hDmaStream,DMA_IT_TC,ENABLE);
		USART_DMACmd(instHandle->deviceInfo.baseAddress,USART_DMAReq_Rx,ENABLE);
	}

	return (retVal);
}

int usart_localStopDmaTransfer(Usart_ChanObj *chanHandle, void *arg)
{
	#ifdef USE_USART_DRIVER_TIMESTAMPS
	Usart_Object *instHandle  = NULL;
	IODEV_Packet *ioPacket;
	uint16_t bytesRemain=0;
	TIMESTAMP *tsp;
	#endif
	
	DMA_Cmd(dmaStreamInfo.dmac[((struct sDmaParam *)(chanHandle->hDma))->dmacId].pStream[((struct sDmaParam *)(chanHandle->hDma))->stream],DISABLE);
	#ifdef USE_USART_DRIVER_TIMESTAMPS
	if (arg!=NULL)
	{
		instHandle = (Usart_Object *)chanHandle->devHandle;
		ioPacket=chanHandle->activeIOP;
		if ((bytesRemain=DMA_GetCurrDataCounter(dmaStreamInfo.dmac[((struct sDmaParam *)(chanHandle->hDma))->dmacId].pStream[((struct sDmaParam *)(chanHandle->hDma))->stream]))!=0)
		{
			tsp=(TIMESTAMP *)ioPacket->addr;
			tsp[-1]= *((TIMESTAMP *)arg); /* mark start and end of frame timestamps */
			tsp[-2]= tsp[-1];
			offsetTimestamp(&tsp[-1], 0-calcUsartBlockWithIdleTime(instHandle, 0));
			offsetTimestamp(&tsp[-2], 0-calcUsartBlockWithIdleTime(instHandle, ioPacket->size-bytesRemain));
		}
	}
	#endif	
	return 0;
}

int stUsartDmaTxIsr(void *arg)
{
	Usart_Object *instHandle  = NULL;
	Usart_ChanObj *chanHandle  = NULL;
    IODEV_Packet *ioPacket   = NULL;
	uint16_t bytesRemain=0;
	DMA_Stream_TypeDef *pstream=NULL;
	int completeIo=0;
	
    assert(NULL != arg);

    chanHandle = (Usart_ChanObj *)arg;
	instHandle = (Usart_Object *)chanHandle->devHandle;
	pstream=dmaStreamInfo.dmac[((struct sDmaParam *)(chanHandle->hDma))->dmacId].pStream[((struct sDmaParam *)(chanHandle->hDma))->stream];
	
	if (DMA_GetFlagStatus(pstream, dmaTCflagid[((struct sDmaParam *)(chanHandle->hDma))->stream])!=RESET)
	{
		DMA_ClearITPendingBit(pstream,dmaTCflagid[((struct sDmaParam *)(chanHandle->hDma))->stream]);
		DMA_ITConfig(pstream,DMA_IT_TC,DISABLE);

		/* Disable the TX DMA. If only   *
		 * there is a packet for processing it shall be re-enabled*/
		USART_DMACmd(instHandle->deviceInfo.baseAddress,USART_DMAReq_Tx,DISABLE);
		ioPacket = chanHandle->activeIOP;
		if (ioPacket!=NULL)
		{
			bytesRemain=DMA_GetCurrDataCounter(pstream);
			chanHandle->bytesRemaining	  = bytesRemain;
			chanHandle->activeBuf		  += ioPacket->size-bytesRemain;
			instHandle->stats.txBytes	  += ioPacket->size-bytesRemain;
			completeIo|=Usart_localCompleteCurrentIO(chanHandle);
			Usart_loadPendedIops(chanHandle);
		}
		else
			Usart_loadPendedIops(chanHandle);


		
	}
	
	return completeIo;
}

int stUsartDmaRxIsr(void *arg)
{
	#ifdef USE_USART_DRIVER_TIMESTAMPS
	TIMESTAMP ts=readTimestamp();
	TIMESTAMP *tsp;
	#endif
	Usart_Object *instHandle  = NULL;
	Usart_ChanObj *chanHandle  = NULL;
    IODEV_Packet *ioPacket   = NULL;
	//uint16_t idleDetected=0;
	uint16_t bytesRemain=0;
	DMA_Stream_TypeDef *pstream=NULL;
	int completeIo=0;
	
    assert(NULL != arg);

    chanHandle = (Usart_ChanObj *)arg;
	instHandle = (Usart_Object *)chanHandle->devHandle;
	pstream=dmaStreamInfo.dmac[((struct sDmaParam *)(chanHandle->hDma))->dmacId].pStream[((struct sDmaParam *)(chanHandle->hDma))->stream];

	if (DMA_GetFlagStatus(pstream, dmaTCflagid[((struct sDmaParam *)(chanHandle->hDma))->stream])!=RESET)
	{
		DMA_ClearITPendingBit(pstream,dmaTCflagid[((struct sDmaParam *)(chanHandle->hDma))->stream]);
		DMA_ITConfig(pstream,DMA_IT_TC,DISABLE);
		
		/* Disable the RX DMA before completing. If only   *
		 * there is a packet for processing it shall be re-enabled*/
		USART_DMACmd(instHandle->deviceInfo.baseAddress,USART_DMAReq_Rx,DISABLE);
		
		ioPacket = chanHandle->activeIOP;
		if (ioPacket!=NULL)
		{
		
			if ((bytesRemain=DMA_GetCurrDataCounter(pstream))==0)
		 	{
				#ifdef USE_USART_DRIVER_TIMESTAMPS
				tsp=(TIMESTAMP *)ioPacket->addr;
				tsp[-1]=ts; /* mark end of frame timestamp */
				offsetTimestamp(&ts, 0-calcUsartBlockTime(instHandle, ioPacket->size));
				tsp[-2]=ts; /* mark start and end of frame timestamps */
				#endif	
		 	}
			chanHandle->bytesRemaining	  = bytesRemain;
			chanHandle->activeBuf		  += ioPacket->size-bytesRemain;
			instHandle->stats.rxBytes	  += ioPacket->size-bytesRemain;
			USART_ITConfig(instHandle->deviceInfo.baseAddress, USART_IT_IDLE, ENABLE);
		
			/* Perform the operation to complete the IO 			  */
			completeIo|=Usart_localCompleteCurrentIO(chanHandle);
			Usart_loadPendedIops(chanHandle);
		}
		else
			Usart_loadPendedIops(chanHandle);
	}
	return completeIo;
}

#ifdef USE_USART_DRIVER_TIMESTAMPS
uint32_t calcUsartBlockTime(Usart_Object *instHandle, size_t blkLen)
{
	uint32_t ticks;
	uint32_t half_bits_per_char;

	switch(instHandle->devParams.charLen)
	{
	 case Uart_CharLen_5:
		half_bits_per_char=10;
		break;
		
	 case Uart_CharLen_6:
		half_bits_per_char=12;
		break;
		
	 case Uart_CharLen_7:
		half_bits_per_char=14;
		break;
		
	 case Uart_CharLen_9:
		half_bits_per_char=18;
		break;
		
	 case Uart_CharLen_8:
	 default:
		half_bits_per_char=16;
		break;
	}
	half_bits_per_char+=2; // start bit
	if ((instHandle->devParams.parity==Uart_Parity_ODD)||(instHandle->devParams.parity==Uart_Parity_EVEN))
	 	half_bits_per_char+=2;
	switch(instHandle->devParams.stopBits)
	{
	 case Uart_NumStopBits_0_5:
		half_bits_per_char+=1;
		break;
		
	 case Uart_NumStopBits_1_5:
		half_bits_per_char+=3;
		break;
		   
	 case Uart_NumStopBits_2:
		half_bits_per_char+=4;
		break;
	 
	 default:
	 case Uart_NumStopBits_1:
	 	half_bits_per_char+=2;
		break;
	}
	ticks=half_bits_per_char*blkLen*TB_FREQUENCY;
	if ((((uint32_t)2)*(ticks%(((uint32_t)2)*instHandle->devParams.baudRate)))<(((uint32_t)2)*instHandle->devParams.baudRate))
		ticks/=(((uint32_t)2)*instHandle->devParams.baudRate);
	else
	{
		ticks/=(((uint32_t)2)*instHandle->devParams.baudRate);
		ticks++;
	}
	
	return ticks;
}
#endif

#ifdef USE_USART_DRIVER_TIMESTAMPS
uint32_t calcUsartBlockWithIdleTime(Usart_Object *instHandle, size_t blkLen)
{
	uint32_t ticks;
	uint32_t half_bits_per_char;
	uint32_t half_bits_per_idle;

	switch(instHandle->devParams.charLen)
	{
	 case Uart_CharLen_5:
		half_bits_per_char=10;
		break;
		
	 case Uart_CharLen_6:
		half_bits_per_char=12;
		break;
		
	 case Uart_CharLen_7:
		half_bits_per_char=14;
		break;
		
	 case Uart_CharLen_9:
		half_bits_per_char=18;
		break;
		
	 case Uart_CharLen_8:
	 default:
		half_bits_per_char=16;
		break;
	}
	half_bits_per_char+=2; // start bit
	if ((instHandle->devParams.parity==Uart_Parity_ODD)||(instHandle->devParams.parity==Uart_Parity_EVEN))
		half_bits_per_char+=2;
	switch(instHandle->devParams.stopBits)
	{
	 case Uart_NumStopBits_0_5:
		half_bits_per_char+=1;
		break;
		
	 case Uart_NumStopBits_1_5:
		half_bits_per_char+=3;
		break;
		   
	 case Uart_NumStopBits_2:
		half_bits_per_char+=4;
		break;
	 
	 default:
	 case Uart_NumStopBits_1:
		half_bits_per_char+=2;
		break;
	}
	half_bits_per_idle=half_bits_per_char;
	ticks=((half_bits_per_char*blkLen)+half_bits_per_idle)*TB_FREQUENCY;
	if ((((uint32_t)2)*(ticks%(((uint32_t)2)*instHandle->devParams.baudRate)))<(((uint32_t)2)*instHandle->devParams.baudRate))
		ticks/=(((uint32_t)2)*instHandle->devParams.baudRate);
	else
	{
		ticks/=(((uint32_t)2)*instHandle->devParams.baudRate);
		ticks++;
	}
	
	return ticks;
}

#endif
#endif

