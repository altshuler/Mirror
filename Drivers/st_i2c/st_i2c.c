/**
* @file st_i2c.c
* @brief i2c IO driver
*
* @author Eli Schneider
* @author David Anidjar
*
* @version 0.0.1
* @date 12.08.2014
*/

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <iodev.h>
#include <sysport.h>
#include <freertos.h>
#include <task.h>
#include <i2c.h>
#include "st_i2c.h"
#include <board.h>
#include <irqhndl.h>
#ifdef USE_SPI_DRIVER_TIMESTAMPS
#include <timebase.h>
#endif
#include <dma.h>
#include "../st_dma/st_dma.h"

#define I2C_DMA_ENABLE
#define i2c_DMA_ENABLE

#if ((defined(I2C1_SDA_PIN) && defined(I2C1_SDA_GPIO_PORT) && defined(I2C1_SDA_GPIO_CLK)  && defined(I2C1_SDA_PIN_SOURCE)) && (defined(I2C1_SCL_PIN) && defined(I2C1_SCL_GPIO_PORT) && defined(I2C1_SCL_GPIO_CLK)  && defined(I2C1_SCL_PIN_SOURCE)) )
	#define USE_I2C1_GPIO
#endif
#if ((defined(I2C2_SDA_PIN) && defined(I2C2_SDA_GPIO_PORT) && defined(I2C2_SDA_GPIO_CLK)  && defined(I2C2_SDA_PIN_SOURCE)) && (defined(I2C2_SCL_PIN) && defined(I2C2_SCL_GPIO_PORT) && defined(I2C2_SCL_GPIO_CLK)  && defined(I2C2_SCL_PIN_SOURCE)) )
	#define USE_I2C2_GPIO
#endif
#if ((defined(I2C3_SDA_PIN) && defined(I2C3_SDA_GPIO_PORT) && defined(I2C3_SDA_GPIO_CLK)  && defined(I2C3_SDA_PIN_SOURCE)) && (defined(I2C3_SCL_PIN) && defined(I2C3_SCL_GPIO_PORT) && defined(I2C3_SCL_GPIO_CLK)  && defined(I2C3_SCL_PIN_SOURCE)) )
	#define USE_I2C3_GPIO
#endif



#define I2C_MODULE_CLOCK	((uint32_t)400000)

const i2c_Params i2c_PARAMS = {
		i2c_OpMode_INTERRUPT,              /* opMode */
		(uint32_t)400000,               /* outputClkFreq */
		portMAX_DELAY,                    /* timeout */
		{
				i2c_CommMode_MASTER,        /* masterOrSlave */
				1,//i2c_PinOpMode_SPISCS_4PIN,  /* pinOpModes */
				0,
				0,
				0,
				0,
				0,
				0,
    },                              /* spiHWCfgData */
    (((void*)0x0)),            /* hDma */
    (uint16_t)0x0,                    /* HWINumber */
};

/* ========================================================================== */
/*                       GLOBAL MODULE STATE                                  */
/* ========================================================================== */

/**
 *  \brief  I2C Module array map Object
 */
#if defined(USE_I2C1_GPIO) && defined(USE_I2C2_GPIO) && defined(USE_I2C3_GPIO)
#define N_I2C	(3)
static int i2cInstalled[N_I2C] = {1,2,3};

#elif defined(USE_I2C1_GPIO) && defined(USE_I2C2_GPIO)
#define N_I2C	(2)
static int i2cInstalled[N_I2C] = {1,2};

#elif defined(USE_I2C1_GPIO) && defined(USE_I2C3_GPIO)
#define N_I2C	(2)
static int i2cInstalled[N_I2C] = {1,3};

#elif defined(USE_I2C2_GPIO) && defined(USE_I2C3_GPIO)
#define N_I2C	(2)
static int i2cInstalled[N_I2C] = {2,3};

#elif defined(USE_I2C1_GPIO)
#define N_I2C	(1)
static int i2cInstalled[N_I2C] = {1};

#elif defined(USE_I2C2_GPIO)
#define N_I2C	(1)
static int i2cInstalled[N_I2C] = {2};

#elif defined(USE_I2C3_GPIO)
#define N_I2C	(1)
static int i2cInstalled[N_I2C] = {3};
#endif

/**
 *  \brief  Array which is part of I2C Module State
 */
static uint8_t inUse[N_I2C];
/**
 *  \brief  I2C Module State Object
 */
static i2c_Module_State i2c_module = {&inUse[0]};
/**
 *  \brief  Array of I2C instance State objects array
 */
static i2c_Object i2c_Instances[N_I2C];
/**
 *  \brief Global variable used as local buffer for transcieve operation.
 *
 *  It will return data to application in transceieve call and it will dump the
 *  data for other calls.
 */
uint8_t i2c_transReceive[I2C_BUFFER_DATA_SIZE];

i2c_HwInfo i2c_deviceInstInfo[N_I2C];

uint8_t DummyBuff[1];
/* ========================================================================== */
/*                        LOCAL FUNCTION PROTOTYPES                           */
/* ========================================================================== */
static int sti2cBindDev		(void **devp, int devId, void *devParams);
static int sti2cUnBindDev	(void *devp);
static int sti2cCreateChan	(void **chanp, void *devp, char *name, int mode, void *chanParams, IODEV_TiomCallback cbFxn, void *cbArg);
static int sti2cDeleteChan	(void *chanp);
static int sti2cSubmitChan	(void *chanp, IODEV_Packet *packet);
static int sti2cControlChan	(void *chanp, unsigned int cmd, void *arg);

const IODEV_Fxns i2c_IODEVFXNS =
{
    &sti2cBindDev,
    &sti2cUnBindDev,
    &sti2cControlChan,
    &sti2cCreateChan,
    &sti2cDeleteChan,
    &sti2cSubmitChan
};

static int i2cTransfer(i2c_Object *instHandle, I2C_ChanObj *chanHandle, i2c_DataParam *dataparam, uint8_t *inBuffer, int transflags, int timeout, void *xferActual);
static int i2cIoctl(void *handle, i2c_ioctlCmd cmd, void *cmdArg, void *param);
//static void spiRegisterIntrHandler(uint32_t instNum, uint32_t intNum, Spi_isr initIsr, void *spiObj);
static int i2cSetupConfig(I2C_ChanObj *chanHandle);
static int i2cSetupClock(I2C_ChanObj *chanHandle);
//static void i2cConfigureOpMode(const i2c_Object *instHandle);
int i2cIntrHandler(i2c_Object *instHandle);
//static void spiUnregisterIntrHandler(uint32_t instNum, uint32_t intNum);
static void i2cWaitForStaleData(void);
static int i2cCheckTimeOut(portTickType startValue,portTickType timeout);
static int i2cCompleteIOInIsr (i2c_Object *instHandle);
static int i2cPolledModeTransfer(I2C_ChanObj *chanHandle);

int sti2cDmaTxIsr(i2c_Object *instHandle);
int sti2cDmaRxIsr(i2c_Object *instHandle);

static void 	i2c_start		(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);
static void 	i2c_write		(I2C_TypeDef* I2Cx, uint8_t data);
static uint8_t 	i2c_read_ack	(I2C_TypeDef* I2Cx);
static uint8_t 	i2c_read_nack	(I2C_TypeDef* I2Cx);
static void 	i2c_stop		(I2C_TypeDef* I2Cx, uint8_t direction);


//static void i2c_DMARead (I2C_TypeDef* I2Cx,DMA_Stream_TypeDef	*hDmaStream,uint8_t *RxBuff,uint32_t RxBuffLength);
//static void i2c_DMAWrite(I2C_TypeDef* I2Cx,DMA_Stream_TypeDef	*hDmaStream,uint8_t *TxBuff,uint32_t TxBuffLength);
#ifdef i2c_DMA_ENABLE
static void i2cDMA_ClearFlags(DMA_Stream_TypeDef * Streamx);
static void i2c_InitDMA(DMA_Stream_TypeDef * Streamx,uint8_t *Buff,uint32_t BuffLength, uint8_t Channel,uint32_t DMABaseAddress, uint8_t direction);
int i2cCompleteIOdmaCallback (i2c_Object *instHandle);
//static uint32_t i2c_Get_DMA_FLAG_TCIFx(DMA_Stream_TypeDef * Streamx);
static void i2cDMA_ClearFlags(DMA_Stream_TypeDef * Streamx);
static void i2c_InitDMA(DMA_Stream_TypeDef * Streamx,uint8_t *Buff,uint32_t BuffLength, uint8_t Channel,uint32_t DMABaseAddress, uint8_t direction);
#endif

/* ========================================================================== */
/*                        LOCAL Constants                          */
/* ========================================================================== */




/* ========================================================================== */
/*                           MODULE FUNCTIONS                                 */
/* ========================================================================== */

/**
 *  \brief  Function called by Bios during instance initialisation
 *
 */
void sti2cInit(void)
{
    int i;

    for (i = 0; i < N_I2C; i++)
    {
        // have to initialize statically
    	i2c_module.inUse[i] = 0;
        memset((void *)&i2c_Instances[i], 0x0, sizeof(i2c_Object));

        switch(i2cInstalled[i])
        {
			case 1:
				i2c_deviceInstInfo[i].baseAddress = I2C1;
				i2c_deviceInstInfo[i].cpuEventNumber = I2C1_EV_IRQn;
				i2c_deviceInstInfo[i].rxDmaEventNumber = 0;
				i2c_deviceInstInfo[i].txDmaEventNumber = 0;
				i2c_deviceInstInfo[i].inputFrequency = I2C_MODULE_CLOCK;
				//i2c_deviceInstInfo[i].maxChipSelect = 1;
			break;
			case 2:
				i2c_deviceInstInfo[i].baseAddress = I2C2;
				i2c_deviceInstInfo[i].cpuEventNumber = I2C2_EV_IRQn;
				i2c_deviceInstInfo[i].rxDmaEventNumber = 0;
				i2c_deviceInstInfo[i].txDmaEventNumber = 0;
				i2c_deviceInstInfo[i].inputFrequency = I2C_MODULE_CLOCK;
				//i2c_deviceInstInfo[i].maxChipSelect = 1;
			break;
			case 3:
				i2c_deviceInstInfo[i].baseAddress = I2C3;
				i2c_deviceInstInfo[i].cpuEventNumber = I2C3_EV_IRQn;
				i2c_deviceInstInfo[i].rxDmaEventNumber = 0;
				i2c_deviceInstInfo[i].txDmaEventNumber = 0;
				i2c_deviceInstInfo[i].inputFrequency = I2C_MODULE_CLOCK;
				//i2c_deviceInstInfo[i].maxChipSelect = 1;
			break;
        }
    }
}

/**
 *  \brief  Function called by Bios during instance initialisation
 *
 *
 *  \return IODEV_COMPLETED    if success
 *          Error ID                 in case of error
 */
static int sti2cBindDev(void **devp, int devId, void *devParams)
{
    int         	status        = IODEV_COMPLETED;
    I2C_ChanObj   	*chanHandle   = NULL;
    uint8_t         count         = 0;
    i2c_Params    	*params       = NULL;
    i2c_Object    	*instHandle   = NULL;

	/* Begin parameter checking                                                   */
	#ifndef DRV_DISABLE_INPUT_PARAMETER_CHECK
    if ((N_I2C <= devId) || (1 == i2c_module.inUse[devId]))
    {
        status = IODEV_EBADARGS;
    }
	#endif
	/* End parameter checking                                                     */

    if (IODEV_COMPLETED == status)
    {
        if (devParams == NULL)
        {
            params = (i2c_Params*)&i2c_PARAMS;
        }
        else
        {
            params = (i2c_Params*) devParams;
        }

        instHandle =  &i2c_Instances[devId];

        i2c_module.inUse[devId] = 1;

        instHandle->instNum					= devId;
        instHandle->opMode            		= params->opMode;
        instHandle->hDma             		= NULL;
        instHandle->cpuEventNumber			= params->cpuEventNumber;
        instHandle->i2cHWconfig       		= params->i2cHWCfgData;
        instHandle->polledModeTimeout 		= params->polledModeTimeout;
        instHandle->numOpens          		= 0;
        instHandle->devState 				= I2C_DriverState_CREATED;
        instHandle->csHighPolarity    		= 0;
        instHandle->dmaChanAllocated  		= 0;
        instHandle->edmaCbCheck       		= 0;
        instHandle->currentActiveChannel 	= NULL;

        /*Initialize Statistics members                                  */
        instHandle->stats.rxBytes 			= 0;
        instHandle->stats.txBytes 			= 0;
        instHandle->stats.pendingPacket 	= 0;
        instHandle->stats.rxOverrunError 	= 0;
        instHandle->stats.timeoutError 		= 0;
        instHandle->stats.bitError 			= 0;
        instHandle->stats.desyncError 		= 0;

        instHandle->deviceInfo.baseAddress 		= i2c_deviceInstInfo[devId].baseAddress;
        instHandle->deviceInfo.inputFrequency 	= i2c_deviceInstInfo[devId].inputFrequency;
        instHandle->deviceInfo.cpuEventNumber 	= i2c_deviceInstInfo[devId].cpuEventNumber;
        instHandle->deviceInfo.rxDmaEventNumber = i2c_deviceInstInfo[devId].rxDmaEventNumber;
        instHandle->deviceInfo.txDmaEventNumber = i2c_deviceInstInfo[devId].txDmaEventNumber;

        for (count = 0; count < I2C_NUM_CHANS; count++)
        {
                chanHandle                   = &instHandle->chanObj[count];
                chanHandle->cbFxn            = NULL;
                chanHandle->cbArg            = NULL;
                chanHandle->mode             = IODEV_OUTPUT;
                chanHandle->instHandle       = NULL;
                chanHandle->channelState     = I2C_DriverState_CLOSED;
                chanHandle->busFreq          = params->outputClkFreq;
                chanHandle->pendingState     = 0;
                chanHandle->cancelPendingIO  = 0;
                chanHandle->currError        = 0;
                chanHandle->currFlags        = 0;
                chanHandle->transcieveFlags  = 0;
                chanHandle->currBuffer       = NULL;
                chanHandle->transBuffer      = NULL;
                chanHandle->currBufferLen    = 0;
        }
        *devp = (void *)instHandle;
    }
    return (status);
}

/**
 *  \brief  Function called by Bios during closing of the instance
 *
 *
 *  \return None
 */
static int sti2cUnBindDev(void *devp)
{
    int result = IODEV_COMPLETED;
    i2c_Object *instHandle = NULL;

	/* Begin parameter checking                                                   */
	#ifndef DRV_DISABLE_INPUT_PARAMETER_CHECK
    if ((NULL == devp) || (N_I2C <= ((i2c_Object *)devp)->instNum))
    {
        result = IODEV_EBADARGS;
    }
	#endif
	/* End parameter checking                                                     */

    if (IODEV_COMPLETED == result)
    {
        instHandle = (i2c_Object *)devp;
        /* set driver state to deleted                                        */
        instHandle->numOpens = 0;
        instHandle->devState = I2C_DriverState_DELETED;
        i2c_module.inUse[instHandle->instNum] = 0;
    }
    return (result);
}

/* ========================================================================== */
/*                           I2C DRIVER FUNCTIONS                                */
/* ========================================================================== */

/**
 *  \brief  Creates a communication channel in specified mode to communicate
 *          data between the application and the SPI device instance. This
 *          function sets the required hardware configurations for the data
 *          transactions.it returns configured channel handle to application.
 *          which will be used in all further transactions with the channel.
 *
 *          This function is called in response to a SIO_create call.
 *
 * \param     obj          [IN]     i2c driver object
 * \param     name         [IN]     i2c Instance name like Spi0
 * \param     mode         [IN]     channel  mode -> input or output
 * \param     chanParams   [IN]     channel parameters from user
 * \param     cbFxn        [IN]     callback function pointer
 * \param     cbArg        [IN]     callback function Arguments
 *
 * \return    channel handle in case of success
 *            NULL   in case of failure
 *
 */
static int sti2cCreateChan (void **chanp, void *devp, char *name, int mode, void *chanParams, IODEV_TiomCallback cbFxn, void *cbArg)
{
	i2c_Object		*instHandle = (i2c_Object *)devp;
	I2C_ChanObj		*chanHandle = NULL;
	i2c_ChanParams	*pChanParams = NULL;
	uint32_t		key = 0;
	uint32_t		chanCount = 0;
	int				status = IODEV_COMPLETED;
	//xTaskHandle		thisTask   = NULL;
	NVIC_InitTypeDef nvicInit;
	register I2C_TypeDef *i2cRegs;

	/* Begin parameter checking                                                   */
	#ifndef DRV_DISABLE_INPUT_PARAMETER_CHECK
	if ((NULL == instHandle) || (NULL == cbFxn) || (NULL == cbArg))
	{
		status = IODEV_EBADARGS;
	}
	#endif
	/* End parameter checking                                                     */

	i2cRegs = instHandle->deviceInfo.baseAddress;

	if(IODEV_COMPLETED == status)
	{
		do
		{
			for (chanCount = 0; chanCount < I2C_NUM_CHANS; chanCount++)
			{
				if (instHandle->chanObj[chanCount].instHandle == NULL)
				{
					/* Assignment of channel                                  */
					chanHandle = &instHandle->chanObj[chanCount];
					break;
				}
			}

			/* Check if channel is NULL                                       */
			if (NULL == chanHandle)
			{
				status = IODEV_EBADARGS;
				break;
			}

			/* update the instance Handle pointer                             */
			chanHandle->instHandle = (void *)instHandle;

			/* update the channel mode,callback function and args             */
			chanHandle->mode = mode;
			chanHandle->cbFxn = cbFxn;
			chanHandle->cbArg = cbArg;

			/* Returns a handle to the currently executing Task object( );*/
			//thisTask = xTaskGetCurrentTaskHandle();

			/* Get task priority( Task_Handle handle );                   */
			//chanHandle->taskPriority = uxTaskPriorityGet(thisTask);

			if (chanParams==NULL)
				instHandle->hDma = NULL;
			else
			{
				pChanParams = (i2c_ChanParams *)chanParams;
				instHandle->hDma = pChanParams->hDma;
			}

			/* Obtain gpio handle                                             */

			if (i2c_CommMode_SLAVE == instHandle->i2cHWconfig.masterOrSlave)
			{
				if(0 == instHandle->isSlaveChannelOpened)
				{
					instHandle->isSlaveChannelOpened = 1;
				}
				else
				{
					status = IODEV_EBADARGS;
					break;
				}
			}

			/* Update the hardware if it is first open                        */
			if (0 == instHandle->numOpens)
			{
				if (instHandle->deviceInfo.baseAddress == I2C1)
				{
#ifdef USE_I2C1_GPIO
					RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
#endif
				}
				else if (instHandle->deviceInfo.baseAddress == I2C2)
				{
#ifdef USE_I2C2_GPIO
					RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);
#endif
				}
				else if (instHandle->deviceInfo.baseAddress == I2C3)
				{
#ifdef USE_I2C3_GPIO
					RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3,ENABLE);
#endif
				}

				key = __disableInterrupts();

				/* If interrupt mode, register ISR                                */
				if (i2c_OpMode_POLLED != instHandle->opMode)
				{
					installInterruptHandler(instHandle->deviceInfo.cpuEventNumber,i2cIntrHandler,instHandle);

					/* configure interrupts 										  */
					nvicInit.NVIC_IRQChannel = instHandle->deviceInfo.cpuEventNumber;
					nvicInit.NVIC_IRQChannelPreemptionPriority = 15;
					nvicInit.NVIC_IRQChannelSubPriority = 1;
					nvicInit.NVIC_IRQChannelCmd = ENABLE;
					NVIC_Init(&nvicInit);

					// Configure I2C interrupts
					I2C_ITConfig(i2cRegs, (I2C_IT_ERR | I2C_IT_EVT | I2C_IT_BUF), DISABLE);
				}

				#ifdef i2c_DMA_ENABLE
				if (instHandle->hDma!=NULL)
				{
					status = i2cLocalDmaChannel_Request(instHandle);

					if (IODEV_COMPLETED == status)
						instHandle->dmaChanAllocated=1;
				}
				#endif
				if (IODEV_COMPLETED == status)
				{
					/* Initialize hardware                                    */
					status = i2cSetupConfig(chanHandle);
				}

				/* restore the interrupts                                     */
				__restoreInterrupts(key);
			}

			QUE_new(&(chanHandle->queuePendingList));

		}while(0);
	}

	if (IODEV_COMPLETED == status)
	{
		/* Increment open count and return driver handle                  */
		++instHandle->numOpens;
		chanHandle->channelState = I2C_DriverState_OPENED;
		*chanp = (void *)chanHandle;
	}
	else
	{
		/* channel opening failed                                         */
		*chanp = NULL;
	}

	return (status);
}

/**
 *  \brief    This function is called by the application to close a previously
 *            opened channel.it deletes the channel so that it is not available
 *            for further transactions. All the allocated reqources are freed &
 *            the channel will be ready for the "open" operation once again.
 *
 *  \param    instHandle [IN]   i2c driver structure
 *            chanp      [IN]   Handle to the channel.
 *            eb         [OUT]  pointer to the error information block.
 *
 *  \return   None
 */
static int sti2cDeleteChan(void *chanp)
{
	i2c_Object	*instHandle;
	I2C_ChanObj	*chanHandle = (I2C_ChanObj *)chanp;
	uint32_t	key = 0;
	int	status = IODEV_COMPLETED;
	NVIC_InitTypeDef nvicInit;

	/* Begin parameter checking                                                   */
	#ifndef DRV_DISABLE_INPUT_PARAMETER_CHECK
	if ((NULL == chanHandle) || (NULL == chanHandle->instHandle) || (I2C_DriverState_OPENED != chanHandle->channelState))
	{
		status = IODEV_EBADARGS;
	}
	#endif
	/* End parameter checking                                                     */

	if (IODEV_COMPLETED == status)
	{
		instHandle = chanHandle->instHandle;

		/*  Check for any IO is pending, if yes then cancel io            */
		if (0 != chanHandle->pendingState)
		{
			i2cIoctl(chanHandle,i2c_IOCTL_CANCEL_PENDING_IO,NULL,NULL);
		}

		/* disable the global interrupts                                  */
		key = __disableInterrupts();

		/* decrement the number of open channels as we are closing the    *
		* channel                                                        */
		--instHandle->numOpens;

		/* enable the global interrupts                                   */
		__restoreInterrupts(key);

		if (0 == instHandle->numOpens)
		{
			/* all the channels are closed hence unregister the interrupts*/
			if (instHandle->opMode != i2c_OpMode_POLLED)
			{
				/* configure i2c interrupts 										  */
				I2C_ITConfig(instHandle->deviceInfo.baseAddress, (I2C_IT_ERR | I2C_IT_EVT | I2C_IT_BUF), DISABLE);
				nvicInit.NVIC_IRQChannel = instHandle->deviceInfo.cpuEventNumber;
				nvicInit.NVIC_IRQChannelPreemptionPriority = 15;
				nvicInit.NVIC_IRQChannelSubPriority = 1;
				nvicInit.NVIC_IRQChannelCmd = DISABLE;
				NVIC_Init(&nvicInit);
				uninstallInterruptHandler(instHandle->deviceInfo.cpuEventNumber);
			}

			#ifdef i2c_DMA_ENABLE
			/* Free the Edma Channels  if mode is DMA                 */
			if ((i2c_OpMode_INTERRUPTDMA == instHandle->opMode) && (0 != instHandle->dmaChanAllocated))
			{
				#ifdef KUKU
				EDMA3_DRV_freeChannel(
				    instHandle->hEdma,
				    instHandle->deviceInfo.rxDmaEventNumber);

				EDMA3_DRV_freeChannel(
				    instHandle->hEdma,
				    instHandle->deviceInfo.txDmaEventNumber);
				#endif
			}
			else
			{
				/* Dma channel is not allocated,return error          */
				status = IODEV_EBADARGS;
			}
			#endif  /* i2c_DMA_ENABLE */

			/* Put the device in reset mode and quit                      */
			I2C_DeInit(instHandle->deviceInfo.baseAddress);
		}

		chanHandle->instHandle = NULL;
		/* Updated the driver state                                       */
		chanHandle->channelState = I2C_DriverState_CLOSED;
	}
	return (status);
}

/**
 *  \brief    Submit a I/O packet to a channel for processing
 *
 *   The GIO layer calls this function to cause the mini-driver
 *   to process the IOM_Packet for read/write operations.
 *
 *  \param   instHandle [IN]  Spi driver structure pointer
 *  \param   chanp      [IN]  Handle to the channel
 *  \param   ioPacket   [IN]  Pointer to packet to be submitted
 *  \Param   eb         [OUT] error block
 *
 *  \return  IOM_COMPLETED, if packet is fully processed
 *           IOM_PENDING,   if packet is not fully processed
 *           IOM_EBADIO      if error in processing
 */
static int sti2cSubmitChan(void *chanp, IODEV_Packet *ioPacket)
{
	i2c_Object		*instHandle = NULL;
	I2C_ChanObj		*chanHandle = NULL;
	int				status = IODEV_COMPLETED;
	i2c_DataParam	*dataparam = NULL;
	uint32_t		cnt = 0;
	int				transFlags = 0;

	/* Begin parameter checking                                                   */
	#ifndef DRV_DISABLE_INPUT_PARAMETER_CHECK
	if ((NULL == chanp) || (NULL == ioPacket))
	{
		status = IODEV_EBADARGS;
	}
	else
	{
		chanHandle = (I2C_ChanObj *)chanp;
		instHandle = chanHandle->instHandle;

		if (NULL == instHandle)
		{
			status = IODEV_EBADARGS;
		}
	}
	#endif  /* DRV_DISABLE_INPUT_PARAMETER_CHECK */
	/* End parameter checking                                                     */

	if(IODEV_COMPLETED == status)
	{
		do
		{
			/* obtain the Handle to the channel                               */
			chanHandle = (I2C_ChanObj *)chanp;
			instHandle = chanHandle->instHandle;

			if ((ioPacket->cmd == IODEV_ABORT) || (ioPacket->cmd == IODEV_FLUSH))
			{
				status = IODEV_ENOTIMPL;
				break;
			}

			/* Validate the packet send by the user                           */
			if ((ioPacket->cmd == IODEV_READ) || (ioPacket->cmd == IODEV_WRITE))
			{
				if ((ioPacket->addr == NULL) || (ioPacket->size <= 0))
				{
					status = IODEV_EBADIO;
					break;
				}
			}

			dataparam = (i2c_DataParam *)ioPacket->addr;

			/* Validating State of the channel                                */
			if (I2C_DriverState_OPENED  !=  chanHandle->channelState)
			{
				status = IODEV_EBADIO;
				break;
			}

			if (NULL == chanHandle->instHandle)
			{
				status = IODEV_EBADIO;
				break;
			}

			if (ioPacket->cmd == IODEV_WRITE)
			{
				if(dataparam->outBuffer == NULL)
				{
					status = IODEV_EBADIO;
					break;
				}
			}
			if (ioPacket->cmd == IODEV_READ)
			{
				if(dataparam->inBufLen == 0)
				{
					status = IODEV_EBADIO;
					break;
				}
			}

			/* check for zero '0' timeout                                     */
			if (0 == instHandle->polledModeTimeout)
			{
				status = IODEV_EBADIO;
				break;
			}

			/* Update the transceieve handle with source buffer, Check        *
			* if passed spitransbuff is NULL                                 */
			if (NULL == dataparam->inBuffer)
			{
				/* If NULL update spiObj->transBuffer with                    *
				* static buffer                                              */
				for (cnt =0; cnt < I2C_BUFFER_DATA_SIZE; cnt++)
				{
					i2c_transReceive[cnt] = 0;
				}

				chanHandle->transBuffer = &i2c_transReceive[0];

				/* When user does not need transceive operations i.e          *
				* if i2ctranbuff is null update transflags as FALSE          */
				transFlags = 0;
			}
			else
			{
				/* when passed i2ctransbuff is not NULL                       */
				transFlags = 1;
				chanHandle->transBuffer = dataparam->inBuffer;
			}

			/* Check if we need to queue this io request or we should
			* initiate IO here itself of this channel                        */
			if ( NULL == instHandle->currentActiveChannel)
			{
				/* No transfer is happening in any of the channels in this
				* instance of the driver.So for sure we can initiate the
				* transfer here itself                                       */
				chanHandle->activeIOP = ioPacket;

				/* Call the i2cTransfer data transfer through hardware       */
				status =  i2cTransfer(instHandle, chanHandle, dataparam, chanHandle->transBuffer, transFlags, instHandle->polledModeTimeout, (uint32_t *) &cnt);
			}
			else
			{
				/* that means some IO from some channel is in progress we may
				* need to queue the request in pending queue for this channel.
				* one the control comes to the completion of the current IO,
				* the queue will be processed and this IO will be programmed
				* In the priority based implementation, please note that only the
				* channel that was created from the task with high priority would
				* be processed first                                             */
				QUE_put(&(chanHandle->queuePendingList),(void *)ioPacket);
				instHandle->stats.pendingPacket++;
			}
		}while(0);
	}

	if (IODEV_COMPLETED == status)
	{
		status = IODEV_COMPLETED;
	}
	else if (IODEV_PENDING == status)
	{
		status = IODEV_PENDING;
	}
	else
	{
		status = IODEV_EBADIO;
	}

	return status;
}

/**
 *  \brief   This function executes a control command passed by the application
 *
 *   The application's request for a IOCTL to be executed is routed here by the
 *   stream. if the command is supported then the control command is executed.
 *
 *  \param    instHandle [IN]    i2c driver structure
 *  \param    chanp      [IN]    Channel handle
 *  \param    cmd        [IN]    control command given by the application
 *  \param    cmdArgs    [IN]    Optional args required for command execution
 *  \param    eb         [OUT]   error block
 *
 *  \return   None
 */
static int sti2cControlChan(void *chanp, unsigned int cmd, void *cmdArg)
{
	I2C_ChanObj	*chanHandle = (I2C_ChanObj *)chanp;
	int			status      = IODEV_COMPLETED;

	/* Begin parameter checking                                                   */
	#ifndef DRV_DISABLE_INPUT_PARAMETER_CHECK
	if ((NULL == chanp) || (I2C_DriverState_OPENED!= chanHandle->channelState))
	{
		/* invalid params have been detected                                  */
		status = IODEV_EBADARGS;
	}
	#endif /* DRV_DISABLE_INPUT_PARAMETER_CHECK */
	/* End parameter checking                                                     */

	if (IODEV_COMPLETED == status)
	{
		/* call the function to execute the control commands              */
		status = i2cIoctl(chanHandle, (i2c_ioctlCmd)cmd,(void *) cmdArg, NULL);
	}

	return (status);
}

/* ========================================================================== */
/*                            LOCAL  FUNCTIONS                                */
/* ========================================================================== */

/**
 *  \brief  This to configure the data control bit of I2C data register .
 *  \param  handle      [IN]   i2c driver object for respective instance.
 *
 */
void i2cLocalControlDataConfig(I2C_ChanObj *chanHandle)
{
    register i2c_Object     *instHandle;
	register I2C_TypeDef *i2cRegs;

    assert(NULL != chanHandle);

    instHandle = chanHandle->instHandle;

    i2cRegs=instHandle->deviceInfo.baseAddress;

	i2cSetupConfig(chanHandle);

	/* Disable I2C 							  */
	I2C_Cmd(i2cRegs, DISABLE);
	//I2C_Init(i2cRegs, &i2cInitSt);

	/* Enable I2C for further transaction							  */
	I2C_Cmd(i2cRegs, ENABLE);

	/* To make transaction is in progress                                     */
    chanHandle->pendingState = 1;
}

/**
 *  \brief  Function call for i2c data transfer. It transfer data as transcieve
 *         operation.If transflags is true i.e. user can access read & write
 *         data. when transflags is FALSE and flags is write - Read data is
 *         dumped to local buffer (no access to user). When transflags is
 *         FALSE and flags is Read - Write data is dumped to local buffer.
 *
 *  \param  handle      [IN]    SPI Driver handle
 *  \param  dataparam   [IN]    User data parameters for data Source.
 *  \param  inBuffer    [IN]    User or local buffer source for transceieve
 *                             operation.( User or local buffer selection
 *                             depends on transflags status ).
 *  \param  transflags  [IN]    Flag for transceieve True - Used user buffer
 *                             for Transceieve, False - Local buffer for
 *                             Transceieve.
 *  \param  timeout     [IN]    Timeout Value.
 *  \param  xferActual  [OUT]   Transfer byte count
 *
 *  \return            IOM_COMPLETED if success
 *                    Suitable error code
 */
static int i2cTransfer(i2c_Object *instHandle, I2C_ChanObj *chanHandle, i2c_DataParam *dataparam, uint8_t *inBuffer, int transflags, int timeout, void *xferActual)
{
	int				retVal = IODEV_COMPLETED;
	register uint8_t			*outBuffer/* = NULL*/;
	register uint32_t		outBufLen/* = 0*/;
	uint32_t		flags/* = 0*/;
	//uint16_t		deviceAddress/* = 0*/;

	//register i2c_HWConfigData *hwc;
	register I2C_TypeDef *i2cRegs;

	assert((NULL != instHandle)&&(NULL != chanHandle) && (NULL != dataparam)&&(NULL != inBuffer));

	flags      = dataparam->flags;
	outBuffer  = (uint8_t *)&dataparam;
	outBufLen  = sizeof(dataparam);

	//deviceAddress = dataparam->deviceAddress;
	instHandle->currentActiveChannel = chanHandle;

	i2cRegs=instHandle->deviceInfo.baseAddress;
	//hwc=&instHandle->i2cHWconfig;


	/** If polled mode, wait for data transfer & completion, else wait     *
	* (on a SEM) for transaction to complete in interrupt mode      *
	* In interrupt mode, for write case, it has to poll on RX_INT bit,    *
	* as there is no transmit interrupt bit to set                        */
	if (i2c_OpMode_INTERRUPTDMA == instHandle->opMode)
	{
		/* DMAINTERRUPT Mode of data transfer                             */
		#ifdef i2c_DMA_ENABLE
		/* Mask all interrupts,& populate data to spi object structure*
		* so driver can extact all information at dma configSpiion  *
		* and dma Rx & Tx callback                                   */


		/* Transmit data                                              */
//		while (I2C_GetFlagStatus(i2cRegs,I2C_FLAG_RXNE))
//		{
//			/* Read before Write to remove any stale data             */
//			*chanHandle->transBuffer = (uint8_t)(I2C_ReceiveData(i2cRegs));
//		}

		/*disable all I2C interrupts */
		I2C_ITConfig(i2cRegs, (I2C_IT_ERR | I2C_IT_EVT | I2C_IT_BUF), DISABLE);

		chanHandle->currError           = 0;
		chanHandle->currFlags           = flags;
		chanHandle->transcieveFlags     = transflags;
		chanHandle->currBuffer          = outBuffer;
		chanHandle->currBufferLen       = outBufLen;

		/* Call this to complete dma configuration and transfer      */
		retVal = i2cLocalDmaTransfer(instHandle);

		if (IODEV_COMPLETED ==  retVal)
		{
			retVal = IODEV_PENDING;
		}

		#endif/* DMA MODE CODE ENDS                                                  */
	}

	/* POLLED MODE CODE STARTS                                                */
	else if (i2c_OpMode_POLLED == instHandle->opMode)
	{
		//i2cLocalControlDataConfig(chanHandle);
		retVal = i2cPolledModeTransfer(chanHandle);

	}/*POLLED MODE CODE ENDS                                                  */


	/* INTERUPPT MODE CODE STARTS                                             */
	else if (i2c_OpMode_INTERRUPT == instHandle->opMode)
	{
		/* Populate the user send params to i2c object structure              *
		* So device can use all the information when running at  ISR.        */

		chanHandle->currError         = 0;
		chanHandle->currFlags         = flags;
		chanHandle->transcieveFlags   = transflags;
		chanHandle->currBuffer        = outBuffer;
		chanHandle->currBufferLen     = outBufLen;
		chanHandle->transBuffer		  = outBuffer;
		chanHandle->activeIOP->addr  = dataparam;

		chanHandle->dataParam.deviceAddress = dataparam->deviceAddress;
		chanHandle->dataParam.flags = dataparam->flags;
		chanHandle->dataParam.inBufLen = dataparam->inBufLen;
		chanHandle->dataParam.inBuffer = dataparam->inBuffer;
		chanHandle->dataParam.outBufLen = dataparam->outBufLen;
		chanHandle->dataParam.outBuffer = dataparam->outBuffer;
		chanHandle->dataParam.param = dataparam->param;

		/*Consume any stale data                                              */
		//while (I2C_GetFlagStatus(i2cRegs,I2C_FLAG_RXNE))
		//{
		//	/* Read before Write to remove any stale data             */
		//	*chanHandle->transBuffer = (uint8_t)(I2C_ReceiveData(i2cRegs));
		//}

		i2cLocalControlDataConfig(chanHandle);

		if ((chanHandle->currBufferLen) > 0)
		{
			/* Enabling receive and error interrupts						  */
			I2C_ITConfig(i2cRegs,I2C_IT_BUF|I2C_IT_EVT|I2C_IT_ERR,ENABLE);
			I2C_GenerateSTART(i2cRegs, ENABLE);
		}
		retVal = IODEV_PENDING;
	} /* INTERUPPT MODE CODE ENDS                                             */

	return retVal;
}

/**
 *  \brief  function is used to transfer data in polled mode
 *
 *  This function is used to transfer data in polled mode.
 *
 *  \param  chanHandle  [IN]    Channel handle
 *
 *  \return IODEV_COMPLETED in case of success
 *          IODEV_ERROR in case of failure
 *
 *  \enter  chanHandle  must be a valid pointer and should not be null.
 *
 *  \leave  Not Implemented.
 */
static int i2cPolledModeTransfer(I2C_ChanObj *chanHandle)
{
	int				status = IODEV_COMPLETED;
	//uint32_t		timeCnt = 0;
	uint32_t		cnt = 0;
	uint32_t		len = 0;
	//int             retStatus = 0;
	uint8_t			*outBuffer = NULL;
	uint8_t			*inBuffer = NULL;
	//int				transflags = 0;
	i2c_Object		*instHandle = NULL;
	i2c_DataParam	*dataParam  = NULL;
	I2C_TypeDef		*i2cRegs;

	assert(NULL != chanHandle);

	instHandle = chanHandle->instHandle;

	i2cRegs=instHandle->deviceInfo.baseAddress;

	/* Polled mode of data transfer Mask all interrupts                       */
	I2C_ITConfig(i2cRegs,I2C_IT_BUF|I2C_IT_EVT|I2C_IT_ERR,DISABLE);

	/* To get current ticks to find out the data transfer timeout             */
	//timeCnt = xTaskGetTickCount();

	dataParam = (i2c_DataParam *)chanHandle->activeIOP->addr;
	outBuffer = dataParam->outBuffer;
	inBuffer = dataParam->inBuffer;

//	if (NULL == dataParam->inBuffer)
//		transflags = 0;
//	else
//		transflags = 1;

	i2cLocalControlDataConfig(chanHandle);

	if(dataParam->flags == I2C_WRITE)
	{
		i2c_start(i2cRegs, dataParam->deviceAddress, I2C_Direction_Transmitter);
		len = dataParam->outBufLen;
	}
	else
	{
		i2c_start(i2cRegs, dataParam->deviceAddress, I2C_Direction_Receiver);
		len = dataParam->inBufLen;
	}

	while (cnt < len)
	{
		if(dataParam->flags == I2C_WRITE)
		{
			i2c_write(i2cRegs, outBuffer[cnt]); // write one byte to the slave
			cnt++;
		}
		else
		{
			if(cnt == len-1)
				inBuffer[cnt] = i2c_read_nack(i2cRegs);
			else
				inBuffer[cnt] = i2c_read_ack(i2cRegs); // write one byte to the slave
			cnt++;
		}

//		if ((0 == i2cCheckTimeOut(timeCnt,instHandle->polledModeTimeout))&& (IODEV_COMPLETED == status))
//		{
//
//		}
//		else
//		{
//			if (IODEV_COMPLETED == status)
//			{
//				status = IODEV_EBADIO;
//				instHandle->stats.timeoutError++;
//			}
//			break;
//		}

	}/* While  */

	if(dataParam->flags == I2C_WRITE)
	{
		i2c_stop(i2cRegs,I2C_Direction_Transmitter);
		instHandle->stats.txBytes += cnt;
	}
	else
	{
		i2c_stop(i2cRegs,I2C_Direction_Receiver);
		instHandle->stats.rxBytes += cnt;
	}

	instHandle->currentActiveChannel = NULL;

	return status;
}


/**
 *  \brief  IO Control for SPI device. Currently it is used for loopback only
 *
 *  \param  handle      [IN]    Spi Driver instance handle.
 *  \param  cmd         [IN]    Command to do operation.
 *  \param  cmdArg      [IN]    Additional parameters required for the command
 *  \param  param       [IN]    For future reference.
 *  \param  eb          [OUT]   error block
 *
 *  \return None
 *
 */
static int i2cIoctl(void *handle, i2c_ioctlCmd cmd, void *cmdArg,void *param)
{
    uint32_t	key = 0;
    I2C_ChanObj	*chanHandle = NULL;
    i2c_Object	*instHandle = NULL;
    int			status = IODEV_COMPLETED;

    assert(NULL != handle);

    chanHandle = (I2C_ChanObj *)handle ;
    instHandle = chanHandle->instHandle;

    if (i2c_IOCTL_CANCEL_PENDING_IO == cmd)
    {
        /* Start of Critical Section                                      */
        key = __disableInterrupts();

        /* Calling this when no iopacket is in transmission               */
        if (0 != chanHandle->pendingState)
        {
            /* to cancel io                                               */
            chanHandle->cancelPendingIO = 1;
        }
        else
        {
            status = IODEV_EBADMODE;
        }
        /* End of Critical Section                                        */
        __restoreInterrupts(key);
    }
    else if (i2c_IOCTL_SET_CS_POLARITY == cmd)
    {
        if (NULL != cmdArg)
        {
            instHandle->csHighPolarity = *(int *)cmdArg;
        }
        else
        {
            status = IODEV_EBADARGS;
        }
    }
    else if (i2c_IOCTL_SET_POLLEDMODETIMEOUT == cmd)
    {
        /* Update the polledModeTimeout value                                 */
        instHandle->polledModeTimeout = *((uint32_t *)cmdArg);
    }
    else if (i2c_IOCTL_SET_CLOCK_RATE== cmd)
    {
        /* Update the polledModeTimeout value                                 */
        chanHandle->busFreq = *((uint32_t *)cmdArg);

		status=i2cSetupClock(chanHandle);
    }
    /* Unrecognised Command                                                   */
    else
        status = IODEV_EBADARGS;

    return (status);
}









/**
 *  \brief   function is used after the completion of ISR
 *
 *  This function is called after Interrupt routine is proccessed out.
 *  This functions ensure whether driver is ready for next operation or not.
 *  Also it gets the next available channel to be processed for data transfer.
 *
 *  \param   instHandle [IN] pointer to the spi driver object
 *
 *  \return  None
 *
 *  \enter   instHandle  must be a valid pointer and should not be null.
 *
 *  \leave   Not Implemented.
 */
static int i2cCompleteIOInIsr (i2c_Object *instHandle)
{
    register I2C_ChanObj		*chanHandle;
    register i2c_DataParam	*dataParam;
    register IODEV_Packet	*ioPacket;
	register I2C_TypeDef *i2cRegs;
	int completeIo=0;

    assert(NULL != instHandle);

    chanHandle = instHandle->currentActiveChannel;
	ioPacket=chanHandle->activeIOP;
    dataParam  = (i2c_DataParam *)ioPacket->addr;
    i2cRegs=instHandle->deviceInfo.baseAddress;

    /* Check the Pending State   TBD to test                                  */
    if (0 != chanHandle->cancelPendingIO)
    {
        /* Error of Cancel IO                                                 */
        ioPacket->status = IODEV_EBADIO;
        chanHandle->cancelPendingIO = 0;
        ioPacket->size = chanHandle->currBufferLen;
        //chanHandle->charLength16Bits = 0;
        /*set IOP error with proper error raise value TBD                     */
    }

    /* check if CSHOLD needs to be enable after the transmission also.        *
     * If No or if there is any eror disable CSHOLD value, else do nothing    *
     * Also toggle the gpio pin if gpioChipSelect is enabled                  */


    /* Mask off interrupts                                                    */

	//SPI_I2S_ITConfig(spiRegs,SPI_I2S_IT_TXE,DISABLE);
	/* Disable Receive interrupt										  */
	//I2C_ITConfig(i2cRegs,I2C_IT_RXNE,DISABLE);
    I2C_ITConfig(i2cRegs,I2C_IT_BUF|I2C_IT_EVT|I2C_IT_ERR,DISABLE);
	//I2C_ITConfig(i2cRegs,I2C_IT_ERR,DISABLE);

    /* call the application completion callback function registered           *
     * with us during opening of the channel                                  */
    if (NULL != chanHandle->cbFxn)
    {
        /* Invoke Application callback for this channel                       */
		completeIo=(*chanHandle->cbFxn)((void *)chanHandle->cbArg, ioPacket);
		//completeIo=1;
    }

    chanHandle->activeIOP = NULL;

    /*get the channel with highest priority                                   */
    i2cLocalGetNextChannel(instHandle, &(instHandle->currentActiveChannel));

    /*store for local use                                                     */
    chanHandle = instHandle->currentActiveChannel;

    /* check and load next pending packet                                     */
    if (NULL != chanHandle)
    {
        /* we have atleast one packet                                         */
        ioPacket = (IODEV_Packet *)QUE_get(&(chanHandle->queuePendingList));
        instHandle->stats.pendingPacket--;

        /* validate and update the iop                                        */
        if (NULL  != ioPacket)
        {
            chanHandle->activeIOP = ioPacket;
            dataParam = (i2c_DataParam *)ioPacket->addr;

            i2cLocalControlDataConfig(chanHandle);

            /* Set the current buffer params correctly                        */
            chanHandle->currError         = 0;
            chanHandle->currFlags         = dataParam->flags;
            chanHandle->transcieveFlags   = (NULL == dataParam->inBuffer)? 0 : 1; /* transflags */
            chanHandle->currBuffer        = dataParam->outBuffer;
            chanHandle->currBufferLen     = dataParam->outBufLen;

            /*Consume any stale data                                          */

//

			/* Enabling receive and error interrupts                      */
			I2C_ITConfig(i2cRegs,I2C_IT_BUF|I2C_IT_EVT|I2C_IT_ERR,ENABLE);
			//I2C_ITConfig(i2cRegs,SPI_I2S_IT_RXNE,ENABLE);
			//I2C_ITConfig(i2cRegs,SPI_I2S_IT_OVR,ENABLE);
		}/**< if currBuffLen                                              */
    }
  	return completeIo;
}

/**
 *  \brief  function is used get the next channel
 *
 *  This function is used to get the next highest priority channel. Also it
 *  checks for any pending data in that stream.
 *
 *  \param  instHandle  [IN]    handle to the SPI instance
 *  \param  pChanHandle [OUT]   pointer to the channel handle
 *
 *  \return None
 *
 *  \enter  instHandle  must be a valid pointer and should not be null.
 *          pChanHandle must be a valid pointer and should not be null.
 *
 *  \leave  Not Implemented.
 *
 */
void i2cLocalGetNextChannel(i2c_Object *instHandle, I2C_ChanObj **pChanHandle)
{
	#if (1<i2c_NUM_CHANS)
	register uint16_t counter;
	register uint16_t chanIndexWithMaxPri = i2c_NUM_CHANS;
	register uint16_t lastFoundMaxPri     = 0;

	assert(NULL != instHandle);

	for(counter=0; counter<i2c_NUM_CHANS; counter++)
	{
		if((i2c_DriverState_OPENED == instHandle->chanObj[counter].channelState) && (lastFoundMaxPri < instHandle->chanObj[counter].taskPriority))
		{
			if(0 == QUE_empty(&(instHandle->chanObj[counter].queuePendingList)))
			{
				lastFoundMaxPri = instHandle->chanObj[counter].taskPriority;
				chanIndexWithMaxPri = counter;
			}
		}
	}

	if (i2c_NUM_CHANS != chanIndexWithMaxPri)
	{
		*pChanHandle = &(instHandle->chanObj[chanIndexWithMaxPri]);
	}
	else
	{
		*pChanHandle = NULL;
	}
	#else
	register uint16_t chanIndexWithMaxPri = I2C_NUM_CHANS;
	//register uint16_t lastFoundMaxPri     = 0;

	if((I2C_DriverState_OPENED == instHandle->chanObj[0].channelState) )
	{
		if(0 == QUE_empty(&(instHandle->chanObj[0].queuePendingList)))
		{
			//lastFoundMaxPri = instHandle->chanObj[0].taskPriority;
			chanIndexWithMaxPri = 0;
		}
	}
	if (I2C_NUM_CHANS != chanIndexWithMaxPri)
	{
		*pChanHandle = &(instHandle->chanObj[chanIndexWithMaxPri]);
	}
	else
	{
		*pChanHandle = NULL;
	}
	#endif
}

/**
 *  \brief  It will configure the SPI hardware as per user configuration params.
 *         Before setting the Hardware it will put the SPI device in reset and
 *         again out of reset too before setting hardware.
 *         It will set the enable bit of the SPI device.
 *
 *  \param  instHandle   [IN]  pointer to the spi driver object instance
 *  \param  eb           [OUT] error block
 *
 *  \return IODEV_COMPLETED if success
 *         else error code in case of failure
 */
static int i2cSetupConfig(I2C_ChanObj *chanHandle)
{
	int					status 		= IODEV_COMPLETED;
	i2c_Object			*instHandle = NULL;
	register 			I2C_TypeDef *i2cRegs;
	I2C_InitTypeDef 	i2cInitSt;
	GPIO_InitTypeDef 	gpioInitSt;

	assert(NULL != chanHandle);

	instHandle = chanHandle->instHandle;

	assert(NULL != instHandle);

	i2cRegs = instHandle->deviceInfo.baseAddress;

	if (IODEV_COMPLETED == status)
	{
		if(i2cRegs == I2C1)
		{
#ifdef USE_I2C1_GPIO
			GPIO_StructInit(&gpioInitSt);

			/* RCC Configuration */
			/*I2C Peripheral clock enable */
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

			/*SDA GPIO clock enable */
			RCC_AHB1PeriphClockCmd(I2C1_SDA_GPIO_CLK, ENABLE);

			/*SCL GPIO clock enable */
			RCC_AHB1PeriphClockCmd(I2C1_SCL_GPIO_CLK, ENABLE);

			/* Reset I2Cx IP */
			RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);

			/* Release reset signal of I2Cx IP */
			RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);

			/* GPIO Configuration */
			/*Configure I2C SCL pin */
			gpioInitSt.GPIO_Pin = I2C1_SCL_PIN;
			gpioInitSt.GPIO_Mode = GPIO_Mode_AF;
			gpioInitSt.GPIO_Speed = GPIO_Speed_50MHz;
			gpioInitSt.GPIO_OType = GPIO_OType_OD;
			gpioInitSt.GPIO_PuPd  = GPIO_PuPd_UP;
			GPIO_Init(I2C1_SCL_GPIO_PORT, &gpioInitSt);

			/*Configure I2C SDA pin */
			gpioInitSt.GPIO_Pin = I2C1_SDA_PIN;
			GPIO_Init(I2C1_SDA_GPIO_PORT, &gpioInitSt);

			/* Connect PXx to I2C_SCL */
			GPIO_PinAFConfig(I2C1_SCL_GPIO_PORT, I2C1_SCL_PIN_SOURCE, GPIO_AF_I2C1);

			/* Connect PXx to I2C_SDA */
			GPIO_PinAFConfig(I2C1_SDA_GPIO_PORT, I2C1_SDA_PIN_SOURCE, GPIO_AF_I2C1);
#endif
		}
		else if(i2cRegs == I2C2)
		{
#ifdef USE_I2C2_GPIO
			GPIO_StructInit(&gpioInitSt);

			/* RCC Configuration */
			/*I2C Peripheral clock enable */
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

			/*SDA GPIO clock enable */
			RCC_AHB1PeriphClockCmd(I2C2_SDA_GPIO_CLK, ENABLE);

			/*SCL GPIO clock enable */
			RCC_AHB1PeriphClockCmd(I2C2_SCL_GPIO_CLK, ENABLE);

			/* Reset I2Cx IP */
			RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, ENABLE);

			/* Release reset signal of I2Cx IP */
			RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, DISABLE);

			/* GPIO Configuration */
			/*Configure I2C SCL pin */
			gpioInitSt.GPIO_Pin = I2C2_SCL_PIN;
			gpioInitSt.GPIO_Mode = GPIO_Mode_AF;
			gpioInitSt.GPIO_Speed = GPIO_Speed_50MHz;
			gpioInitSt.GPIO_OType = GPIO_OType_OD;
			gpioInitSt.GPIO_PuPd  = GPIO_PuPd_UP;
			GPIO_Init(I2C2_SCL_GPIO_PORT, &gpioInitSt);

			/*Configure I2C SDA pin */
			gpioInitSt.GPIO_Pin = I2C2_SDA_PIN;
			GPIO_Init(I2C2_SDA_GPIO_PORT, &gpioInitSt);

			/* Connect PXx to I2C_SCL */
			GPIO_PinAFConfig(I2C2_SCL_GPIO_PORT, I2C2_SCL_PIN_SOURCE, GPIO_AF_I2C2);

			/* Connect PXx to I2C_SDA */
			GPIO_PinAFConfig(I2C2_SDA_GPIO_PORT, I2C2_SDA_PIN_SOURCE, GPIO_AF_I2C2);
#endif
		}
		else if(i2cRegs == I2C3)
		{
#ifdef USE_I2C3_GPIO
			GPIO_StructInit(&gpioInitSt);

			/* RCC Configuration */
			/*I2C Peripheral clock enable */
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);

			/*SDA GPIO clock enable */
			RCC_AHB1PeriphClockCmd(I2C3_SDA_GPIO_CLK, ENABLE);

			/*SCL GPIO clock enable */
			RCC_AHB1PeriphClockCmd(I2C3_SCL_GPIO_CLK, ENABLE);

			/* Reset I2Cx IP */
			RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C3, ENABLE);

			/* Release reset signal of I2Cx IP */
			RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C3, DISABLE);

			/* GPIO Configuration */
			/*Configure I2C SCL pin */
			gpioInitSt.GPIO_Pin = I2C3_SCL_PIN;
			gpioInitSt.GPIO_Mode = GPIO_Mode_AF;
			gpioInitSt.GPIO_Speed = GPIO_Speed_50MHz;
			gpioInitSt.GPIO_OType = GPIO_OType_OD;
			gpioInitSt.GPIO_PuPd  = GPIO_PuPd_UP;
			GPIO_Init(I2C3_SCL_GPIO_PORT, &gpioInitSt);

			/*Configure I2C SDA pin */
			gpioInitSt.GPIO_Pin = I2C3_SDA_PIN;
			GPIO_Init(I2C3_SDA_GPIO_PORT, &gpioInitSt);

			/* Connect PXx to I2C_SCL */
			GPIO_PinAFConfig(I2C3_SCL_GPIO_PORT, I2C3_SCL_PIN_SOURCE, GPIO_AF_I2C3);

			/* Connect PXx to I2C_SDA */
			GPIO_PinAFConfig(I2C3_SDA_GPIO_PORT, I2C3_SDA_PIN_SOURCE, GPIO_AF_I2C3);
#endif
		}
		I2C_AcknowledgeConfig(i2cRegs, ENABLE);
		/* I2C ENABLE */
		I2C_Cmd(i2cRegs, ENABLE);

		/*Put the module in reset mode                                    */
		/* Bring module out of reset                                      */
		I2C_DeInit(i2cRegs);

		//if(instHandle->i2cHWconfig.i2c_Mode == i2c_CommMode_MASTER)
		i2cInitSt.I2C_Mode = I2C_Mode_I2C;

		if(instHandle->i2cHWconfig.i2c_DutyCycle == 1)
			i2cInitSt.I2C_DutyCycle = I2C_DutyCycle_2;
		else
			i2cInitSt.I2C_DutyCycle = I2C_DutyCycle_16_9;

		i2cInitSt.I2C_OwnAddress1 = instHandle->i2cHWconfig.i2c_OwnAddress;

		if(instHandle->i2cHWconfig.i2c_Ack == 1)
			i2cInitSt.I2C_Ack = I2C_Ack_Enable;
		else
			i2cInitSt.I2C_Ack = I2C_Ack_Disable;

		i2cInitSt.I2C_ClockSpeed = instHandle->i2cHWconfig.i2c_ClockSpeed ;
		i2cInitSt.I2C_AcknowledgedAddress = instHandle->i2cHWconfig.i2c_AcknowledgedAddress;
		I2C_Init(i2cRegs, &i2cInitSt);

	}/*< if status inside  if status                                          */

	return (status); /* success  or error code                                */
}

/**
 *  \brief  It will configure the SPI clock rate as per user configuration params.
 *
 *  \param  instHandle   [IN]  pointer to the spi driver object instance
 *  \param  eb           [OUT] error block
 *
 *  \return IOM_COMPLETED if success
 *         else error code in case of failure
 */
static int i2cSetupClock(I2C_ChanObj *chanHandle)
{
	int             status   = IODEV_COMPLETED;
	//int             count    = 0;
	i2c_Object		*instHandle = NULL;
	//register I2C_TypeDef *i2cRegs;

	assert(NULL != chanHandle);

	instHandle = chanHandle->instHandle;

	assert(NULL != instHandle);

//	i2cRegs=instHandle->deviceInfo.baseAddress;
//	/* clock is enabled in master mode only                               */
//	if (i2c_CommMode_MASTER == instHandle->i2cHWconfig.masterOrSlave)
//	{
//		/* Prescale value calculation                                     */
//		RCC_GetClocksFreq(&RCC_ClocksStatus);
//
//		if ((i2cRegs == SPI2) || (i2cRegs == SPI3))
//		{
//		  apbclock = RCC_ClocksStatus.PCLK1_Frequency;
//		}
//		else
//		{
//		  apbclock = RCC_ClocksStatus.PCLK2_Frequency;
//		}
//		prescale = spiPrescaler(apbclock,chanHandle->busFreq);
//
//		/* prescalar is a 8 bit register and can hold maximum 255         */
//		if (!IS_SPI_BAUDRATE_PRESCALER(prescale) )
//		{
//			status = IODEV_EBADARGS;
//		}
//	}


//	if (IODEV_COMPLETED == status)
//	{
//		i2cRegs->CR1=(i2cRegs->CR1 & ~SPI_BaudRatePrescaler_256) | prescale;
//	}
	return (status); /* success  or error code                                */
}

/**
 *  \brief  This to configure the delay for stale sdata
 *
 *  \Param  None
 *
 *  \return None
 *
 */
//void i2cWaitForStaleData(void)
//{
//    volatile uint32_t count = 0x2FFFu;
//
//    /* Waits for some period of time to stale the data                        */
//    while (0 != count)
//    {
//        count--;
//    }
//}

/**
 *  \brief   checks if a timeout has happened from the given time.
 *
 *  This function checks if the timeout with respect to the start time has
 *  occured. The timeout and the start time is also input to the function.
 *  it checks the current time and compares with the recorded start time value
 *  and then specified timeout. The function returns TRUE in case of time out
 *  else FALSE is returned.
 *
 *  \param   startValue  [IN]   Start value
 *  \param   timeout     [IN]   Timeout value
 *
 *  \return  TRUE   if time out occured.
 *          FALSE  if time out has not occured.
 *
 */
static int i2cCheckTimeOut(portTickType startValue,portTickType timeout)
{
    portTickType  checkValue = 0;
    int    retVal     = 0;

    /* get the current tick value and compare with the start value            */
    checkValue = xTaskGetTickCount();

    /* check if the current tick count counter has overflowed and suitably    *
     * calculate the elapsed time                                             */
    if (checkValue < startValue)
    {
        checkValue = (((portMAX_DELAY) - startValue) + checkValue) + (1U) ;
    }
    else
    {
        checkValue = checkValue - startValue;
    }

    /* if the elapsed time is greater than start tick then set retval to TRUE *
     * to indicate time out else send false                                   */
    if (checkValue < timeout)
    {
        retVal  =   0;
    }
    else
    {
        retVal  =   1;
    }
    return  retVal;
}

#ifdef I2C_DMA_ENABLE
#define DISCRETE_DMA_STRUCT_INIT

//static uint32_t i2c_Get_DMA_FLAG_TCIFx(DMA_Stream_TypeDef * Streamx)
//{
//	if((Streamx == DMA1_Stream0) || (Streamx == DMA2_Stream0))
//	{
//		return DMA_FLAG_TCIF0;
//	}
//	else if((Streamx == DMA1_Stream1) || (Streamx == DMA2_Stream1))
//	{
//		return DMA_FLAG_TCIF1;
//	}
//	else if((Streamx == DMA1_Stream2) || (Streamx == DMA2_Stream2))
//	{
//		return DMA_FLAG_TCIF2;
//	}
//	else if((Streamx == DMA1_Stream3) || (Streamx == DMA2_Stream3))
//	{
//		return DMA_FLAG_TCIF3;
//	}
//	else if((Streamx == DMA1_Stream4) || (Streamx == DMA2_Stream4))
//	{
//		return DMA_FLAG_TCIF4;
//	}
//	else if((Streamx == DMA1_Stream5) || (Streamx == DMA2_Stream5))
//	{
//		return DMA_FLAG_TCIF5;
//	}
//	else if((Streamx == DMA1_Stream6) || (Streamx == DMA2_Stream6))
//	{
//		return DMA_FLAG_TCIF6;
//	}
//	else if((Streamx == DMA1_Stream7) || (Streamx == DMA2_Stream7))
//	{
//		return DMA_FLAG_TCIF7;
//	}
//	else
//		return 0;
//}

static void i2cDMA_ClearFlags(DMA_Stream_TypeDef * Streamx)
{
	if((Streamx == DMA1_Stream0) || (Streamx == DMA2_Stream0))
	{
		DMA_ClearFlag(Streamx,DMA_FLAG_TCIF0|DMA_FLAG_FEIF0|DMA_FLAG_DMEIF0|DMA_FLAG_TEIF0|DMA_FLAG_HTIF0);
	}
	else if((Streamx == DMA1_Stream1) || (Streamx == DMA2_Stream1))
	{
		DMA_ClearFlag(Streamx,DMA_FLAG_TCIF1|DMA_FLAG_FEIF1|DMA_FLAG_DMEIF1|DMA_FLAG_TEIF1|DMA_FLAG_HTIF1);
	}
	else if((Streamx == DMA1_Stream2) || (Streamx == DMA2_Stream2))
	{
		DMA_ClearFlag(Streamx,DMA_FLAG_TCIF2|DMA_FLAG_FEIF2|DMA_FLAG_DMEIF2|DMA_FLAG_TEIF2|DMA_FLAG_HTIF2);
	}
	else if((Streamx == DMA1_Stream3) || (Streamx == DMA2_Stream3))
	{
		DMA_ClearFlag(Streamx,DMA_FLAG_TCIF3|DMA_FLAG_FEIF3|DMA_FLAG_DMEIF3|DMA_FLAG_TEIF3|DMA_FLAG_HTIF3);
	}
	else if((Streamx == DMA1_Stream4) || (Streamx == DMA2_Stream4))
	{
		DMA_ClearFlag(Streamx,DMA_FLAG_TCIF4|DMA_FLAG_FEIF4|DMA_FLAG_DMEIF4|DMA_FLAG_TEIF4|DMA_FLAG_HTIF4);
	}
	else if((Streamx == DMA1_Stream5) || (Streamx == DMA2_Stream5))
	{
		DMA_ClearFlag(Streamx,DMA_FLAG_TCIF5|DMA_FLAG_FEIF5|DMA_FLAG_DMEIF5|DMA_FLAG_TEIF5|DMA_FLAG_HTIF5);
	}
	else if((Streamx == DMA1_Stream6) || (Streamx == DMA2_Stream6))
	{
		DMA_ClearFlag(Streamx,DMA_FLAG_TCIF6|DMA_FLAG_FEIF6|DMA_FLAG_DMEIF6|DMA_FLAG_TEIF6|DMA_FLAG_HTIF6);
	}
	else if((Streamx == DMA1_Stream7) || (Streamx == DMA2_Stream7))
	{
		DMA_ClearFlag(Streamx,DMA_FLAG_TCIF7|DMA_FLAG_FEIF7|DMA_FLAG_DMEIF7|DMA_FLAG_TEIF7|DMA_FLAG_HTIF7);
	}
}
static void i2c_InitDMA(DMA_Stream_TypeDef * Streamx,uint8_t *Buff,uint32_t BuffLength, uint8_t Channel,uint32_t DMABaseAddress, uint8_t direction)
{
	DMA_InitTypeDef  DMA_InitStructure;
	/*============= DMA Configuration========================= */

	if((Streamx == DMA1_Stream0)||(Streamx == DMA1_Stream1)||(Streamx == DMA1_Stream2)||(Streamx == DMA1_Stream3)||(Streamx == DMA1_Stream4)||(Streamx == DMA1_Stream5)||(Streamx == DMA1_Stream6)||(Streamx == DMA1_Stream7))
	{
		/* Enable the DMA clock */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	}
	else if((Streamx == DMA2_Stream0)||(Streamx == DMA2_Stream1)||(Streamx == DMA2_Stream2)||(Streamx == DMA2_Stream3)||(Streamx == DMA2_Stream4)||(Streamx == DMA2_Stream5)||(Streamx == DMA2_Stream6)||(Streamx == DMA2_Stream7))
	{
		/* Enable the DMA clock */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	}

	if(direction == I2C_Direction_Transmitter)
	{
		/* Clear any pending flag on Tx Stream  */
		i2cDMA_ClearFlags(Streamx);
		/* Disable the I2C Tx DMA stream */
		DMA_Cmd(Streamx, DISABLE);
		/* Configure the DMA stream for the I2C peripheral TX direction */
		DMA_DeInit(Streamx);
	}
	else if(direction == I2C_Direction_Receiver)
	{
		/* Clear any pending flag on Rx Stream  */
		i2cDMA_ClearFlags(Streamx);
		/* Disable the I2C Rx DMA stream */
		DMA_Cmd(Streamx, DISABLE);
		/* Configure the DMA stream for the I2C peripheral RX direction */
		DMA_DeInit(Streamx);
	}

	/* Initialize the DMA_Channel member */
	DMA_InitStructure.DMA_Channel = dmaChanTable[Channel];

	/* Initialize the DMA_PeripheralBaseAddr member */
	DMA_InitStructure.DMA_PeripheralBaseAddr = DMABaseAddress;

	/* Initialize the DMA_PeripheralInc member */
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;

	/* Initialize the DMA_MemoryInc member */
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;

	/* Initialize the DMA_PeripheralDataSize member */
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

	/* Initialize the DMA_MemoryDataSize member */
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

	/* Initialize the DMA_Mode member */
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;

	/* Initialize the DMA_Priority member */
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;

	/* Initialize the DMA_FIFOMode member */
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;

	/* Initialize the DMA_FIFOThreshold member */
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;

	/* Initialize the DMA_MemoryBurst member */
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;

	/* Initialize the DMA_PeripheralBurst member */
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	if(direction == I2C_Direction_Transmitter)
	{
		/* Init DMA for Transmission */
		/* Initialize the DMA_DIR member */
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
		/* Initialize the DMA_Memory0BaseAddr member */
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Buff;
		/* Initialize the DMA_BufferSize member */
		DMA_InitStructure.DMA_BufferSize = BuffLength;
		DMA_DeInit(Streamx);
		DMA_Init(Streamx, &DMA_InitStructure);
	}
	else if(direction == I2C_Direction_Receiver)
	{
		/* Init DMA for Reception */
		/* Initialize the DMA_DIR member */
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		if(Buff != NULL)
		{
			/* Initialize the DMA_Memory0BaseAddr member */
			DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Buff;
			/* Initialize the DMA_BufferSize member */
			DMA_InitStructure.DMA_BufferSize = BuffLength;
		}
		else
		{
			/* Initialize the DMA_Memory0BaseAddr member */
			DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)DummyBuff;
			/* Initialize the DMA_BufferSize member */
			DMA_InitStructure.DMA_BufferSize = 1;
		}
		DMA_DeInit(Streamx);
		DMA_Init(Streamx, &DMA_InitStructure);
	}
}

int i2cLocalDmaTransfer(void *handle)
{
    //uint8_t *pTransBuffer/*   = NULL*/;
    //uint8_t *pCurrBuffer/*   = NULL*/;
    i2c_Object           *instHandle/*   = NULL*/;
    I2C_ChanObj          *chanHandle/*   = NULL*/;
    int                 retVal       = IODEV_COMPLETED;
    uint16_t                flag;
    i2c_DataParam        *dataParam/*   = NULL*/;
	DMA_Stream_TypeDef	*hDmaTxStream/*   = NULL*/;
	DMA_Stream_TypeDef	*hDmaRxStream/*   = NULL*/;
	//DMA_InitTypeDef dmaTxInit;
	//DMA_InitTypeDef dmaRxInit;
	//struct sDmaResSet *hDmaResSet=NULL;
	//DMA_RESOURCE *hDmaRes=NULL;
	struct sDmaParam *hDmaTxParam/*   = NULL*/;
	struct sDmaParam *hDmaRxParam/*   = NULL*/;
	//uint32_t key;
	I2C_TypeDef		*i2cRegs;

    assert(NULL != handle);

    instHandle = (i2c_Object *)handle ;
    i2cRegs=instHandle->deviceInfo.baseAddress;
    chanHandle = instHandle->currentActiveChannel;
    dataParam  = (i2c_DataParam *)chanHandle->activeIOP->addr;
    flag = dataParam->flags;

    i2cLocalControlDataConfig(chanHandle);
	
    if(flag == I2C_WRITE)
    {
    	hDmaTxParam=&((struct sDmaResSet *)instHandle->hDma)->set[TX_DMA];
    	hDmaTxStream=dmaStreamInfo.dmac[hDmaTxParam->dmacId].pStream[hDmaTxParam->stream];
    	i2c_InitDMA(hDmaTxStream,dataParam->outBuffer,dataParam->outBufLen, hDmaTxParam->channel,(uint32_t)&instHandle->deviceInfo.baseAddress->DR,I2C_Direction_Transmitter);


    	DMA_ITConfig(hDmaTxStream,DMA_IT_TC,ENABLE);
    	i2c_start(i2cRegs,dataParam->deviceAddress , I2C_Direction_Transmitter);

    	/* I2Cx DMA Enable */
    	I2C_DMACmd(i2cRegs, ENABLE);
    	DMA_Cmd(hDmaTxStream,ENABLE);

    	//i2c_DMAWrite(i2cRegs,hDmaTxStream,dataParam->outBuffer,dataParam->outBufLen);

    	//i2c_stop(i2cRegs,I2C_Direction_Transmitter); // stop the transmission
    }
    else
    {
    	hDmaRxParam =& ((struct sDmaResSet *)instHandle->hDma)->set[RX_DMA];
    	hDmaRxStream = dmaStreamInfo.dmac[hDmaRxParam->dmacId].pStream[hDmaRxParam->stream];
    	i2c_InitDMA(hDmaRxStream,dataParam->inBuffer,dataParam->inBufLen, hDmaRxParam->channel,(uint32_t)&instHandle->deviceInfo.baseAddress->DR,I2C_Direction_Receiver);

    	DMA_ITConfig(hDmaRxStream,DMA_IT_TC,ENABLE);
    	i2c_start(i2cRegs,dataParam->deviceAddress , I2C_Direction_Receiver);
    	I2C_DMACmd(i2cRegs, ENABLE);
		I2C_AcknowledgeConfig(i2cRegs, ENABLE);  
		/* Inform the DMA that the next End Of Transfer Signal will be the last one */
    	I2C_DMALastTransferCmd(i2cRegs, ENABLE); 
    	DMA_Cmd(hDmaRxStream,ENABLE);

    	//i2c_DMARead(i2cRegs,hDmaRxStream,dataParam->inBuffer,dataParam->inBufLen);
		//i2c_stop(i2cRegs,I2C_Direction_Receiver); // stop the reception
    }


    //instHandle->currentActiveChannel = NULL;
	return (retVal);
}

int i2cLocalDmaChannel_Request(i2c_Object *instHandle)
{
	uint32_t key;
	DMA_RESOURCE *hDmaRes=NULL;

	struct sDmaParam *hDmaTxParam=NULL;
	struct sDmaParam *hDmaRxParam=NULL;
	NVIC_InitTypeDef nvicInit;

    assert(NULL != instHandle);

	if (instHandle->hDma==NULL)
	{
	}
	else
	{
		hDmaTxParam=&((struct sDmaResSet *)instHandle->hDma)->set[TX_DMA];
		hDmaRxParam=&((struct sDmaResSet *)instHandle->hDma)->set[RX_DMA];
		if (instHandle->hDmaRes==NULL)
		{
			/* Obtain DMA resource */
			hDmaRes=allocDmaResource(hDmaRxParam->dmacId,hDmaRxParam->stream,hDmaRxParam->channel,portMAX_DELAY,NULL);
			key=__disableInterrupts();
			instHandle->hDmaRes=hDmaRes;
			__restoreInterrupts(key);
		}
		else
			hDmaRes=(DMA_RESOURCE *)instHandle->hDmaRes;
		if (hDmaRes==NULL)
			return IODEV_EBADIO;

		/* install Rx DMA interrupt handler */
		nvicInit.NVIC_IRQChannel = dmaStreamInfo.dmac[hDmaRxParam->dmacId].nIrq[hDmaRxParam->stream];
		nvicInit.NVIC_IRQChannelPreemptionPriority = 15;
		nvicInit.NVIC_IRQChannelSubPriority = 1;
		nvicInit.NVIC_IRQChannelCmd = ENABLE;
		installInterruptHandler((uint16_t)nvicInit.NVIC_IRQChannel,sti2cDmaRxIsr,instHandle);
		instHandle->hRxDmaHandler=sti2cDmaRxIsr;
		instHandle->hRxDmaHandlerArg=instHandle;
		NVIC_Init(&nvicInit);

		/* install Tx DMA interrupt handler */
		nvicInit.NVIC_IRQChannel = dmaStreamInfo.dmac[hDmaTxParam->dmacId].nIrq[hDmaTxParam->stream];
		nvicInit.NVIC_IRQChannelPreemptionPriority = 15;
		nvicInit.NVIC_IRQChannelSubPriority = 1;
		nvicInit.NVIC_IRQChannelCmd = ENABLE;
		installInterruptHandler((uint16_t)nvicInit.NVIC_IRQChannel,sti2cDmaTxIsr,instHandle);
		instHandle->hTxDmaHandler=sti2cDmaTxIsr;
		instHandle->hTxDmaHandlerArg=instHandle;
		NVIC_Init(&nvicInit);
	}
	return (IODEV_COMPLETED);
}

#define TX_DMA_XFER_COMPLETE	0x04
#define RX_DMA_XFER_COMPLETE	0x08

int sti2cDmaTxIsr(i2c_Object *instHandle)
{
	I2C_ChanObj *chanHandle/*  = NULL*/;
    IODEV_Packet *ioPacket/*   = NULL*/;
	uint16_t bytesRemain=0;
	DMA_Stream_TypeDef *pstream/*=NULL*/;
	uint16_t stream=0;
	int completeIo=0;

    assert(NULL != instHandle);

    chanHandle = instHandle->currentActiveChannel;

	stream=((struct sDmaResSet *)instHandle->hDma)->set[TX_DMA].stream;
	pstream=dmaStreamInfo.dmac[((struct sDmaResSet *)instHandle->hDma)->set[TX_DMA].dmacId].pStream[stream];

	if (DMA_GetFlagStatus(pstream, dmaTCflagid[stream])!=RESET)
	{
		DMA_ClearITPendingBit(pstream,dmaTCflagid[stream]);
		DMA_ITConfig(pstream,DMA_IT_TC,DISABLE);

		/* Disable the TX DMA. If only   *
		 * there is a packet for processing it shall be re-enabled*/
		I2C_DMACmd(instHandle->deviceInfo.baseAddress,DISABLE);
		ioPacket = chanHandle->activeIOP;
		if (ioPacket!=NULL)
		{
			//chanHandle->activeIOP->status|=TX_DMA_XFER_COMPLETE;
			bytesRemain=DMA_GetCurrDataCounter(pstream);
			instHandle->stats.txBytes += ioPacket->size-bytesRemain;
			//if (chanHandle->activeIOP->status & RX_DMA_XFER_COMPLETE)
			{
				i2c_stop(instHandle->deviceInfo.baseAddress,I2C_Direction_Transmitter); // stop the transmission
				completeIo|=i2cCompleteIOdmaCallback(instHandle);
			}
		}
	}
	return completeIo;
}

int sti2cDmaRxIsr(i2c_Object *instHandle)
{
	#ifdef USE_SPI_DRIVER_TIMESTAMPS
	TIMESTAMP ts=readTimestamp();
	TIMESTAMP *tsp;
	#endif
	I2C_ChanObj *chanHandle/*  = NULL*/;
    IODEV_Packet *ioPacket/*   = NULL*/;
	uint16_t bytesRemain=0;
	uint16_t stream/*=0*/;
	DMA_Stream_TypeDef *pstream/*=NULL*/;
	int completeIo=0;

    assert(NULL != instHandle);

    chanHandle = instHandle->currentActiveChannel;

	stream=((struct sDmaResSet *)instHandle->hDma)->set[RX_DMA].stream;
	pstream=dmaStreamInfo.dmac[((struct sDmaResSet *)instHandle->hDma)->set[RX_DMA].dmacId].pStream[stream];

	if (DMA_GetFlagStatus(pstream, dmaTCflagid[stream])!=RESET)
	{
		DMA_ClearITPendingBit(pstream,dmaTCflagid[stream]);
		DMA_ITConfig(pstream,DMA_IT_TC,DISABLE);

		/* Disable the RX DMA before completing. If only   *
		 * there is a packet for processing it shall be re-enabled*/
		I2C_DMACmd(instHandle->deviceInfo.baseAddress,DISABLE);

		ioPacket = chanHandle->activeIOP;
		if (ioPacket!=NULL)
		{
			ioPacket->status|=RX_DMA_XFER_COMPLETE;
			bytesRemain=DMA_GetCurrDataCounter(pstream);
			instHandle->stats.rxBytes += ioPacket->size-bytesRemain;
			//if (chanHandle->activeIOP->status & TX_DMA_XFER_COMPLETE)
			{
				i2c_stop(instHandle->deviceInfo.baseAddress,I2C_Direction_Receiver); // stop the reception
				ioPacket->status=IODEV_COMPLETED;
				completeIo |= i2cCompleteIOdmaCallback(instHandle);
			}
		}
	}
	return completeIo;
}

/**
 *  \brief Function used after dma callback
 *
 *  This function will be called after the completion of dma callback. This
 *  function requests the next channel and also calls the app callback function.
 *
 *  \param   instHandle  [IN]   Pointer to the spi driver instance object
 *
 *  \return  None
 *
 *  \enter  instHandle must be a valid pointer and should not be null.
 *
 *  \leave  Not Implemented.
 */
int i2cCompleteIOdmaCallback (i2c_Object *instHandle)
{
    register I2C_ChanObj *chanHandle/* = NULL*/;
    register i2c_DataParam *dataParam/*  = NULL*/;
    int transFlags = 0;
    register IODEV_Packet *ioPacket/*   = NULL*/;
	register I2C_TypeDef *i2cRegs;
	int completeIo=0;
	//register i2c_HWConfigData *hwc;

    assert(NULL != instHandle);

    chanHandle = instHandle->currentActiveChannel;
    dataParam  = (i2c_DataParam *)chanHandle->activeIOP->addr;
    i2cRegs=instHandle->deviceInfo.baseAddress;
    //hwc=&instHandle->i2cHWconfig;

    /* Mask off interrupts                                                    */
    I2C_ITConfig(i2cRegs,I2C_IT_BUF|I2C_IT_EVT|I2C_IT_ERR,DISABLE);
	//I2C_ITConfig(i2cRegs,I2C_IT_ERR,DISABLE);

    /* call the application completion callback function registered           *
     * with us during opening of the channel                                  */
    if (NULL != chanHandle->cbFxn)
    {
        /* Invoke Application callback for this channel                       */
        completeIo |= (*chanHandle->cbFxn)((void *)chanHandle->cbArg, chanHandle->activeIOP);
    }

    chanHandle->activeIOP = NULL;

    /*get the channel with highest priority                                   */
    i2cLocalGetNextChannel(instHandle, &(instHandle->currentActiveChannel));

    /*store for local use                                                     */
    chanHandle = instHandle->currentActiveChannel;

    /* check and load next pending packet                                     */
    if (NULL != chanHandle)
    {
        /* we have atleast one packet                                         */
        ioPacket = (IODEV_Packet *) QUE_get(&(chanHandle->queuePendingList));
        instHandle->stats.pendingPacket--;

        /* validate and update the iop                                        */
        if (NULL  != ioPacket)
        {
            chanHandle->activeIOP = ioPacket;
            dataParam = (i2c_DataParam *)ioPacket->addr;

            if (NULL == dataParam->inBuffer)
                transFlags = 0;
            else
                transFlags = 1;

            /* Mask all interrupts,& populate data to spi object structure    *
             * so driver can extact all information at dma configuartion      *
             * and dma Rx & Tx callback                                       */

            /* Transmit data                                                  */
			while (I2C_GetFlagStatus(i2cRegs,I2C_FLAG_RXNE))
			{
				/* Read before Write to remove any stale data			  */
				*chanHandle->transBuffer = (uint8_t)(I2C_ReceiveData(i2cRegs));
			}

            chanHandle->currError           = 0;
            chanHandle->currFlags           = dataParam->flags;
            chanHandle->transcieveFlags     = transFlags;
            chanHandle->currBuffer          = dataParam->outBuffer;
            chanHandle->currBufferLen       = dataParam->outBufLen;

            /* Call this to complete edma configuration and transfer          */
            //i2cLocalDmaTransfer(instHandle,dataParam->chipSelect,
            //        dataParam->dataFormat,dataParam->flags);
        }
    }
	return completeIo;
}
#endif
/* This function issues a start condition and
 * transmits the slave address + R/W bit
 * Poll mode
 * Parameters:
 * 		I2Cx --> the I2C peripheral e.g. I2C1
 * 		address --> the 7 bit slave address
 * 		direction --> the transmission direction can be:
 * 						I2C_Direction_Tranmitter for Master transmitter mode
 * 						I2C_Direction_Receiver for Master receiver
 */
void i2c_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction)
{
	// wait until I2C2 is not busy any more
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

	// Send I2C2 START condition
	I2C_GenerateSTART(I2Cx, ENABLE);

	// wait for I2C2 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

	// Send slave Address for write
	I2C_Send7bitAddress(I2Cx, address, direction);

	/* wait for I2Cx EV6, check if
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */
	if(direction == I2C_Direction_Transmitter)
	{
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	}
	else if(direction == I2C_Direction_Receiver)
	{
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}
}

/* This function transmits one byte to the slave device
 * Poll mode
 * Parameters:
 *		I2Cx --> the I2C peripheral e.g. I2C1
 *		data --> the data byte to be transmitted
 */
static void i2c_write(I2C_TypeDef* I2Cx, uint8_t data)
{
	// wait for I2C1 EV8 --> last byte is still being transmitted (last byte in SR, buffer empty), next byte can already be written
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
	I2C_SendData(I2Cx, data);
}

/* This function reads one byte from the slave device
 * Poll mode
 * and acknowledges the byte (requests another byte)
 */
static uint8_t i2c_read_ack(I2C_TypeDef* I2Cx)
{
	// enable acknowledge of received data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

/* This function reads one byte from the slave device
 * and doesn't acknowledge the received data
 * after that a STOP condition is transmitted
 * Poll mode
 */
static uint8_t i2c_read_nack(I2C_TypeDef* I2Cx)
{
	// disable acknowledge of received data
	// nack also generates stop condition after last byte received
	// see reference manual for more info
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	I2C_GenerateSTOP(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

/* This function issues a stop condition and therefore
 * releases the bus
 * Poll mode
 */
static void i2c_stop(I2C_TypeDef* I2Cx, uint8_t direction)
{
	if(direction == I2C_Direction_Transmitter)
	{
		// wait for I2Cx EV8_2 --> byte has been transmitted
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	}
	else if(direction == I2C_Direction_Receiver)
	{
		//while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	}
	// Send I2Cx STOP Condition after last byte has been transmitted or received
	I2C_GenerateSTOP(I2Cx, ENABLE);

	//I2C_GenerateSTOP(I2Cx, DISABLE);
}


/**
 *  \brief  Interrupt handler for I2C Device
 *
 *         It will check the following errors like bit error, desync error
 *         parity error, overrun error. Data transfer either read or write
 *         is done on RX INT flag only.
 *
 *  \param  instHandle  [IN]    Pointer to the spi driver object
 *
 *  \return None
 */


/// Interupt demo function
///**
//  * @brief  This function handles I2Cx event interrupt request.
//  * @param  pointer to instHandle
//  * @retval None
//  */
int i2cIntrHandler(i2c_Object *instHandle)
{
    register I2C_ChanObj *chanHandle;
	register I2C_TypeDef *i2cRegs;
	int completeIo=0;
	uint32_t Event = 0x00;
	i2c_DataParam *dataParam;
    assert(NULL != instHandle);

    i2cRegs=instHandle->deviceInfo.baseAddress;
    chanHandle = instHandle->currentActiveChannel;
    dataParam = &chanHandle->dataParam;

	/* Once the Start condition is sent the master can be master receiver
  	  or master transmitter */
    if(dataParam->flags == I2C_WRITE)
	{
		/* Get Last I2C Event */
		Event = I2C_GetLastEvent(i2cRegs);
		switch (Event)
		{
			/* ************************************************************************/
			/*                        Master Transmitter Events                       */
			/*                                                                        */
			/* ************************************************************************/
			/* Sending the header sequence for Master Transmitter case ----------------*/

			/* I2C_7BITS_ADDRESS */
			/* Check on EV5 */
			case I2C_EVENT_MASTER_MODE_SELECT :
				/* Send slave Address for write */
				I2C_Send7bitAddress(i2cRegs, (uint8_t)dataParam->deviceAddress, I2C_Direction_Transmitter);
				break;

			/* Check on EV6 */
			case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
				/* Transmit the First Data  */
				I2C_SendData(i2cRegs,(*dataParam->outBuffer));
				dataParam->outBuffer++;
				dataParam->outBufLen--;
				instHandle->stats.txBytes++;
				break;

			/* Check on EV8 */
			case I2C_EVENT_MASTER_BYTE_TRANSMITTING:
			case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
			case 0x70480:
				if(dataParam->outBufLen == 0)
				{
					/* Send STOP condition */
					I2C_GenerateSTOP(i2cRegs, ENABLE);
					I2C_ITConfig(i2cRegs, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
					completeIo |= i2cCompleteIOInIsr(instHandle);
				}
				else
				{
					/* Transmit Data TxBuffer */
					I2C_SendData(i2cRegs,(*dataParam->outBuffer));
					dataParam->outBuffer++;
					dataParam->outBufLen--;
					instHandle->stats.txBytes++;
				}
				break;

			default:
				break;
		}
	}
	/*************************************************************************/
	/*                        Master Receiver Events                         */
	/*                                                                       */
	/*************************************************************************/
	else /* MASTER_MODE_RECEIVER */
	{
		/* Check on EV5 */
		if(I2C_GetITStatus(i2cRegs, I2C_IT_SB)== SET)
		{
			/* Send slave Address for read */
			I2C_Send7bitAddress(i2cRegs, dataParam->deviceAddress, I2C_Direction_Receiver);
			I2C_AcknowledgeConfig(i2cRegs, ENABLE);
		}
		else if(I2C_GetITStatus(i2cRegs, I2C_IT_ADDR)== SET)
		{
			if (dataParam->inBufLen== 1)
			{
				I2C_AcknowledgeConfig(i2cRegs, DISABLE);
			}
			/* Clear ADDR Register */
			(void)(i2cRegs->SR1);
			(void)(i2cRegs->SR2);
		}
		else if((I2C_GetITStatus(i2cRegs, I2C_IT_RXNE)== SET)&&(I2C_GetITStatus(i2cRegs, I2C_IT_BTF)== RESET))
		{
			/* Store I2C received data */
			if(dataParam->inBuffer != NULL)
				*dataParam->inBuffer = I2C_ReceiveData(i2cRegs);
			dataParam->inBufLen--;
			dataParam->inBuffer++;
			instHandle->stats.rxBytes++;

			if (dataParam->inBufLen == 0x00)
			{
				/* Disable Error and Buffer Interrupts */
				I2C_GenerateSTOP(i2cRegs, ENABLE);
				I2C_ITConfig(i2cRegs, (I2C_IT_EVT | I2C_IT_BUF), DISABLE);
				completeIo |= i2cCompleteIOInIsr(instHandle);
			}
		}
		/* BUSY, MSL and RXNE flags */
		else if(I2C_GetITStatus(i2cRegs, I2C_IT_BTF)== SET)
		{
			/* Store I2C received data */
			if(dataParam->inBufLen>0)
			{
				if(dataParam->inBuffer != NULL)
					*dataParam->inBuffer = I2C_ReceiveData(i2cRegs);
				dataParam->inBufLen--;
				dataParam->inBuffer++;
				instHandle->stats.rxBytes++;
			}
			else
			{
				I2C_GenerateSTOP(i2cRegs, ENABLE);
				I2C_ITConfig(i2cRegs, (I2C_IT_EVT | I2C_IT_BUF), DISABLE);
				completeIo |= i2cCompleteIOInIsr(instHandle);
			}
		}
	}
    return completeIo;
}
///**
//  * @brief  This function handles I2Cx Error interrupt request.
//  * @param  None
//  * @retval None
//  */
//void I2Cx_ER_IRQHANDLER(void)
//{
//	/* Read SR1 register to get I2C error */
//	if ((I2C_ReadRegister(I2Cx, I2C_Register_SR1) & 0xFF00) != 0x00)
//	{
//		/* Clears error flags */
//		I2Cx->SR1 &= 0x00FF;
//	}
//	/*handle Error TBD */
//}
//


