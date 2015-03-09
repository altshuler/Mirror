/**
* @file st_spi.c
* @brief spi IO driver
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 28.06.2012
*/

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <iodev.h>
#include <sysport.h>
#include <freertos.h>
#include <task.h>
#include <spi.h>
#include "st_spi.h"
#include <board.h>
#include <irqhndl.h>
#ifdef USE_SPI_DRIVER_TIMESTAMPS
#include <timebase.h>
#endif
#include <dma.h>
#include "../st_dma/st_dma.h"

#if ((defined(SPI1_MISO_PIN) && defined(SPI1_MISO_GPIO_PORT) && defined(SPI1_MISO_GPIO_CLK)  && defined(SPI1_MISO_PIN_SOURCE)) && (defined(SPI1_MOSI_PIN) && defined(SPI1_MOSI_GPIO_PORT) && defined(SPI1_MOSI_GPIO_CLK)  && defined(SPI1_MOSI_PIN_SOURCE)) && (defined(SPI1_NSS_PIN) && defined(SPI1_NSS_GPIO_PORT) && defined(SPI1_CLK_GPIO_CLK)  && defined(SPI1_CLK_PIN_SOURCE)) && (defined(SPI1_CLK_PIN) && defined(SPI1_NSS_GPIO_PORT) && defined(SPI1_NSS_GPIO_CLK)  && defined(SPI1_NSS_PIN_SOURCE)))
	#define USE_SPI1_GPIO
#endif

#if ((defined(SPI2_MISO_PIN) && defined(SPI2_MISO_GPIO_PORT) && defined(SPI2_MISO_GPIO_CLK)  && defined(SPI2_MISO_PIN_SOURCE)) && (defined(SPI2_MOSI_PIN) && defined(SPI2_MOSI_GPIO_PORT) && defined(SPI2_MOSI_GPIO_CLK)  && defined(SPI2_MOSI_PIN_SOURCE)) && (defined(SPI1_NSS_PIN) && defined(SPI1_NSS_GPIO_PORT) && defined(SPI1_CLK_GPIO_CLK)  && defined(SPI1_CLK_PIN_SOURCE)) && (defined(SPI1_CLK_PIN) && defined(SPI1_NSS_GPIO_PORT) && defined(SPI1_NSS_GPIO_CLK)  && defined(SPI1_NSS_PIN_SOURCE)))
	#define USE_SPI2_GPIO
#endif

#if ((defined(SPI3_MISO_PIN) && defined(SPI3_MISO_GPIO_PORT) && defined(SPI3_MISO_GPIO_CLK)  && defined(SPI3_MISO_PIN_SOURCE)) && (defined(SPI3_MOSI_PIN) && defined(SPI3_MOSI_GPIO_PORT) && defined(SPI3_MOSI_GPIO_CLK)  && defined(SPI3_MOSI_PIN_SOURCE)) && (defined(SPI1_NSS_PIN) && defined(SPI1_NSS_GPIO_PORT) && defined(SPI1_CLK_GPIO_CLK)  && defined(SPI1_CLK_PIN_SOURCE)) && (defined(SPI1_CLK_PIN) && defined(SPI1_NSS_GPIO_PORT) && defined(SPI1_NSS_GPIO_CLK)  && defined(SPI1_NSS_PIN_SOURCE)))
	#define USE_SPI3_GPIO
#endif



#define SPI_MODULE_CLOCK	((uint32_t)160000000)

Spi_ConfigDataFmt	configDatafmt[4];	/**< Data Format Configuration values									  */



const Spi_Params Spi_PARAMS = 
#ifdef KUKU
{
    Spi_OpMode_POLLED,              /* opMode */
	0,								/* loopbackEnabled */
	(uint16_t)0x0,					  /* HWINumber */
    (uint32_t)3000000,               /* outputClkFreq */
    portMAX_DELAY,                    /* timeout */
    {
        Spi_CommMode_MASTER,        /* masterOrSlave */
        1,                          /* clkInternal */
        0,                          /* enableHighZ */
        Spi_PinOpMode_SPISCS_4PIN,  /* pinOpModes */
        (uint32_t)0x0,                /* waitDelay */
        {
            {
                (uint32_t)0x0,        /* wDelay */
                0x0,        /* charLength */
                0,                  /* lsbFirst */
                0,                  /* phaseIn */
                0,                  /* oddParity */
                0,                  /* parityEnable */
                0,                  /* clkHigh */
                0,                  /* waitEnable */
            }, /* [0] */
            {
                (uint32_t)0x0,        /* wDelay */
                0x0,        /* charLength */
                0,                  /* lsbFirst */
                0,                  /* phaseIn */
                0,                  /* oddParity */
                0,                  /* parityEnable */
                0,                  /* clkHigh */
                0,                  /* waitEnable */
            },  /* [1] */
            {
                (uint32_t)0x0,        /* wDelay */
                0x0,        /* charLength */
                0,                  /* lsbFirst */
                0,                  /* phaseIn */
                0,                  /* oddParity */
                0,                  /* parityEnable */
                0,                  /* clkHigh */
                0,                  /* waitEnable */
            },  /* [2] */
            {
                (uint32_t)0x0,        /* wDelay */
                0x0,        /* charLength */
                0,                  /* lsbFirst */
                0,                  /* phaseIn */
                0,                  /* oddParity */
                0,                  /* parityEnable */
                0,                  /* clkHigh */
                0,                  /* waitEnable */
            },  /* [3] */
        },                          /* configDatafmt */
		0xff,				/* csDefault */
		0,                      /* gpioChipselectFlag */
		0,							/* gpioPinNo */
		0,							/* gpioPinClock */
		NULL,							/* gpioPinPort */
		//0,						  /* intrLevel */
		#ifdef EXTENDED_CHIP_SELECT
		NULL,						/* extGpiochipSelect */
		#endif
    },                              /* spiHWCfgData */
    (((void*)0x0)),            /* hDma */
			//TRUE, 						  /*enablecache */
};
#else
{
    Spi_OpMode_POLLED,              /* opMode */
	0,								/* loopbackEnabled */
	(uint16_t)0x0,					  /* HWINumber */
    (uint32_t)3000000,               /* outputClkFreq */
    portMAX_DELAY,                    /* timeout */
    {
        Spi_CommMode_MASTER,        /* masterOrSlave */
        1,                          /* clkInternal */
        0,                          /* enableHighZ */
        Spi_PinOpMode_SPISCS_4PIN,  /* pinOpModes */
		0,						/* gpioChipselectFlag */
		0,
		0xff,				/* csDefault */
		(uint32_t)0x0,				  /* waitDelay */
		0,							/* gpioPinNo */
		0,							/* gpioPinClock */
		NULL,							/* gpioPinPort */
        {
            {
                (uint32_t)0x0,        /* wDelay */
                0x0,        /* charLength */
                0,                  /* lsbFirst */
                0,                  /* phaseIn */
                0,                  /* oddParity */
                0,                  /* parityEnable */
                0,                  /* clkHigh */
                0                   /* waitEnable */
            }, /* [0] */
            {
                (uint32_t)0x0,        /* wDelay */
                0x0,        /* charLength */
                0,                  /* lsbFirst */
                0,                  /* phaseIn */
                0,                  /* oddParity */
                0,                  /* parityEnable */
                0,                  /* clkHigh */
                0                   /* waitEnable */
            },  /* [1] */
            {
                (uint32_t)0x0,        /* wDelay */
                0x0,        /* charLength */
                0,                  /* lsbFirst */
                0,                  /* phaseIn */
                0,                  /* oddParity */
                0,                  /* parityEnable */
                0,                  /* clkHigh */
                0                   /* waitEnable */
            },  /* [2] */
            {
                (uint32_t)0x0,        /* wDelay */
                0x0,        /* charLength */
                0,                  /* lsbFirst */
                0,                  /* phaseIn */
                0,                  /* oddParity */
                0,                  /* parityEnable */
                0,                  /* clkHigh */
                0                   /* waitEnable */
            },  /* [3] */
        },                          /* configDatafmt */
		#ifdef EXTENDED_CHIP_SELECT
		NULL,						/* extGpiochipSelect */
		#endif
    },                              /* spiHWCfgData */
    (((void*)0x0)),            /* hDma */
			//TRUE, 						  /*enablecache */
};

#endif

/* ========================================================================== */
/*                       GLOBAL MODULE STATE                                  */
/* ========================================================================== */
/**
 *  \brief  Array which is part of Spi Module State
 */   
static int inUse[N_SPI];
/**
 *  \brief  Spi Module State Object
 */    
static Spi_Module_State Spi_module = {&inUse[0]};
/**
 *  \brief  Array of Spi instance State objects array
 */
static Spi_Object Spi_Instances[N_SPI];
/**
 *  \brief Global variable used as local buffer for transcieve operation.
 *  
 *  It will return data to application in transceieve call and it will dump the 
 *  data for other calls.
 */
uint8_t Spi_transReceive[Spi_BUFFER_DATA_SIZE];

Spi_HwInfo Spi_deviceInstInfo[N_SPI];

/* ========================================================================== */
/*                        LOCAL FUNCTION PROTOTYPES                           */
/* ========================================================================== */
static int stSpiBindDev(void **devp, int devId, void *devParams);
static int stSpiUnBindDev(void *devp);
static int stSpiCreateChan(void **chanp, void *devp, char *name, int mode, void *chanParams, IODEV_TiomCallback cbFxn, void *cbArg);
static int  stSpiDeleteChan(void *chanp);
static int  stSpiSubmitChan(void *chanp, IODEV_Packet *packet);
static int  stSpiControlChan(void *chanp, unsigned int cmd, void *arg);


const IODEV_Fxns Spi_IODEVFXNS =
{
    &stSpiBindDev,
    &stSpiUnBindDev,
    &stSpiControlChan,
    &stSpiCreateChan,
    &stSpiDeleteChan,
    &stSpiSubmitChan
};

static int spiTransfer(Spi_Object *instHandle, Spi_ChanObj *chanHandle, Spi_DataParam *dataparam, uint8_t *inBuffer, int transflags, int timeout, void *xferActual);
static int spiIoctl(void *handle, Spi_ioctlCmd cmd, void *cmdArg, void *param);
//static void spiRegisterIntrHandler(uint32_t instNum, uint32_t intNum, Spi_isr initIsr, void *spiObj);
static int spiSetupConfig(Spi_ChanObj *chanHandle);
static int spiSetupClock(Spi_ChanObj *chanHandle);
static void spiConfigureOpMode(const Spi_Object *instHandle);
int spiIntrHandler(Spi_Object *instHandle);
//static void spiUnregisterIntrHandler(uint32_t instNum, uint32_t intNum);
static void spiWaitForStaleData(void);
static int spiCheckTimeOut(portTickType startValue,portTickType timeout);
static int spiCompleteIOInIsr (Spi_Object *instHandle);
static int spiPolledModeTransfer(Spi_ChanObj *chanHandle);
static uint16_t spiPrescaler(uint32_t inFreq, uint32_t busFreq);

int stSpiDmaTxIsr(Spi_Object *instHandle);
int stSpiDmaRxIsr(Spi_Object *instHandle);
#ifdef Spi_DMA_ENABLE
int spiCompleteIOdmaCallback (Spi_Object *instHandle);
#endif

/* ========================================================================== */
/*                        LOCAL Constants                          */
/* ========================================================================== */

#ifdef USE_SPI1_GPIO
const GPIO_InitTypeDef gpioSpi1NssOutInit= {\
	SPI1_NSS_PIN, /* GPIO_Pin */ \
	GPIO_Mode_OUT, /* GPIO_Mode */ \
	GPIO_Speed_100MHz, /* GPIO_Speed */ \
	GPIO_OType_PP, /* GPIO_OType */ \
	GPIO_PuPd_NOPULL /* GPIO_PuPd_NOPULL */ \
};
#endif
						
#ifdef USE_SPI2_GPIO
const GPIO_InitTypeDef gpioSpi2NssOutInit= {\
	SPI2_NSS_PIN, /* GPIO_Pin */ \
	GPIO_Mode_OUT, /* GPIO_Mode */ \
	GPIO_Speed_100MHz, /* GPIO_Speed */ \
	GPIO_OType_PP, /* GPIO_OType */ \
	GPIO_PuPd_NOPULL /* GPIO_PuPd_NOPULL */ \
};
#endif
												
#ifdef USE_SPI3_GPIO
const GPIO_InitTypeDef gpioSpi3NssOutInit= {\
	SPI3_NSS_PIN, /* GPIO_Pin */ \
	GPIO_Mode_OUT, /* GPIO_Mode */ \
	GPIO_Speed_100MHz, /* GPIO_Speed */ \
	GPIO_OType_PP, /* GPIO_OType */ \
	GPIO_PuPd_NOPULL /* GPIO_PuPd_NOPULL */ \
};
#endif
																		
#ifdef USE_SPI1_GPIO
const GPIO_InitTypeDef gpioSpi1NssAfInit= {\
	SPI1_NSS_PIN, /* GPIO_Pin */ \
	GPIO_Mode_AF, /* GPIO_Mode */ \
	GPIO_Speed_100MHz, /* GPIO_Speed */ \
	GPIO_OType_PP, /* GPIO_OType */ \
	GPIO_PuPd_NOPULL /* GPIO_PuPd_NOPULL */ \
};
#endif
						
#ifdef USE_SPI2_GPIO
const GPIO_InitTypeDef gpioSpi2NssAfInit= {\
	SPI2_NSS_PIN, /* GPIO_Pin */ \
	GPIO_Mode_AF, /* GPIO_Mode */ \
	GPIO_Speed_100MHz, /* GPIO_Speed */ \
	GPIO_OType_PP, /* GPIO_OType */ \
	GPIO_PuPd_NOPULL /* GPIO_PuPd_NOPULL */ \
};
#endif
												
#ifdef USE_SPI3_GPIO
const GPIO_InitTypeDef gpioSpi3NssAfInit= {\
	SPI3_NSS_PIN, /* GPIO_Pin */ \
	GPIO_Mode_AF, /* GPIO_Mode */ \
	GPIO_Speed_100MHz, /* GPIO_Speed */ \
	GPIO_OType_PP, /* GPIO_OType */ \
	GPIO_PuPd_NOPULL /* GPIO_PuPd_NOPULL */ \
};
#endif
																																				

/* ========================================================================== */
/*                           MODULE FUNCTIONS                                 */
/* ========================================================================== */

/**
 *  \brief  Function called by Bios during instance initialisation
 *
 */
void stSpiInit(void)
{
    int i;
    
    for (i = 0; i < N_SPI; i++)
    {
        // have to initialize statically
        Spi_module.inUse[i] = 0;
        memset((void *)&Spi_Instances[i], 0x0, sizeof(Spi_Object));

        if (i == 0)        
        {
            Spi_deviceInstInfo[i].baseAddress = SPI1;
            Spi_deviceInstInfo[i].cpuEventNumber = SPI1_IRQn;
            Spi_deviceInstInfo[i].rxDmaEventNumber = 0;
            Spi_deviceInstInfo[i].txDmaEventNumber = 0;
            Spi_deviceInstInfo[i].inputFrequency = SPI_MODULE_CLOCK;
            //Spi_deviceInstInfo[i].maxChipSelect = 1;
        }
        else if (i == 1) 
        {
            Spi_deviceInstInfo[i].baseAddress = SPI2;
            Spi_deviceInstInfo[i].cpuEventNumber = SPI2_IRQn;
            Spi_deviceInstInfo[i].rxDmaEventNumber = 0;
            Spi_deviceInstInfo[i].txDmaEventNumber = 0;
            Spi_deviceInstInfo[i].inputFrequency = SPI_MODULE_CLOCK;
            //Spi_deviceInstInfo[i].maxChipSelect = 1;
        }
        else if (i == 2) 
        {
            Spi_deviceInstInfo[i].baseAddress = SPI3;
            Spi_deviceInstInfo[i].cpuEventNumber = SPI3_IRQn;
            Spi_deviceInstInfo[i].rxDmaEventNumber = 0;
            Spi_deviceInstInfo[i].txDmaEventNumber = 0;
            Spi_deviceInstInfo[i].inputFrequency = SPI_MODULE_CLOCK;
            //Spi_deviceInstInfo[i].maxChipSelect = 1;
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
static int stSpiBindDev(void **devp, int devId, void *devParams)
{
    int         status        = IODEV_COMPLETED;
    Spi_ChanObj   *chanHandle   = NULL;
    uint8_t         count         = 0;
    Spi_Params    *params       = NULL;
    Spi_Object    *instHandle   = NULL;
    
	/* Begin parameter checking                                                   */
	#ifndef DRV_DISABLE_INPUT_PARAMETER_CHECK
    if ((N_SPI <= devId)   ||
        (1 == Spi_module.inUse[devId]))
    {
        status = IODEV_EBADARGS;
    }
	#endif
	/* End parameter checking                                                     */ 
    
    if (IODEV_COMPLETED == status)
    {
        if (devParams == NULL) {
            params = (Spi_Params*)&Spi_PARAMS;
        }
        else {
            params = (Spi_Params*) devParams;
        }
            
        instHandle =  &Spi_Instances[devId];  
    
        Spi_module.inUse[devId] = 1;

        instHandle->instNum           = devId;
        //instHandle->enableCache       = params->enableCache;
        instHandle->opMode            = params->opMode;
        instHandle->hDma             = NULL;
        instHandle->cpuEventNumber= params->cpuEventNumber;
        instHandle->spiHWconfig       = params->spiHWCfgData;
        instHandle->polledModeTimeout = params->polledModeTimeout;
        instHandle->numOpens          = 0;
        instHandle->devState = Spi_DriverState_CREATED;
        instHandle->csHighPolarity    = 0;
        instHandle->dmaChanAllocated  = 0;
        instHandle->edmaCbCheck       = 0;
        instHandle->currentActiveChannel = NULL;

        /*Inintialize Statistics members                                  */
        instHandle->stats.rxBytes = 0;
        instHandle->stats.txBytes = 0;
        instHandle->stats.pendingPacket = 0;
        instHandle->stats.rxOverrunError = 0;
        instHandle->stats.timeoutError = 0;
        instHandle->stats.bitError = 0;
        instHandle->stats.parityError = 0;
        instHandle->stats.desyncError = 0;

        instHandle->deviceInfo.baseAddress =
            Spi_deviceInstInfo[devId].baseAddress;
        instHandle->deviceInfo.inputFrequency =
            Spi_deviceInstInfo[devId].inputFrequency;
        instHandle->deviceInfo.cpuEventNumber =
            Spi_deviceInstInfo[devId].cpuEventNumber;
        instHandle->deviceInfo.rxDmaEventNumber =
            Spi_deviceInstInfo[devId].rxDmaEventNumber;
        instHandle->deviceInfo.txDmaEventNumber =
            Spi_deviceInstInfo[devId].txDmaEventNumber;

        for (count = 0; count < Spi_NUM_CHANS; count++)
        {
                chanHandle                   = &instHandle->chanObj[count];
                chanHandle->cbFxn            = NULL;
                chanHandle->cbArg            = NULL;
                chanHandle->mode             = IODEV_OUTPUT;
                chanHandle->instHandle       = NULL;
                chanHandle->channelState     = Spi_DriverState_CLOSED;
                chanHandle->busFreq          = params->outputClkFreq;
                chanHandle->loopbackEnabled  = params->loopbackEnabled;
                chanHandle->pendingState     = 0;
                chanHandle->cancelPendingIO  = 0;
                chanHandle->charLength16Bits = 0;
                chanHandle->currError        = 0;
                chanHandle->currFlags        = 0;
                chanHandle->transcieveFlags  = 0;
                chanHandle->currBuffer       = NULL;
                chanHandle->transBuffer      = NULL;
                chanHandle->currBufferLen    = 0;
                chanHandle->taskPriority     = 0;
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
static int stSpiUnBindDev(void *devp)
{
    int       result = IODEV_COMPLETED;
    Spi_Object  *instHandle = NULL;
    
	/* Begin parameter checking                                                   */
	#ifndef DRV_DISABLE_INPUT_PARAMETER_CHECK
    if ((NULL == devp) ||
        (N_SPI <= ((Spi_Object *)devp)->instNum))
    {
        result = IODEV_EBADARGS;
    }
	#endif
	/* End parameter checking                                                     */ 

    if (IODEV_COMPLETED == result)
    {
        instHandle = (Spi_Object *)devp;
        /* set driver state to deleted                                        */
        instHandle->numOpens = 0;
        instHandle->devState = Spi_DriverState_DELETED;
        Spi_module.inUse[instHandle->instNum] = 0;
    }
    return (result);
}



/* ========================================================================== */
/*                           SPI DRIVER FUNCTIONS                                */
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
 * \param     obj          [IN]     Spi driver object
 * \param     name         [IN]     Spi Instance name like Spi0
 * \param     mode         [IN]     channel  mode -> input or output
 * \param     chanParams   [IN]     channel parameters from user
 * \param     cbFxn        [IN]     callback function pointer
 * \param     cbArg        [IN]     callback function Arguments
 *
 * \return    channel handle in case of success
 *            NULL   in case of failure
 *
 */
static int stSpiCreateChan (void **chanp, void *devp, char *name, int mode, void *chanParams, IODEV_TiomCallback cbFxn, void *cbArg)
{
	Spi_Object		*instHandle = (Spi_Object *)devp;
	Spi_ChanObj		*chanHandle = NULL;
	Spi_ChanParams	*pChanParams = NULL;
	uint32_t		key = 0;
	uint32_t		chanCount = 0;
	int				status = IODEV_COMPLETED;
	xTaskHandle		thisTask   = NULL;
	NVIC_InitTypeDef nvicInit;
	

	#ifdef SPI_GPIO_DIR_ON_OPEN
	#ifdef EXTENDED_CHIP_SELECT
	uint16_t		cnt;
	uint8_t			mask;
	uint8_t			tmpMask;
	GPIO_TypeDef	*gpioPort;
	uint16_t		gpioPin;
	uint32_t		gpioClk;
	uint8_t			gpioPinSource;
	GPIO_InitTypeDef	gpioInitSt;
	#endif
	#endif

	/* Begin parameter checking                                                   */
	#ifndef DRV_DISABLE_INPUT_PARAMETER_CHECK
	if ((NULL == instHandle) || (NULL == cbFxn) || (NULL == cbArg))
	{
		status = IODEV_EBADARGS;
	}
	#endif
	/* End parameter checking                                                     */ 

	if(IODEV_COMPLETED == status)
	{
		do
		{
			for (chanCount = 0; chanCount < Spi_NUM_CHANS; chanCount++)
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
			thisTask = xTaskGetCurrentTaskHandle();

			/* Get task priority( Task_Handle handle );                   */
			chanHandle->taskPriority = uxTaskPriorityGet(thisTask);

			if (chanParams==NULL)
				instHandle->hDma = NULL;
			else
			{
				pChanParams = (Spi_ChanParams *)chanParams;
				instHandle->hDma = pChanParams->hDma;
			}

			/* Obtain gpio handle                                             */

			if (Spi_CommMode_SLAVE == instHandle->spiHWconfig.masterOrSlave) 
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
				if (instHandle->deviceInfo.baseAddress== SPI1)
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
				else if (instHandle->deviceInfo.baseAddress== SPI2)
					RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
				else if (instHandle->deviceInfo.baseAddress == SPI3)
					RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,ENABLE);
				
				
				#ifdef EXTENDED_CHIP_SELECT 
				#ifdef SPI_GPIO_DIR_ON_OPEN
				if (Spi_CommMode_MASTER== instHandle->spiHWconfig.masterOrSlave) 
				{
					if (instHandle->spiHWconfig.extGpiochipSelect)
					{
						tmpMask=instHandle->spiHWconfig.extGpiochipSelect->enableMask;
						for (cnt=0,mask=1;mask!=0;mask<<=1, cnt++)
						{
							if (tmpMask & mask)
							{
								
								gpioPort = instHandle->spiHWconfig.extGpiochipSelect->extGpioPinPort[cnt];
								gpioPin = instHandle->spiHWconfig.extGpiochipSelect->extGpioPinNo[cnt];
								gpioClk = instHandle->spiHWconfig.extGpiochipSelect->extGpioPinClock[cnt];


								/* Configure GPIO pin as an output									  */
								
								GPIO_StructInit(&gpioInitSt);
								
								gpioInitSt.GPIO_Pin=gpioPin;
								gpioInitSt.GPIO_Mode=GPIO_Mode_OUT;
								gpioInitSt.GPIO_Speed=GPIO_Speed_100MHz;
								
								GPIO_Init(gpioPort,&gpioInitSt);

								/* Make the GPIO pin high									  */
								GPIO_WriteBit(gpioPort,gpioPin,(BitAction)Spi_HIGH);
							}
						}
					}
				}

				#endif
				#endif

				key = __disableInterrupts();


				/* If interrupt mode, register ISR                                */
				if (Spi_OpMode_POLLED != instHandle->opMode)
				{
					installInterruptHandler(instHandle->deviceInfo.cpuEventNumber,spiIntrHandler,instHandle);
					
					/* configure interrupts 										  */
					nvicInit.NVIC_IRQChannel = instHandle->deviceInfo.cpuEventNumber;
					nvicInit.NVIC_IRQChannelPreemptionPriority = 15;
					nvicInit.NVIC_IRQChannelSubPriority = 1;
					nvicInit.NVIC_IRQChannelCmd = ENABLE;
					NVIC_Init(&nvicInit);
					SPI_ITConfig(instHandle->deviceInfo.baseAddress, SPI_I2S_IT_TXE, DISABLE);
					SPI_ITConfig(instHandle->deviceInfo.baseAddress, SPI_I2S_IT_RXNE, DISABLE);
					SPI_ITConfig(instHandle->deviceInfo.baseAddress, SPI_I2S_IT_ERR, DISABLE);
				}

				#ifdef Spi_DMA_ENABLE
				if (instHandle->hDma!=NULL)
				{
					status = spiLocalDmaChannel_Request(instHandle);
					
					if (IODEV_COMPLETED == status)
						instHandle->dmaChanAllocated=1;
				}
				#endif
				if (IODEV_COMPLETED == status)
				{
					/* Initialize hardware                                    */
					status = spiSetupConfig(chanHandle);
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
		chanHandle->channelState = Spi_DriverState_OPENED;
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
 *  \param    instHandle [IN]   Spi driver structure
 *            chanp      [IN]   Handle to the channel.
 *            eb         [OUT]  pointer to the error information block.
 *
 *  \return   None
 */
static int stSpiDeleteChan(void *chanp)
{
	Spi_Object	*instHandle;
	Spi_ChanObj	*chanHandle = (Spi_ChanObj *)chanp;
	uint32_t	key = 0;
	int	status = IODEV_COMPLETED;
	NVIC_InitTypeDef nvicInit;

	/* Begin parameter checking                                                   */
	#ifndef DRV_DISABLE_INPUT_PARAMETER_CHECK
	if ((NULL == chanHandle) || (NULL == chanHandle->instHandle) || (Spi_DriverState_OPENED != chanHandle->channelState))
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
			spiIoctl(chanHandle,Spi_IOCTL_CANCEL_PENDING_IO,NULL,NULL);
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
			if (instHandle->opMode != Spi_OpMode_POLLED)
			{
				/* configure interrupts 										  */
				SPI_ITConfig(instHandle->deviceInfo.baseAddress, SPI_I2S_IT_TXE, DISABLE);
				SPI_ITConfig(instHandle->deviceInfo.baseAddress, SPI_I2S_IT_RXNE, DISABLE);
				SPI_ITConfig(instHandle->deviceInfo.baseAddress, SPI_I2S_IT_ERR, DISABLE);
				nvicInit.NVIC_IRQChannel = instHandle->deviceInfo.cpuEventNumber;
				nvicInit.NVIC_IRQChannelPreemptionPriority = 15;
				nvicInit.NVIC_IRQChannelSubPriority = 1;
				nvicInit.NVIC_IRQChannelCmd = DISABLE;
				NVIC_Init(&nvicInit);
				uninstallInterruptHandler(instHandle->deviceInfo.cpuEventNumber);
			}

			#ifdef Spi_DMA_ENABLE
			/* Free the Edma Channels  if mode is DMA                 */
			if ((Spi_OpMode_INTERRUPTDMA== instHandle->opMode) && (0 != instHandle->dmaChanAllocated))
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
			#endif  /* Spi_DMA_ENABLE */
			/* Disable the spi enable pin                                 */
			//SPI_SSOutputCmd(instHandle->deviceInfo.baseAddress,DISABLE);

			/* Put the device in reset mode and quit                      */
			SPI_I2S_DeInit(instHandle->deviceInfo.baseAddress);

		}

		chanHandle->instHandle = NULL;
		/* Updated the driver state                                       */
		chanHandle->channelState = Spi_DriverState_CLOSED;

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
static int stSpiSubmitChan(void *chanp, IODEV_Packet *ioPacket)
{
	Spi_Object		*instHandle = NULL;
	Spi_ChanObj		*chanHandle = NULL;
	int				status = IODEV_COMPLETED;
	Spi_DataParam	*dataparam = NULL;
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
		chanHandle = (Spi_ChanObj *)chanp;
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
			chanHandle = (Spi_ChanObj *)chanp;
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

			dataparam = (Spi_DataParam *)ioPacket->addr;

			/* Validating State of the channel                                */
			if (Spi_DriverState_OPENED  !=  chanHandle->channelState)
			{
				status = IODEV_EBADIO;
				break;
			}

			if ((NULL == chanHandle->instHandle) || (NULL == dataparam->outBuffer))
			{
				status = IODEV_EBADIO;
				break;
			}

			if (dataparam->bufLen == 0)
			{
				status = IODEV_EBADIO;
				break;
			}

			#ifdef KUKU
			/* Supports gpioChipSelect only with CSHOLD enabled mode          */
			if (Spi_CSHOLD != (dataparam->flags & Spi_CSHOLD) && (0 != instHandle->spiHWconfig.gpioChipselectFlag))
			{
				status = IODEV_EBADARGS;
				break;
			}
			#endif

			#ifdef Spi_DMA_ENABLE
			/* if the transfer is in DMA mode                                 */
			/* Validate the buffer alignment if mode is DMA                   */
			if (instHandle->opMode == Spi_OpMode_INTERRUPTDMA)
			{
				if ((0 != (((uint32_t)dataparam->outBuffer) % 1)) || (0 != (((uint32_t)dataparam->inBuffer) % 1)))
				{
					status = IODEV_EBADARGS;
					break;
				}
			}
			#endif  /* Spi_EDMA_ENABLE */

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
				for (cnt =0; cnt < Spi_BUFFER_DATA_SIZE; cnt++)
				{
					Spi_transReceive[cnt] = 0;
				}

				chanHandle->transBuffer = &Spi_transReceive[0];

				/* When user does not need transceive operations i.e          *
				* if spitranbuff is null update transflags as FALSE          */
				transFlags = 0;
			}
			else
			{
				/* when passed spitransbuff is not NULL                       */
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

				/* Call the Spi_transfer data transfer through hardware       */
				status =  spiTransfer(instHandle, chanHandle, dataparam, chanHandle->transBuffer, transFlags, instHandle->polledModeTimeout, (uint32_t *) &cnt);
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
 *  \param    instHandle [IN]    Spi driver structure
 *  \param    chanp      [IN]    Channel handle
 *  \param    cmd        [IN]    control command given by the application
 *  \param    cmdArgs    [IN]    Optional args required for command execution
 *  \param    eb         [OUT]   error block
 *
 *  \return   None
 */
static int stSpiControlChan(void *chanp, unsigned int cmd, void *cmdArg)
{
	Spi_ChanObj	*chanHandle = (Spi_ChanObj *)chanp;
	int			status      = IODEV_COMPLETED;

	/* Begin parameter checking                                                   */
	#ifndef DRV_DISABLE_INPUT_PARAMETER_CHECK
	if ((NULL == chanp) || (Spi_DriverState_OPENED!= chanHandle->channelState))
	{
		/* invalid params have been detected                                  */
		status = IODEV_EBADARGS;
	}
	#endif /* DRV_DISABLE_INPUT_PARAMETER_CHECK */
	/* End parameter checking                                                     */ 

	if (IODEV_COMPLETED == status)
	{
		/* call the function to execute the control commands              */
		status = spiIoctl(chanHandle, (Spi_ioctlCmd)cmd,(void *) cmdArg, NULL);
	}

	return (status);
}

/* ========================================================================== */
/*                            LOCAL  FUNCTIONS                                */
/* ========================================================================== */

/**
 *  \brief  This to configure the data control bit of SPI data register SPIDAT1.
 *         All four params (CSHold, waitdelay, chip-select, Data format) will
 *         be uploaded to SPIDAT1 before transfer starts.
 *
 *  \param  handle      [IN]   SPI driver object for respective instance.
 *  \param  chipSelect  [IN]   Chip select number
 *  \param  dataFormat  [IN]   Data format register selection
 *  \param  flags       [IN]   Flag for Read/Write of data
 *
 */
void spiLocalControlDataConfig(Spi_ChanObj *chanHandle, uint32_t chipSelect, Spi_DataFormat dataFormat, uint32_t flags)
{
    //register uint8_t          csHold     = 0;
    register Spi_Object     *instHandle;
	#ifdef EXTENDED_CHIP_SELECT
	GPIO_InitTypeDef	gpioInitSt;
	register uint16_t	cnt;
	register uint8_t	mask;
	register uint8_t	tmpMask;
	register uint16_t	csMax;
	register uint16_t *extGpioPinNo;
	GPIO_TypeDef	*gpioPort;
	uint16_t		gpioPin;
	uint32_t		gpioClk;
	#endif
	register Spi_HWConfigData *hwc;
	register SPI_TypeDef *spiRegs;
	SPI_InitTypeDef spiInitSt;

    assert(NULL != chanHandle);

    instHandle = chanHandle->instHandle;

    //if (Spi_CSHOLD == (flags & Spi_CSHOLD))
    //{
    //    csHold = 1;
    //}

	spiRegs=instHandle->deviceInfo.baseAddress;
	hwc=&instHandle->spiHWconfig;
	
	if (/*instHandle->spiHWconfig.*/hwc->configDatafmt[dataFormat].clkHigh)
	{
		// Pull up the SPI_CLK pin
		if (spiRegs==SPI1)
		{
			#ifdef USE_SPI1_GPIO
			SPI1_CLK_GPIO_PORT->PUPDR= (SPI1_CLK_GPIO_PORT->PUPDR & (~(GPIO_PUPDR_PUPDR0 << ((uint16_t)SPI1_CLK_PIN_SOURCE * 2)))) |
												(((uint32_t)GPIO_PuPd_UP) << (SPI1_CLK_PIN_SOURCE * 2));
			#endif
		}
		else if (spiRegs==SPI2)
		{
			#ifdef USE_SPI2_GPIO
			SPI2_CLK_GPIO_PORT->PUPDR= (SPI2_CLK_GPIO_PORT->PUPDR & (~(GPIO_PUPDR_PUPDR0 << ((uint16_t)SPI2_CLK_PIN_SOURCE * 2)))) |
												(((uint32_t)GPIO_PuPd_UP) << (SPI2_CLK_PIN_SOURCE * 2));
			#endif
		}
		else if (spiRegs==SPI3)
		{
			#ifdef USE_SPI3_GPIO
			SPI3_CLK_GPIO_PORT->PUPDR= (SPI3_CLK_GPIO_PORT->PUPDR & (~(GPIO_PUPDR_PUPDR0 << ((uint16_t)SPI3_CLK_PIN_SOURCE * 2)))) |
												(((uint32_t)GPIO_PuPd_UP) << (SPI3_CLK_PIN_SOURCE * 2));
			#endif
		}
	}
	
	else
	{
		// Pull down the SPI_CLK pin
		if (spiRegs==SPI1)
		{
			#ifdef USE_SPI1_GPIO
			SPI1_CLK_GPIO_PORT->PUPDR= (SPI1_CLK_GPIO_PORT->PUPDR & (~(GPIO_PUPDR_PUPDR0 << ((uint16_t)SPI1_CLK_PIN_SOURCE * 2)))) |
												(((uint32_t)GPIO_PuPd_DOWN) << (SPI1_CLK_PIN_SOURCE * 2));
			#endif
		}
		else if (spiRegs==SPI2)
		{
			#ifdef USE_SPI2_GPIO
			SPI2_CLK_GPIO_PORT->PUPDR= (SPI2_CLK_GPIO_PORT->PUPDR & (~(GPIO_PUPDR_PUPDR0 << ((uint16_t)SPI2_CLK_PIN_SOURCE * 2)))) |
												(((uint32_t)GPIO_PuPd_DOWN) << (SPI2_CLK_PIN_SOURCE * 2));
			#endif
		}
		else if (spiRegs==SPI3)
		{
			#ifdef USE_SPI3_GPIO
			SPI3_CLK_GPIO_PORT->PUPDR= (SPI3_CLK_GPIO_PORT->PUPDR & (~(GPIO_PUPDR_PUPDR0 << ((uint16_t)SPI3_CLK_PIN_SOURCE * 2)))) |
												(((uint32_t)GPIO_PuPd_DOWN) << (SPI3_CLK_PIN_SOURCE * 2));
			#endif
		}
	}

	/* configure spi clock phase, clock polarity, char length, loopback, inter-word delay */
	//SPI_StructInit(&spiInitSt);

	spiInitSt.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	spiInitSt.SPI_Mode = (/*instHandle->spiHWconfig.*/hwc->masterOrSlave==Spi_CommMode_MASTER) ? SPI_Mode_Master : SPI_Mode_Slave;
	spiInitSt.SPI_DataSize = ((/*instHandle->spiHWconfig.*/hwc->configDatafmt[dataFormat].charLength<=8) ? SPI_DataSize_8b: SPI_DataSize_16b);
	spiInitSt.SPI_CPOL =  ((/*instHandle->spiHWconfig.*/hwc->configDatafmt[dataFormat].clkHigh) ? SPI_CPOL_High : SPI_CPOL_Low)  ;
	spiInitSt.SPI_CPHA =((/*instHandle->spiHWconfig.*/hwc->configDatafmt[dataFormat].phaseIn) ? SPI_CPHA_2Edge : SPI_CPHA_1Edge ) ;
	//spiInitSt.SPI_NSS = SPI_NSS_Soft;
	spiInitSt.SPI_NSS = (Spi_CommMode_MASTER == /*instHandle->spiHWconfig.*/hwc->masterOrSlave) ? SPI_NSS_Soft: (instHandle->spiHWconfig.pinOpModes==Spi_PinOpMode_3PIN) ?SPI_NSS_Soft : SPI_NSS_Hard;
	spiInitSt.SPI_BaudRatePrescaler = spiRegs->CR1&SPI_BaudRatePrescaler_256;
	spiInitSt.SPI_FirstBit = SPI_FirstBit_MSB;
	spiInitSt.SPI_CRCPolynomial = SPI_GetCRCPolynomial(spiRegs);
	
	/* Disable SPI 							  */
	SPI_Cmd(spiRegs, DISABLE);
	SPI_Init(spiRegs, &spiInitSt);
	//SPI_NSSInternalSoftwareConfig(spiRegs, SPI_NSSInternalSoft_Set);
	
	/* Enable SPI for further transaction							  */
	SPI_Cmd(spiRegs, ENABLE);
		

	
	#ifdef EXTENDED_CHIP_SELECT	
	if (/*instHandle->spiHWconfig.*/hwc->extGpiochipSelect)
	{
		tmpMask=/*instHandle->spiHWconfig.*/hwc->extGpiochipSelect->enableMask;
		csMax=sizeof(/*instHandle->spiHWconfig.*/hwc->extGpiochipSelect->extGpioPinNo)/sizeof(/*instHandle->spiHWconfig.*/hwc->extGpiochipSelect->extGpioPinNo[0]);
		extGpioPinNo=/*instHandle->spiHWconfig.*/hwc->extGpiochipSelect->extGpioPinNo;
		for (cnt=0,mask=1;cnt<csMax;mask<<=1, cnt++)
		{
			if (tmpMask & mask)
			{
				gpioPort = /*instHandle->spiHWconfig.*/hwc->extGpiochipSelect->extGpioPinPort[cnt];
				gpioPin = /*instHandle->spiHWconfig.*/hwc->extGpiochipSelect->extGpioPinNo[cnt];
				GPIO_WriteBit(gpioPort,gpioPin, ((mask & chipSelect) ? Bit_RESET : Bit_SET));
				#ifndef SPI_GPIO_DIR_ON_OPEN
				/* Configure GPIO pin as an output                                    */
				gpioClk = /*instHandle->spiHWconfig.*/hwc->extGpiochipSelect->extGpioPinClock[cnt];
				
				/* Configure GPIO pin as an output									  */
				
				GPIO_StructInit(&gpioInitSt);
				
				gpioInitSt.GPIO_Pin=gpioPin;
				gpioInitSt.GPIO_Mode=GPIO_Mode_OUT;
				gpioInitSt.GPIO_Speed=GPIO_Speed_100MHz;
				
				GPIO_Init(gpioPort,&gpioInitSt);
				#endif
			}
		}
	}
	#endif
	
	if ((Spi_PinOpMode_SPISCS_4PIN == /*instHandle->spiHWconfig.*/hwc->pinOpModes)
		|| (Spi_PinOpMode_5PIN == /*instHandle->spiHWconfig.*/hwc->pinOpModes))
	{
		/* Enable CS Hold only for 4 pin cs and 5 pin mode					  */
	
		if (0 == instHandle->csHighPolarity)
		{
			/* Set the selected CS line as active Low					  */
			/* Make the GPIO pin low									  */
			if ((0 != /*instHandle->spiHWconfig.*/hwc->gpioChipselectFlag) && (/*instHandle->spiHWconfig.*/hwc->gpioPinPort))
			{
				GPIO_WriteBit(/*instHandle->spiHWconfig.*/hwc->gpioPinPort,/*instHandle->spiHWconfig.*/hwc->gpioPinNo,Bit_RESET);
			}
			else if (spiRegs==SPI1)
			{
				#ifdef USE_SPI1_GPIO
				GPIO_WriteBit(SPI1_NSS_GPIO_PORT,SPI1_NSS_PIN, Bit_RESET);
				#endif
			}
			else if (spiRegs==SPI2)
			{
				#ifdef USE_SPI2_GPIO
				GPIO_WriteBit(SPI2_NSS_GPIO_PORT,SPI2_NSS_PIN, Bit_RESET);
				#endif
			}
			else if (spiRegs==SPI3)
			{
				#ifdef USE_SPI3_GPIO
				GPIO_WriteBit(SPI3_NSS_GPIO_PORT,SPI3_NSS_PIN, Bit_RESET);
				#endif
			}
		}
		else
		{
			/* Set the selected CS line as active HIGH					  */
			/* Make the GPIO pin high									  */
			if ((/*instHandle->spiHWconfig.*/hwc->gpioChipselectFlag) && (/*instHandle->spiHWconfig.*/hwc->gpioPinPort))
				GPIO_WriteBit(/*instHandle->spiHWconfig.*/hwc->gpioPinPort,/*instHandle->spiHWconfig.*/hwc->gpioPinNo,Bit_SET);
			else if (spiRegs==SPI1)
			{
				#ifdef USE_SPI1_GPIO
				GPIO_WriteBit(SPI1_NSS_GPIO_PORT,SPI1_NSS_PIN, Bit_SET);
				#endif
			}
			else if (spiRegs==SPI2)
			{
				#ifdef USE_SPI2_GPIO
				GPIO_WriteBit(SPI2_NSS_GPIO_PORT,SPI2_NSS_PIN, Bit_SET);
				#endif
			}
			else if (spiRegs==SPI3)
			{
				#ifdef USE_SPI3_GPIO
				GPIO_WriteBit(SPI3_NSS_GPIO_PORT,SPI3_NSS_PIN, Bit_SET);
				#endif
			}
		}
	}
	else
	{
		/* Enable CS Hold only for 4 pin cs and 5 pin mode					  */
	}

	/* To make transaction is in progress                                     */
    chanHandle->pendingState = 1;
}

/**
 *  \brief  Function call for spi data transfer. It transfer data as transcieve
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
static int spiTransfer(Spi_Object *instHandle, Spi_ChanObj *chanHandle, Spi_DataParam *dataparam, uint8_t *inBuffer, int transflags, int timeout, void *xferActual)
{
	int				retVal = IODEV_COMPLETED;
	register uint8_t			*outBuffer/* = NULL*/;
	register uint32_t		bufLen/* = 0*/;
	uint32_t		chipSelect/* = 0*/;
	Spi_DataFormat	dataFormat;
	uint32_t		flags/* = 0*/;
	#ifdef EXTENDED_CHIP_SELECT
	register uint16_t cnt;
	register uint8_t mask;
	register uint8_t tmpMask;
	register uint16_t	csMax;
	register uint16_t *extGpioPinNo;
	GPIO_TypeDef	*gpioPort;
	uint16_t		gpioPin;
	#endif
	register Spi_HWConfigData *hwc;
	register SPI_TypeDef *spiRegs;
	
	assert((NULL != instHandle)&&(NULL != chanHandle) && (NULL != dataparam)&&(NULL != inBuffer));

	outBuffer  = dataparam->outBuffer;
	bufLen     = dataparam->bufLen;
	chipSelect = dataparam->chipSelect;
	dataFormat = dataparam->dataFormat;
	flags      = dataparam->flags;

	instHandle->currentActiveChannel = chanHandle;

	spiRegs=instHandle->deviceInfo.baseAddress;
	hwc=&instHandle->spiHWconfig;

	
	/** If polled mode, wait for data transfer & completion, else wait     *
	* (on a SEM) for transaction to complete in interrupt mode      *
	* In interrupt mode, for write case, it has to poll on RX_INT bit,    *
	* as there is no transmit interrupt bit to set                        */
	if (Spi_OpMode_INTERRUPTDMA == instHandle->opMode)
	{
		/* DMAINTERRUPT Mode of data transfer                             */
		#ifdef Spi_DMA_ENABLE
		/* Mask all interrupts,& populate data to spi object structure*
		* so driver can extact all information at dma configSpiion  *
		* and dma Rx & Tx callback                                   */


		/* Transmit data                                              */
		while (SPI_I2S_GetFlagStatus(spiRegs,SPI_I2S_FLAG_RXNE))
		{
			/* Read before Write to remove any stale data             */
			*chanHandle->transBuffer = (uint8_t)(SPI_I2S_ReceiveData(spiRegs));
		}

		/*disable all SPI interrupts */
		//SPI_I2S_ITConfig(spiRegs,SPI_I2S_IT_TXE,DISABLE);
		SPI_I2S_ITConfig(spiRegs,SPI_I2S_IT_RXNE,DISABLE);
		SPI_I2S_ITConfig(spiRegs,SPI_I2S_IT_ERR,DISABLE);

		chanHandle->currError           = 0;
		chanHandle->currFlags           = flags;
		chanHandle->transcieveFlags     = transflags;
		chanHandle->currBuffer          = outBuffer;
		chanHandle->currBufferLen       = bufLen;

		/* Call this to complete dma configuration and transfer      */
		retVal = spiLocalDmaTransfer(instHandle,chipSelect,dataFormat,flags);

		if (IODEV_COMPLETED ==  retVal)
		{
			retVal = IODEV_PENDING;                
		}           

		#endif/* DMA MODE CODE ENDS                                                  */                
	}

	/* POLLED MODE CODE STARTS                                                */     
	else if (Spi_OpMode_POLLED == instHandle->opMode)
	{
		//spiLocalControlDataConfig(chanHandle,chipSelect,dataFormat,flags);
		retVal = spiPolledModeTransfer(chanHandle);

		/* check if CSHOLD needs to be enable after the transmission also.        *
		* If No or if there is any eror disable CSHOLD value, else do nothing    *
		* Also toggle the gpio pin if gpioChipSelect is enabled                  */    
		if (((flags & (Spi_CSHOLD_FOR_MULTI_TRANSCEIVE)) !=  Spi_CSHOLD_FOR_MULTI_TRANSCEIVE) || (retVal == IODEV_EBADIO))
		{

			if (0 == instHandle->csHighPolarity)
			{
				#ifdef EXTENDED_CHIP_SELECT
				if (/*instHandle->spiHWconfig.*/hwc->extGpiochipSelect)
				{
					tmpMask= chipSelect & /*instHandle->spiHWconfig.*/hwc->extGpiochipSelect->enableMask;
					csMax=sizeof(/*instHandle->spiHWconfig.*/hwc->extGpiochipSelect->extGpioPinNo)/sizeof(/*instHandle->spiHWconfig.*/hwc->extGpiochipSelect->extGpioPinNo[0]);
					extGpioPinNo=/*instHandle->spiHWconfig.*/hwc->extGpiochipSelect->extGpioPinNo;
					if (tmpMask);
					{
						for (cnt=0,mask=1;cnt<csMax;mask<<=1, cnt++)
						{
							if ( tmpMask & mask)
							{
								/* Make the GPIO pin High									   */
								gpioPort = /*instHandle->spiHWconfig.*/hwc->extGpiochipSelect->extGpioPinPort[cnt];
								gpioPin = extGpioPinNo[cnt];
								GPIO_WriteBit(gpioPort,gpioPin, Bit_SET);
							}	
						}
					}
					
				}
				else
				#endif
				{
					/* Make the GPIO pin high									   */
					if ((/*instHandle->spiHWconfig.*/hwc->gpioChipselectFlag)&&(instHandle->spiHWconfig.gpioPinPort))
						GPIO_WriteBit(instHandle->spiHWconfig.gpioPinPort,instHandle->spiHWconfig.gpioPinNo,Bit_SET);
					else if (spiRegs==SPI1)
					{
						#ifdef USE_SPI2_GPIO
						GPIO_WriteBit(SPI1_NSS_GPIO_PORT,SPI1_NSS_PIN, Bit_SET);
						#endif
					}
					else if (spiRegs==SPI2)
					{
						#ifdef USE_SPI2_GPIO
						GPIO_WriteBit(SPI2_NSS_GPIO_PORT,SPI2_NSS_PIN, Bit_SET);
						#endif
					}
					else if (spiRegs==SPI3)
					{
						#ifdef USE_SPI3_GPIO
						GPIO_WriteBit(SPI3_NSS_GPIO_PORT,SPI3_NSS_PIN, Bit_SET);
						#endif
					}
				}
			}
			else
			{
				#ifdef EXTENDED_CHIP_SELECT
				if (/*instHandle->spiHWconfig.*/hwc->extGpiochipSelect)
				{
					tmpMask= chipSelect & /*instHandle->spiHWconfig.*/hwc->extGpiochipSelect->enableMask;
					csMax=sizeof(/*instHandle->spiHWconfig.*/hwc->extGpiochipSelect->extGpioPinNo)/sizeof(/*instHandle->spiHWconfig.*/hwc->extGpiochipSelect->extGpioPinNo[0]);
					extGpioPinNo=/*instHandle->spiHWconfig.*/hwc->extGpiochipSelect->extGpioPinNo;
					if (tmpMask);
					{
						for (cnt=0,mask=1;cnt<csMax;mask<<=1, cnt++)
						{
							if ( tmpMask & mask)
							{
								/* Make the GPIO pin Low									   */
								gpioPort = /*instHandle->spiHWconfig.*/hwc->extGpiochipSelect->extGpioPinPort[cnt];
								gpioPin = extGpioPinNo[cnt];
								GPIO_WriteBit(gpioPort,gpioPin, Bit_RESET);
							}	
						}
					}
					
				}
				else
				#endif
				{
					/* Make the GPIO pin low									  */
					if ((/*instHandle->spiHWconfig.*/hwc->gpioChipselectFlag)||(/*instHandle->spiHWconfig.*/hwc->gpioPinPort))
						GPIO_WriteBit(/*instHandle->spiHWconfig.*/hwc->gpioPinPort,/*instHandle->spiHWconfig.*/hwc->gpioPinNo,Bit_RESET);
					else if (spiRegs==SPI1)
					{
						#ifdef USE_SPI1_GPIO
						GPIO_WriteBit(SPI1_NSS_GPIO_PORT,SPI1_NSS_PIN, Bit_RESET);
						#endif
					}
					else if (spiRegs==SPI2)
					{
						#ifdef USE_SPI2_GPIO
						GPIO_WriteBit(SPI2_NSS_GPIO_PORT,SPI2_NSS_PIN, Bit_RESET);
						#endif
					}
					else if (spiRegs==SPI3)
					{
						#ifdef USE_SPI3_GPIO
						GPIO_WriteBit(SPI3_NSS_GPIO_PORT,SPI3_NSS_PIN, Bit_RESET);
						#endif
					}
				}
			}
		}



	}/*POLLED MODE CODE ENDS                                                  */


	/* INTERUPPT MODE CODE STARTS                                             */
	else if (Spi_OpMode_INTERRUPT == instHandle->opMode)
	{
		/* Populate the user send params to spi object structure              *
		* So device can use all the information when running at  ISR.        */
		chanHandle->currError         = 0;
		chanHandle->currFlags         = flags;
		chanHandle->transcieveFlags   = transflags;
		chanHandle->currBuffer        = outBuffer;
		chanHandle->currBufferLen     = bufLen;

		
		/*Consume any stale data                                              */
		while (SPI_I2S_GetFlagStatus(spiRegs,SPI_I2S_FLAG_RXNE))
		{
			/* Read before Write to remove any stale data             */
			*chanHandle->transBuffer = (uint8_t)(SPI_I2S_ReceiveData(spiRegs));
		}

		spiLocalControlDataConfig(chanHandle,chipSelect,dataFormat,flags);
		if (/*instHandle->spiHWconfig.*/hwc->configDatafmt[dataFormat].charLength > 8u)
		{
			chanHandle->charLength16Bits = 1;
			
			if ((chanHandle->currBufferLen) > 0)
			{
				SPI_I2S_SendData(spiRegs, (*((uint16_t*)outBuffer/*chanHandle->currBuffer*/)));
				chanHandle->currBuffer += 2;
				/* Enabling receive and error interrupts						  */
				SPI_I2S_ITConfig(spiRegs,SPI_I2S_IT_ERR,ENABLE);
				SPI_I2S_ITConfig(spiRegs,SPI_I2S_IT_RXNE,ENABLE);
			}
		}
		else
		{
			chanHandle->charLength16Bits = 0;
			if ((chanHandle->currBufferLen) > 0)
			{
				SPI_I2S_SendData(spiRegs, *outBuffer/*chanHandle->currBuffer*/);
				chanHandle->currBuffer++;
				/* Enabling receive and error interrupts						  */
				SPI_I2S_ITConfig(spiRegs,SPI_I2S_IT_ERR,ENABLE);
				SPI_I2S_ITConfig(spiRegs,SPI_I2S_IT_RXNE,ENABLE);
			}
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
static int spiPolledModeTransfer(Spi_ChanObj *chanHandle)
{
	int				status = IODEV_COMPLETED;
	uint32_t		timeCnt = 0;
	uint32_t		cnt = 0;
	//int             retStatus = 0;
	uint8_t			*outBuffer = NULL;
	int				transflags = 0; 
	Spi_Object		*instHandle = NULL;
	Spi_DataParam	*dataParam  = NULL;
	SPI_TypeDef		*spiRegs;

	assert(NULL != chanHandle);

	instHandle = chanHandle->instHandle;

	spiRegs=instHandle->deviceInfo.baseAddress;

	/* Polled mode of data transfer Mask all interrupts                       */
	SPI_I2S_ITConfig(spiRegs,SPI_I2S_IT_TXE,DISABLE);
	SPI_I2S_ITConfig(spiRegs,SPI_I2S_IT_RXNE,DISABLE);
	SPI_I2S_ITConfig(spiRegs,SPI_I2S_IT_ERR,DISABLE);

	/* To get current ticks to find out the data transfer timeout             */
	timeCnt = xTaskGetTickCount();

	dataParam  = (Spi_DataParam *)chanHandle->activeIOP->addr;
	outBuffer  = dataParam->outBuffer; 

	if (NULL == dataParam->inBuffer)
		transflags = 0;    
	else
		transflags = 1;

	spiLocalControlDataConfig(chanHandle,dataParam->chipSelect, dataParam->dataFormat, dataParam->flags);

	while (cnt < dataParam->bufLen)
	{
		if ((0 == spiCheckTimeOut(timeCnt,instHandle->polledModeTimeout))&& (IODEV_COMPLETED == status))
		{
			/* Transmit data                                                  */
			while (SPI_I2S_GetFlagStatus(spiRegs,SPI_I2S_FLAG_RXNE))
			{
				/* Read before Write to remove any stale data			  */
				*chanHandle->transBuffer = (uint8_t)(SPI_I2S_ReceiveData(spiRegs));
			}

			/*  Transmit data pend on TX_FULL flag i.e                        *
			*  TX_FULL == TRUE  i.e  data is already in transmit             *
			*  buffer. TX_FULL == FALSE i.e. No data in transmit             *
			*  buffer, write next data.                                      */

			if (RESET != SPI_I2S_GetFlagStatus(spiRegs,SPI_I2S_FLAG_TXE))
			{
				/* Write data in 4-pin CS & 5-pin modes                       */
				if (instHandle->spiHWconfig.configDatafmt[dataParam->dataFormat].charLength <= 8u)
					SPI_I2S_SendData(spiRegs,*outBuffer);
				else
					SPI_I2S_SendData(spiRegs,(*((uint16_t *)outBuffer)));

				//spiWaitForStaleData();

				/* Make sure data transfer is completed before                *
				* reading. Read after Write to implement                     *
				* transcieve operation                                       */

				/* read transceived data                                      */
				while (RESET == SPI_I2S_GetFlagStatus(spiRegs,SPI_I2S_FLAG_RXNE))
				{
					if (0 != spiCheckTimeOut(timeCnt, instHandle->polledModeTimeout))
					{
						instHandle->stats.pendingPacket++;
						status = IODEV_EBADIO;
						break;
					}
				}

				if (instHandle->spiHWconfig.configDatafmt[dataParam->dataFormat].charLength <= 8u)
				{
					*chanHandle->transBuffer = (uint8_t)(SPI_I2S_ReceiveData(spiRegs));
					outBuffer++;

					/* check this is transcieve call or not if yes            *
					* then increment the buffer else not & dumped the        *
					* data  to local buffer.                                 */
					if (0 != transflags)
						chanHandle->transBuffer++;
				}
				else
				{
					(*((uint16_t*)chanHandle->transBuffer)) = (SPI_I2S_ReceiveData(spiRegs));
					outBuffer += 2;
					/* check this is transcieve call or not if yes            *
					* then increment the buffer else not & dumped the        *
					* data  to local buffer.                                 */
					if (0 != transflags)
						chanHandle->transBuffer+=2;
				}
				cnt++;
			}

			/* Check for bit error, desync error,parity error,timeout         *
			* error and receive overflow errors                              */

			if (SPI_I2S_GetFlagStatus(spiRegs,SPI_I2S_FLAG_OVR))
			{
				SPI_I2S_ClearFlag(spiRegs,SPI_I2S_FLAG_OVR);
			
				status = Spi_RECEIVE_OVERRUN_ERR;
				instHandle->stats.rxOverrunError++; 				   
			}
		}
		else
		{
			if (IODEV_COMPLETED == status)
			{
				status = IODEV_EBADIO;
				instHandle->stats.timeoutError++;                
			}
			break;
		}
	}/* While                                                                 */

	/* Updating the transmit and receive stats                                */
	instHandle->stats.txBytes += cnt;
	instHandle->stats.rxBytes += cnt;

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
static int spiIoctl(void *handle, Spi_ioctlCmd cmd, void *cmdArg,void *param)
{
    uint32_t	key = 0;
    Spi_ChanObj	*chanHandle = NULL;
    Spi_Object	*instHandle = NULL;
    int			status = IODEV_COMPLETED;

    assert(NULL != handle);

    chanHandle = (Spi_ChanObj *)handle ;
    instHandle = chanHandle->instHandle;

    if (Spi_IOCTL_CANCEL_PENDING_IO == cmd)
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
    else if (Spi_IOCTL_SET_CS_POLARITY == cmd)
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
    else if (Spi_IOCTL_SET_POLLEDMODETIMEOUT == cmd)
    {
        /* Update the polledModeTimeout value                                 */
        instHandle->polledModeTimeout = *((uint32_t *)cmdArg);
    }
    else if (Spi_IOCTL_SET_CLOCK_RATE== cmd)
    {
        /* Update the polledModeTimeout value                                 */
        chanHandle->busFreq = *((uint32_t *)cmdArg);

		status=spiSetupClock(chanHandle);		
    }
    /* Unrecognised Command                                                   */
    else
        status = IODEV_EBADARGS;
    
    return (status);
}


/**
 *  \brief  Interrupt handler for SPI Device
 *
 *         It will check the following errors like bit error, desync error
 *         parity error, overrun error. Data transfer either read or write
 *         is done on RX INT flag only.
 *
 *  \param  instHandle  [IN]    Pointer to the spi driver object
 *
 *  \return None
 */
int spiIntrHandler(Spi_Object *instHandle)
{
    register Spi_ChanObj *chanHandle;
	register SPI_TypeDef *spiRegs;
	register Spi_HWConfigData *hwc;
	register uint8_t csHold;
	int completeIo=0;
	volatile uint32_t wDelayCnt;

    assert(NULL != instHandle);

	spiRegs=instHandle->deviceInfo.baseAddress;
    chanHandle = instHandle->currentActiveChannel;
	if (SPI_I2S_GetITStatus(spiRegs,SPI_I2S_IT_RXNE))
	{
		hwc= &instHandle->spiHWconfig;
		csHold= (((Spi_DataParam *)chanHandle->activeIOP->addr)->flags & Spi_CSHOLD)? 1 : 0;
		/* SPI_I2S_IT_RXNE flag is used for data recieve and Transmit because 	  *
		 * SPI device is transceive device								  */
		if (!csHold) 
		{
	        if (0 == instHandle->csHighPolarity)
	        {
				/* Make the GPIO pin high									   */
	            if ((/*instHandle->spiHWconfig.*/hwc->gpioChipselectFlag) && (/*instHandle->spiHWconfig.*/hwc->gpioPinPort))
					GPIO_WriteBit(/*instHandle->spiHWconfig.*/hwc->gpioPinPort,/*instHandle->spiHWconfig.*/hwc->gpioPinNo, Bit_SET);
				else if (spiRegs==SPI1)
				{
					#ifdef USE_SPI1_GPIO
					GPIO_WriteBit(SPI1_NSS_GPIO_PORT,SPI1_NSS_PIN, Bit_SET);
					#endif
				}
				else if (spiRegs==SPI2)
				{
					#ifdef USE_SPI2_GPIO
					GPIO_WriteBit(SPI2_NSS_GPIO_PORT,SPI2_NSS_PIN, Bit_SET);
					#endif
				}
				else if (spiRegs==SPI3)
				{
					#ifdef USE_SPI3_GPIO
					GPIO_WriteBit(SPI3_NSS_GPIO_PORT,SPI3_NSS_PIN, Bit_SET);
					#endif
				}
	        }
	        else
	        {
				/* Make the GPIO pin low									 */
	            if ((/*instHandle->spiHWconfig.*/hwc->gpioChipselectFlag) && (/*instHandle->spiHWconfig.*/hwc->gpioPinPort))
					GPIO_WriteBit(/*instHandle->spiHWconfig.*/hwc->gpioPinPort,/*instHandle->spiHWconfig.*/hwc->gpioPinNo, Bit_RESET);
				else if (spiRegs==SPI1)
				{
					#ifdef USE_SPI1_GPIO
					GPIO_WriteBit(SPI1_NSS_GPIO_PORT,SPI1_NSS_PIN, Bit_RESET);
					#endif
				}
				else if (spiRegs==SPI2)
				{
					#ifdef USE_SPI2_GPIO
					GPIO_WriteBit(SPI2_NSS_GPIO_PORT,SPI2_NSS_PIN, Bit_RESET);
					#endif
				}
				else if (spiRegs==SPI3)
				{
					#ifdef USE_SPI3_GPIO
					GPIO_WriteBit(SPI3_NSS_GPIO_PORT,SPI3_NSS_PIN, Bit_RESET);
					#endif
				}
	        }
	    }

		
		if (0 != chanHandle->currBufferLen)
		{
			if (0 != chanHandle->cancelPendingIO)
			{
				completeIo|=spiCompleteIOInIsr(instHandle);
			}
			else
			{
				if (0 != chanHandle->charLength16Bits)
				{
					(*((uint16_t*)chanHandle->transBuffer)) = SPI_I2S_ReceiveData(spiRegs);
					if (0 != chanHandle->transcieveFlags)
						chanHandle->transBuffer += 2;
					instHandle->stats.rxBytes++;
					--chanHandle->currBufferLen;
					if (0 == chanHandle->currBufferLen)
						completeIo|=spiCompleteIOInIsr(instHandle);
					else
					{
						if (!csHold) 
						{
							for (wDelayCnt=0;wDelayCnt<hwc->configDatafmt[((Spi_DataParam *)chanHandle->activeIOP->addr)->dataFormat].wDelay;wDelayCnt++)
								;
								
							if (0 == instHandle->csHighPolarity)
							{
								/* Make the GPIO pin low									 */
								if ((/*instHandle->spiHWconfig.*/hwc->gpioChipselectFlag) && (/*instHandle->spiHWconfig.*/hwc->gpioPinPort))
									GPIO_WriteBit(/*instHandle->spiHWconfig.*/hwc->gpioPinPort,/*instHandle->spiHWconfig.*/hwc->gpioPinNo, Bit_RESET);
								else if (spiRegs==SPI1)
								{
									#ifdef USE_SPI1_GPIO
									GPIO_WriteBit(SPI1_NSS_GPIO_PORT,SPI1_NSS_PIN, Bit_RESET);
									#endif
								}
								else if (spiRegs==SPI2)
								{
									#ifdef USE_SPI2_GPIO
									GPIO_WriteBit(SPI2_NSS_GPIO_PORT,SPI2_NSS_PIN, Bit_RESET);
									#endif
								}
								else if (spiRegs==SPI3)
								{
									#ifdef USE_SPI3_GPIO
									GPIO_WriteBit(SPI3_NSS_GPIO_PORT,SPI3_NSS_PIN, Bit_RESET);
									#endif
								}
							}
							else
							{
								/* Make the GPIO pin high									   */
								if ((/*instHandle->spiHWconfig.*/hwc->gpioChipselectFlag) && (/*instHandle->spiHWconfig.*/hwc->gpioPinPort))
									GPIO_WriteBit(/*instHandle->spiHWconfig.*/hwc->gpioPinPort,/*instHandle->spiHWconfig.*/hwc->gpioPinNo, Bit_SET);
								else if (spiRegs==SPI1)
								{
									#ifdef USE_SPI1_GPIO
									GPIO_WriteBit(SPI1_NSS_GPIO_PORT,SPI1_NSS_PIN, Bit_SET);
									#endif
								}
								else if (spiRegs==SPI2)
								{
									#ifdef USE_SPI2_GPIO
									GPIO_WriteBit(SPI2_NSS_GPIO_PORT,SPI2_NSS_PIN, Bit_SET);
									#endif
								}
								else if (spiRegs==SPI3)
								{
									#ifdef USE_SPI3_GPIO
									GPIO_WriteBit(SPI3_NSS_GPIO_PORT,SPI3_NSS_PIN, Bit_SET);
									#endif
								}
										
							}
						}
					
						/* Transmit user supplied data						  */
						SPI_I2S_SendData(spiRegs,(*((uint16_t *)chanHandle->currBuffer)));
						chanHandle->currBuffer += 2;
						instHandle->stats.txBytes++;
					}
				}
				else
				{
					*chanHandle->transBuffer = SPI_I2S_ReceiveData(spiRegs);
					if (0 != chanHandle->transcieveFlags)
						chanHandle->transBuffer++;
					instHandle->stats.rxBytes++;
					--chanHandle->currBufferLen;
					if (0 == chanHandle->currBufferLen)
						completeIo|=spiCompleteIOInIsr(instHandle);
					else
					{
						if (!csHold) 
						{
							for (wDelayCnt=0;wDelayCnt<hwc->configDatafmt[((Spi_DataParam *)chanHandle->activeIOP->addr)->dataFormat].wDelay;wDelayCnt++)
								;
							if (0 == instHandle->csHighPolarity)
							{
								/* Make the GPIO pin low									 */
								if ((/*instHandle->spiHWconfig.*/hwc->gpioChipselectFlag) && (/*instHandle->spiHWconfig.*/hwc->gpioPinPort))
									GPIO_WriteBit(/*instHandle->spiHWconfig.*/hwc->gpioPinPort,/*instHandle->spiHWconfig.*/hwc->gpioPinNo, Bit_RESET);
								else if (spiRegs==SPI1)
								{
									#ifdef USE_SPI1_GPIO
									GPIO_WriteBit(SPI1_NSS_GPIO_PORT,SPI1_NSS_PIN, Bit_RESET);
									#endif
								}
								else if (spiRegs==SPI2)
								{
									#ifdef USE_SPI2_GPIO
									GPIO_WriteBit(SPI2_NSS_GPIO_PORT,SPI2_NSS_PIN, Bit_RESET);
									#endif
								}
								else if (spiRegs==SPI3)
								{
									#ifdef USE_SPI3_GPIO
									GPIO_WriteBit(SPI3_NSS_GPIO_PORT,SPI3_NSS_PIN, Bit_RESET);
									#endif
								}
							}
							else
							{
								/* Make the GPIO pin high									   */
								if ((/*instHandle->spiHWconfig.*/hwc->gpioChipselectFlag) && (/*instHandle->spiHWconfig.*/hwc->gpioPinPort))
									GPIO_WriteBit(/*instHandle->spiHWconfig.*/hwc->gpioPinPort,/*instHandle->spiHWconfig.*/hwc->gpioPinNo, Bit_SET);
								else if (spiRegs==SPI1)
								{
									#ifdef USE_SPI1_GPIO
									GPIO_WriteBit(SPI1_NSS_GPIO_PORT,SPI1_NSS_PIN, Bit_SET);
									#endif
								}
								else if (spiRegs==SPI2)
								{
									#ifdef USE_SPI2_GPIO
									GPIO_WriteBit(SPI2_NSS_GPIO_PORT,SPI2_NSS_PIN, Bit_SET);
									#endif
								}
								else if (spiRegs==SPI3)
								{
									#ifdef USE_SPI3_GPIO
									GPIO_WriteBit(SPI3_NSS_GPIO_PORT,SPI3_NSS_PIN, Bit_SET);
									#endif
								}
							}
						}

					
						/* Transmit user supplied data						  */
						SPI_I2S_SendData(spiRegs,(*chanHandle->currBuffer));
						chanHandle->currBuffer ++;
						instHandle->stats.txBytes++;
					}
				}
			}
		}
		else
		{
			completeIo|=spiCompleteIOInIsr(instHandle);
			return completeIo;
		}
	}

	if (SPI_I2S_GetITStatus(spiRegs,SPI_I2S_IT_OVR))
	{
		/* Receive Over run interrupt									  */
		chanHandle->currError = Spi_RECEIVE_OVERRUN_ERR;
		instHandle->stats.rxOverrunError++; 				   
	}

  	return completeIo;  
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
static int spiCompleteIOInIsr (Spi_Object *instHandle)
{
    register Spi_ChanObj		*chanHandle;
    register Spi_DataParam	*dataParam;
    register IODEV_Packet	*ioPacket;
	register SPI_TypeDef *spiRegs;
	int completeIo=0;
	#ifdef EXTENDED_CHIP_SELECT
	GPIO_TypeDef	*gpioPort;
	uint16_t		gpioPin;
	uint16_t			cnt;
	uint8_t				mask;
	uint8_t				tmpMask;
	register uint16_t	csMax;
	register uint16_t *extGpioPinNo;
	#endif
    
    assert(NULL != instHandle);

    chanHandle = instHandle->currentActiveChannel;
	ioPacket=chanHandle->activeIOP;
    dataParam  = (Spi_DataParam *)ioPacket->addr;
	spiRegs=instHandle->deviceInfo.baseAddress;

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
    if ((dataParam->flags & Spi_CSHOLD_FOR_MULTI_TRANSCEIVE) != Spi_CSHOLD_FOR_MULTI_TRANSCEIVE) 
    {

        if (0 == instHandle->csHighPolarity)
        {
			#ifdef EXTENDED_CHIP_SELECT
			if (instHandle->spiHWconfig.extGpiochipSelect)
			{
				tmpMask=instHandle->spiHWconfig.extGpiochipSelect->enableMask;
				csMax=sizeof(instHandle->spiHWconfig.extGpiochipSelect->extGpioPinNo)/sizeof(instHandle->spiHWconfig.extGpiochipSelect->extGpioPinNo[0]);
				extGpioPinNo=instHandle->spiHWconfig.extGpiochipSelect->extGpioPinNo;
				for (cnt=0,mask=1;cnt<csMax;mask<<=1, cnt++)
				{
					if (tmpMask & mask)
					{
		                /* Make the GPIO pin high                                      */
						gpioPort = instHandle->spiHWconfig.extGpiochipSelect->extGpioPinPort[cnt];
						gpioPin = extGpioPinNo[cnt];
						GPIO_WriteBit(gpioPort,gpioPin, Bit_SET);
					}	
				}
			}
			else
			#endif
			{
				/* Make the GPIO pin high									   */
	            if ((0 != instHandle->spiHWconfig.gpioChipselectFlag) && (instHandle->spiHWconfig.gpioPinPort))
					GPIO_WriteBit(instHandle->spiHWconfig.gpioPinPort,instHandle->spiHWconfig.gpioPinNo, Bit_SET);
				else if (spiRegs==SPI1)
				{
					#ifdef USE_SPI1_GPIO
					GPIO_WriteBit(SPI1_NSS_GPIO_PORT,SPI1_NSS_PIN, Bit_SET);
					#endif
				}
				else if (spiRegs==SPI2)
				{
					#ifdef USE_SPI2_GPIO
					GPIO_WriteBit(SPI2_NSS_GPIO_PORT,SPI2_NSS_PIN, Bit_SET);
					#endif
				}
				else if (spiRegs==SPI3)
				{
					#ifdef USE_SPI3_GPIO
					GPIO_WriteBit(SPI3_NSS_GPIO_PORT,SPI3_NSS_PIN, Bit_SET);
					#endif
				}
					
			}
        }
        else
        {
			#ifdef EXTENDED_CHIP_SELECT
			if (instHandle->spiHWconfig.extGpiochipSelect)
			{
				tmpMask=instHandle->spiHWconfig.extGpiochipSelect->enableMask;
				csMax=sizeof(instHandle->spiHWconfig.extGpiochipSelect->extGpioPinNo)/sizeof(instHandle->spiHWconfig.extGpiochipSelect->extGpioPinNo[0]);
				extGpioPinNo=instHandle->spiHWconfig.extGpiochipSelect->extGpioPinNo;
				for (cnt=0,mask=1;cnt<csMax;mask<<=1, cnt++)
				{
					if (tmpMask & mask)
					{
		                /* Make the GPIO pin low */
						gpioPort = instHandle->spiHWconfig.extGpiochipSelect->extGpioPinPort[cnt];
						gpioPin = extGpioPinNo[cnt];
						GPIO_WriteBit(gpioPort,gpioPin, Bit_RESET);
					}	
				}
			}
			else
			#endif
			{
				/* Make the GPIO pin low									 */
	            if ((0 != instHandle->spiHWconfig.gpioChipselectFlag) && (instHandle->spiHWconfig.gpioPinPort))
					GPIO_WriteBit(instHandle->spiHWconfig.gpioPinPort,instHandle->spiHWconfig.gpioPinNo, Bit_RESET);
				else if (spiRegs==SPI1)
				{
					#ifdef USE_SPI1_GPIO
					GPIO_WriteBit(SPI1_NSS_GPIO_PORT,SPI1_NSS_PIN, Bit_RESET);
					#endif
				}
				else if (spiRegs==SPI2)
				{
					#ifdef USE_SPI2_GPIO
					GPIO_WriteBit(SPI2_NSS_GPIO_PORT,SPI2_NSS_PIN, Bit_RESET);
					#endif
				}
				else if (spiRegs==SPI3)
				{
					#ifdef USE_SPI3_GPIO
					GPIO_WriteBit(SPI3_NSS_GPIO_PORT,SPI3_NSS_PIN, Bit_RESET);
					#endif
				}
			}
        }
    }

    /* Mask off interrupts                                                    */
	
	//SPI_I2S_ITConfig(spiRegs,SPI_I2S_IT_TXE,DISABLE);
	/* Disable Receive interrupt										  */
	SPI_I2S_ITConfig(spiRegs,SPI_I2S_IT_RXNE,DISABLE);
	SPI_I2S_ITConfig(spiRegs,SPI_I2S_IT_ERR,DISABLE);

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
    spiLocalGetNextChannel(instHandle, &(instHandle->currentActiveChannel));
   
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
            dataParam = (Spi_DataParam *)ioPacket->addr;

            
            spiLocalControlDataConfig(chanHandle,dataParam->chipSelect, dataParam->dataFormat,dataParam->flags);
            
            /* Set the current buffer params correctly                        */
            chanHandle->currError         = 0;
            chanHandle->currFlags         = dataParam->flags;
            chanHandle->transcieveFlags   = (NULL == dataParam->inBuffer)? 0 : 1; /* transflags */
            chanHandle->currBuffer        = dataParam->outBuffer;
            chanHandle->currBufferLen     = dataParam->bufLen;

            /*Consume any stale data                                          */
			
			while (SPI_I2S_GetFlagStatus(spiRegs,SPI_I2S_FLAG_RXNE))
			{
				/* Read before Write to remove any stale data			  */
				*chanHandle->transBuffer = (uint8_t)(SPI_I2S_ReceiveData(spiRegs));
			}

            if (instHandle->spiHWconfig.configDatafmt[dataParam->dataFormat].charLength > 8u)
                chanHandle->charLength16Bits = 1;

            if ((chanHandle->currBufferLen) > 0)
            {
            
                if (0 == chanHandle->charLength16Bits)
                {
					SPI_I2S_SendData(spiRegs,*chanHandle->currBuffer);
                    chanHandle->currBuffer++;
                }
                else
                {
					SPI_I2S_SendData(spiRegs,(*((uint16_t *)chanHandle->currBuffer)));
					chanHandle->currBuffer += 2;
                }
                

                /* Enabling receive and error interrupts                      */
				SPI_I2S_ITConfig(spiRegs,SPI_I2S_IT_RXNE,ENABLE);
				SPI_I2S_ITConfig(spiRegs,SPI_I2S_IT_OVR,ENABLE);
            }/**< if currBuffLen                                              */
        }
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
void spiLocalGetNextChannel(Spi_Object *instHandle, Spi_ChanObj **pChanHandle)
{
	#if (1<Spi_NUM_CHANS)
	register uint16_t counter;
	register uint16_t chanIndexWithMaxPri = Spi_NUM_CHANS;
	register uint16_t lastFoundMaxPri     = 0;

	assert(NULL != instHandle);

	for(counter=0; counter<Spi_NUM_CHANS; counter++)
	{
		if((Spi_DriverState_OPENED == instHandle->chanObj[counter].channelState) && (lastFoundMaxPri < instHandle->chanObj[counter].taskPriority))
		{
			if(0 == QUE_empty(&(instHandle->chanObj[counter].queuePendingList)))
			{
				lastFoundMaxPri = instHandle->chanObj[counter].taskPriority;
				chanIndexWithMaxPri = counter;
			}
		}
	}

	if (Spi_NUM_CHANS != chanIndexWithMaxPri)
	{
		*pChanHandle = &(instHandle->chanObj[chanIndexWithMaxPri]);
	}
	else
	{
		*pChanHandle = NULL;
	}
	#else
	register uint16_t chanIndexWithMaxPri = Spi_NUM_CHANS;
	register uint16_t lastFoundMaxPri     = 0;
	
	if((Spi_DriverState_OPENED == instHandle->chanObj[0].channelState) && (lastFoundMaxPri < instHandle->chanObj[0].taskPriority))
	{
		if(0 == QUE_empty(&(instHandle->chanObj[0].queuePendingList)))
		{
			lastFoundMaxPri = instHandle->chanObj[0].taskPriority;
			chanIndexWithMaxPri = 0;
		}
	}
	if (Spi_NUM_CHANS != chanIndexWithMaxPri)
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
static int spiSetupConfig(Spi_ChanObj *chanHandle)
{
	uint32_t apbclock = 0x00;
	RCC_ClocksTypeDef RCC_ClocksStatus;
	uint16_t prescale;
	int			status = IODEV_COMPLETED;
	//int			count = 0;
	Spi_Object	*instHandle = NULL;
	register SPI_TypeDef *spiRegs;
	SPI_InitTypeDef spiInitSt;
	GPIO_InitTypeDef gpioInitSt;

	assert(NULL != chanHandle);

	instHandle = chanHandle->instHandle;

	assert(NULL != instHandle);
	
	spiRegs=instHandle->deviceInfo.baseAddress;

	/* clock is enabled in master mode only                               */
	if (Spi_CommMode_MASTER == instHandle->spiHWconfig.masterOrSlave)
	{
		/* Prescale value calculation                                     */
		
		RCC_GetClocksFreq(&RCC_ClocksStatus);
		
		if ((spiRegs == SPI2) || (spiRegs == SPI3))
		{
		  apbclock = RCC_ClocksStatus.PCLK1_Frequency;
		}
		else
		{
		  apbclock = RCC_ClocksStatus.PCLK2_Frequency;
		}
		prescale = spiPrescaler(apbclock,chanHandle->busFreq);

		/* prescalar is a 8 bit register and can hold maximum 255         */
		if (!IS_SPI_BAUDRATE_PRESCALER(prescale) )
		{
			status = IODEV_EBADARGS;
		}
	}

	if (IODEV_COMPLETED == status)
	{
		/*Put the module in reset mode                                    */
		/* Bring module out of reset                                      */
		SPI_I2S_DeInit(spiRegs);		

		//SPI_StructInit(&spiInitSt);
		/* Initialize the SPI_Direction member */
		spiInitSt.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
		/* initialize the SPI_Mode member */
		spiInitSt.SPI_Mode = (Spi_CommMode_MASTER == instHandle->spiHWconfig.masterOrSlave) ? SPI_Mode_Master: SPI_Mode_Slave;
		/* initialize the SPI_DataSize member */
		spiInitSt.SPI_DataSize = (instHandle->spiHWconfig.configDatafmt[0].charLength<=8) ? SPI_DataSize_8b : SPI_DataSize_16b;
		/* Initialize the SPI_CPOL member */
		spiInitSt.SPI_CPOL = (instHandle->spiHWconfig.configDatafmt[0].clkHigh)? SPI_CPOL_High: SPI_CPOL_Low;
		/* Initialize the SPI_CPHA member */
		spiInitSt.SPI_CPHA = (instHandle->spiHWconfig.configDatafmt[0].phaseIn)? SPI_CPHA_2Edge : SPI_CPHA_1Edge;
		/* Initialize the SPI_NSS member */
		
		//spiInitSt.SPI_NSS = SPI_NSS_Soft;
		spiInitSt.SPI_NSS = (Spi_CommMode_MASTER == instHandle->spiHWconfig.masterOrSlave) ? SPI_NSS_Soft: (instHandle->spiHWconfig.pinOpModes==Spi_PinOpMode_3PIN) ?SPI_NSS_Soft : SPI_NSS_Hard;
		/* Initialize the SPI_BaudRatePrescaler member */
		spiInitSt.SPI_BaudRatePrescaler = spiPrescaler(instHandle->deviceInfo.inputFrequency,chanHandle->busFreq);
		/* Initialize the SPI_FirstBit member */
		spiInitSt.SPI_FirstBit = (instHandle->spiHWconfig.configDatafmt[0].lsbFirst)? SPI_FirstBit_LSB : SPI_FirstBit_MSB;
		/* Initialize the SPI_CRCPolynomial member */
		spiInitSt.SPI_CRCPolynomial = 7;

		SPI_Init(spiRegs, &spiInitSt);

		
		if (Spi_CommMode_MASTER == instHandle->spiHWconfig.masterOrSlave) 
			SPI_NSSInternalSoftwareConfig(spiRegs, SPI_NSSInternalSoft_Set);
		else if (instHandle->spiHWconfig.pinOpModes==Spi_PinOpMode_3PIN)
			SPI_NSSInternalSoftwareConfig(spiRegs, SPI_NSSInternalSoft_Set);


		/* Modes of operation                                             */
		switch (instHandle->spiHWconfig.pinOpModes)
		{
		 case Spi_PinOpMode_5PIN:
			/**< 5-pin configuration(DI, DO, CLK, CS & ENA pins are used) */
			/* 4-pin CS configuration (DI, DO, CLK & ENA pins are used)	 */
		 case Spi_PinOpMode_SPIENA_4PIN:
			/* 4-pin CS configuration (DI, DO, CLK & CS pins are used)	 */
			SPI_SSOutputCmd(spiRegs,DISABLE);
			if (spiRegs==SPI1)
			{
				#ifdef USE_SPI1_GPIO
				GPIO_StructInit(&gpioInitSt);
				gpioInitSt.GPIO_Pin=SPI1_MISO_PIN;
				gpioInitSt.GPIO_Mode=GPIO_Mode_AF;
				gpioInitSt.GPIO_Speed=GPIO_Speed_100MHz;
				GPIO_Init(SPI1_MISO_GPIO_PORT,&gpioInitSt);
				GPIO_PinAFConfig(SPI1_MISO_GPIO_PORT,SPI1_MISO_PIN_SOURCE,GPIO_AF_SPI1);

				gpioInitSt.GPIO_Pin=SPI1_MOSI_PIN;
				GPIO_Init(SPI1_MOSI_GPIO_PORT,&gpioInitSt);
				GPIO_PinAFConfig(SPI1_MOSI_GPIO_PORT,SPI1_MOSI_PIN_SOURCE,GPIO_AF_SPI1);

				gpioInitSt.GPIO_Pin=SPI1_CLK_PIN;
				gpioInitSt.GPIO_PuPd= (instHandle->spiHWconfig.configDatafmt[0].clkHigh) ? GPIO_PuPd_UP : GPIO_PuPd_DOWN;
				GPIO_Init(SPI1_CLK_GPIO_PORT,&gpioInitSt);
				GPIO_PinAFConfig(SPI1_CLK_GPIO_PORT,SPI1_CLK_PIN_SOURCE,GPIO_AF_SPI1);

				if ((0==instHandle->spiHWconfig.gpioChipselectFlag) ||(Spi_CommMode_SLAVE== instHandle->spiHWconfig.masterOrSlave))
				{
					gpioInitSt.GPIO_Pin=SPI1_NSS_PIN;
					GPIO_Init(SPI1_NSS_GPIO_PORT,&gpioInitSt);
					GPIO_PinAFConfig(SPI1_NSS_GPIO_PORT,SPI1_NSS_PIN_SOURCE,GPIO_AF_SPI1);
				}
				#endif
			}
			else if (spiRegs==SPI2)
			{
				#ifdef USE_SPI2_GPIO
				GPIO_StructInit(&gpioInitSt);
				gpioInitSt.GPIO_Pin=SPI2_MISO_PIN;
				gpioInitSt.GPIO_Mode=GPIO_Mode_AF;
				gpioInitSt.GPIO_Speed=GPIO_Speed_100MHz;
				GPIO_Init(SPI2_MISO_GPIO_PORT,&gpioInitSt);
				GPIO_PinAFConfig(SPI2_MISO_GPIO_PORT,SPI2_MISO_PIN_SOURCE,GPIO_AF_SPI2);

				gpioInitSt.GPIO_Pin=SPI2_MOSI_PIN;
				GPIO_Init(SPI2_MOSI_GPIO_PORT,&gpioInitSt);
				GPIO_PinAFConfig(SPI2_MOSI_GPIO_PORT,SPI2_MOSI_PIN_SOURCE,GPIO_AF_SPI2);

				gpioInitSt.GPIO_Pin=SPI2_CLK_PIN;
				gpioInitSt.GPIO_PuPd= (instHandle->spiHWconfig.configDatafmt[0].clkHigh) ? GPIO_PuPd_UP : GPIO_PuPd_DOWN;
				GPIO_Init(SPI2_CLK_GPIO_PORT,&gpioInitSt);
				GPIO_PinAFConfig(SPI2_CLK_GPIO_PORT,SPI2_CLK_PIN_SOURCE,GPIO_AF_SPI2);
				if ((0==instHandle->spiHWconfig.gpioChipselectFlag) ||(Spi_CommMode_SLAVE== instHandle->spiHWconfig.masterOrSlave))
				{
					gpioInitSt.GPIO_Pin=SPI2_NSS_PIN;
					GPIO_Init(SPI2_NSS_GPIO_PORT,&gpioInitSt);
					GPIO_PinAFConfig(SPI2_NSS_GPIO_PORT,SPI2_NSS_PIN_SOURCE, GPIO_AF_SPI2);
				}
				#endif
			}
			else if (spiRegs==SPI3)
			{
				#ifdef USE_SPI3_GPIO
				GPIO_StructInit(&gpioInitSt);
				gpioInitSt.GPIO_Pin=SPI3_MISO_PIN;
				gpioInitSt.GPIO_Mode=GPIO_Mode_AF;
				gpioInitSt.GPIO_Speed=GPIO_Speed_100MHz;
				//gpioInitSt.GPIO_PuPd=GPIO_PuPd_UP; // *****
				GPIO_Init(SPI3_MISO_GPIO_PORT,&gpioInitSt);
				GPIO_PinAFConfig(SPI3_MISO_GPIO_PORT,SPI3_MISO_PIN_SOURCE,GPIO_AF_SPI3);

				gpioInitSt.GPIO_Pin=SPI3_MOSI_PIN;
				//gpioInitSt.GPIO_PuPd=GPIO_PuPd_NOPULL; // *****
				GPIO_Init(SPI3_MOSI_GPIO_PORT,&gpioInitSt);
				GPIO_PinAFConfig(SPI3_MOSI_GPIO_PORT,SPI3_MOSI_PIN_SOURCE,GPIO_AF_SPI3);

				gpioInitSt.GPIO_Pin=SPI3_CLK_PIN;
				gpioInitSt.GPIO_PuPd= (instHandle->spiHWconfig.configDatafmt[0].clkHigh) ? GPIO_PuPd_UP : GPIO_PuPd_DOWN;
				GPIO_Init(SPI3_CLK_GPIO_PORT,&gpioInitSt);
				GPIO_PinAFConfig(SPI3_CLK_GPIO_PORT,SPI3_CLK_PIN_SOURCE,GPIO_AF_SPI3);
				
				if ((0==instHandle->spiHWconfig.gpioChipselectFlag) ||(Spi_CommMode_SLAVE== instHandle->spiHWconfig.masterOrSlave))
				{
					gpioInitSt.GPIO_Pin=SPI3_NSS_PIN;
					GPIO_Init(SPI3_NSS_GPIO_PORT,&gpioInitSt);
					GPIO_PinAFConfig(SPI3_NSS_GPIO_PORT,SPI3_NSS_PIN_SOURCE, GPIO_AF_SPI2);
				}
				#endif
			}
			spiConfigureOpMode(instHandle);
			break ;
			 
		 case Spi_PinOpMode_SPISCS_4PIN:
			//SPI_SSOutputCmd(spiRegs,ENABLE);
			if (spiRegs==SPI1)
			{
				#ifdef USE_SPI1_GPIO
				GPIO_StructInit(&gpioInitSt);
				gpioInitSt.GPIO_Pin=SPI1_MISO_PIN;
				gpioInitSt.GPIO_Mode=GPIO_Mode_AF;
				gpioInitSt.GPIO_Speed=GPIO_Speed_100MHz;
				GPIO_Init(SPI1_MISO_GPIO_PORT,&gpioInitSt);
				GPIO_PinAFConfig(SPI1_MISO_GPIO_PORT,SPI1_MISO_PIN_SOURCE,GPIO_AF_SPI1);

				gpioInitSt.GPIO_Pin=SPI1_MOSI_PIN;
				GPIO_Init(SPI1_MOSI_GPIO_PORT,&gpioInitSt);
				GPIO_PinAFConfig(SPI1_MOSI_GPIO_PORT,SPI1_MOSI_PIN_SOURCE,GPIO_AF_SPI1);

				gpioInitSt.GPIO_Pin=SPI1_CLK_PIN;
				GPIO_Init(SPI1_CLK_GPIO_PORT,&gpioInitSt);
				GPIO_PinAFConfig(SPI1_CLK_GPIO_PORT,SPI1_CLK_PIN_SOURCE,GPIO_AF_SPI1);
				if (Spi_CommMode_SLAVE== instHandle->spiHWconfig.masterOrSlave)
				{
					gpioInitSt.GPIO_Pin=SPI1_NSS_PIN;
					GPIO_Init(SPI1_NSS_GPIO_PORT,&gpioInitSt);
					GPIO_PinAFConfig(SPI1_NSS_GPIO_PORT,SPI1_NSS_PIN_SOURCE,GPIO_AF_SPI1);
				}
				else if ((0!=instHandle->spiHWconfig.gpioChipselectFlag) && (instHandle->spiHWconfig.gpioPinPort!=NULL))
				{
					//GPIO_StructInit(&gpioInitSt);
					gpioInitSt.GPIO_Pin=instHandle->spiHWconfig.gpioPinNo;
					gpioInitSt.GPIO_Mode=GPIO_Mode_OUT;
					gpioInitSt.GPIO_Speed=GPIO_Speed_100MHz;
					gpioInitSt.GPIO_OType=GPIO_OType_PP;
					gpioInitSt.GPIO_PuPd=GPIO_PuPd_NOPULL;
					
					GPIO_Init(instHandle->spiHWconfig.gpioPinPort,&gpioInitSt);
					GPIO_WriteBit(instHandle->spiHWconfig.gpioPinPort,instHandle->spiHWconfig.gpioPinNo, (0 == instHandle->csHighPolarity) ? Bit_SET : Bit_RESET); /* Make the GPIO pin high */
				}
				else
				{
					GPIO_Init(SPI1_NSS_GPIO_PORT,(GPIO_InitTypeDef *)&gpioSpi1NssOutInit);
					GPIO_WriteBit(SPI1_NSS_GPIO_PORT,SPI1_NSS_PIN, (0 == instHandle->csHighPolarity) ? Bit_SET : Bit_RESET); /* Make the GPIO pin high */
				}
				#endif
			}
			else if (spiRegs==SPI2)
			{
				#ifdef USE_SPI2_GPIO
				GPIO_StructInit(&gpioInitSt);
				gpioInitSt.GPIO_Pin=SPI2_MISO_PIN;
				gpioInitSt.GPIO_Mode=GPIO_Mode_AF;
				gpioInitSt.GPIO_Speed=GPIO_Speed_100MHz;
				GPIO_Init(SPI2_MISO_GPIO_PORT,&gpioInitSt);
				GPIO_PinAFConfig(SPI2_MISO_GPIO_PORT,SPI2_MISO_PIN_SOURCE,GPIO_AF_SPI2);

				gpioInitSt.GPIO_Pin=SPI2_MOSI_PIN;
				GPIO_Init(SPI2_MOSI_GPIO_PORT,&gpioInitSt);
				GPIO_PinAFConfig(SPI2_MOSI_GPIO_PORT,SPI2_MOSI_PIN_SOURCE,GPIO_AF_SPI2);

				gpioInitSt.GPIO_Pin=SPI2_CLK_PIN;
				GPIO_Init(SPI2_CLK_GPIO_PORT,&gpioInitSt);
				GPIO_PinAFConfig(SPI2_CLK_GPIO_PORT,SPI2_CLK_PIN_SOURCE,GPIO_AF_SPI2);
				
				if (Spi_CommMode_SLAVE== instHandle->spiHWconfig.masterOrSlave)
				{
					gpioInitSt.GPIO_Pin=SPI2_NSS_PIN;
					GPIO_Init(SPI2_NSS_GPIO_PORT,&gpioInitSt);
					GPIO_PinAFConfig(SPI2_NSS_GPIO_PORT,SPI2_NSS_PIN_SOURCE,GPIO_AF_SPI2);
				}
				else if ((0!=instHandle->spiHWconfig.gpioChipselectFlag) && (instHandle->spiHWconfig.gpioPinPort!=NULL))
				{
					//GPIO_StructInit(&gpioInitSt);
					gpioInitSt.GPIO_Pin=instHandle->spiHWconfig.gpioPinNo;
					gpioInitSt.GPIO_Mode=GPIO_Mode_OUT;
					gpioInitSt.GPIO_Speed=GPIO_Speed_100MHz;
					gpioInitSt.GPIO_OType=GPIO_OType_PP;
					gpioInitSt.GPIO_PuPd=GPIO_PuPd_NOPULL;
					
					GPIO_Init(instHandle->spiHWconfig.gpioPinPort,&gpioInitSt);
					GPIO_WriteBit(instHandle->spiHWconfig.gpioPinPort,instHandle->spiHWconfig.gpioPinNo, (0 == instHandle->csHighPolarity) ? Bit_SET : Bit_RESET); /* Make the GPIO pin high */
				}
				else
				{
					GPIO_Init(SPI2_NSS_GPIO_PORT,(GPIO_InitTypeDef *)&gpioSpi2NssOutInit);
					GPIO_WriteBit(SPI2_NSS_GPIO_PORT,SPI2_NSS_PIN, (0 == instHandle->csHighPolarity) ? Bit_SET : Bit_RESET); /* Make the GPIO pin high */
				}
				#endif
			}
			else if (spiRegs==SPI3)
			{
				#ifdef USE_SPI3_GPIO
				GPIO_StructInit(&gpioInitSt);
				gpioInitSt.GPIO_Pin=SPI3_MISO_PIN;
				gpioInitSt.GPIO_Mode=GPIO_Mode_AF;
				gpioInitSt.GPIO_Speed=GPIO_Speed_100MHz;
				GPIO_Init(SPI3_MISO_GPIO_PORT,&gpioInitSt);
				GPIO_PinAFConfig(SPI3_MISO_GPIO_PORT,SPI3_MISO_PIN_SOURCE,GPIO_AF_SPI3);

				gpioInitSt.GPIO_Pin=SPI3_MOSI_PIN;
				GPIO_Init(SPI3_MOSI_GPIO_PORT,&gpioInitSt);
				GPIO_PinAFConfig(SPI3_MOSI_GPIO_PORT,SPI3_MOSI_PIN_SOURCE,GPIO_AF_SPI3);

				gpioInitSt.GPIO_Pin=SPI3_CLK_PIN;
				GPIO_Init(SPI3_CLK_GPIO_PORT,&gpioInitSt);
				GPIO_PinAFConfig(SPI3_CLK_GPIO_PORT,SPI3_CLK_PIN_SOURCE,GPIO_AF_SPI3);
				
				if (Spi_CommMode_SLAVE== instHandle->spiHWconfig.masterOrSlave)
				{
					gpioInitSt.GPIO_Pin=SPI3_NSS_PIN;
					GPIO_Init(SPI3_NSS_GPIO_PORT,&gpioInitSt);
					GPIO_PinAFConfig(SPI3_NSS_GPIO_PORT,SPI3_NSS_PIN_SOURCE,GPIO_AF_SPI2);
				}
				else if ((0!=instHandle->spiHWconfig.gpioChipselectFlag) && (instHandle->spiHWconfig.gpioPinPort!=NULL))
				{
					//GPIO_StructInit(&gpioInitSt);
					gpioInitSt.GPIO_Pin=instHandle->spiHWconfig.gpioPinNo;
					gpioInitSt.GPIO_Mode=GPIO_Mode_OUT;
					gpioInitSt.GPIO_Speed=GPIO_Speed_100MHz;
					gpioInitSt.GPIO_OType=GPIO_OType_PP;
					gpioInitSt.GPIO_PuPd=GPIO_PuPd_NOPULL;
					
					GPIO_Init(instHandle->spiHWconfig.gpioPinPort,&gpioInitSt);
					GPIO_WriteBit(instHandle->spiHWconfig.gpioPinPort,instHandle->spiHWconfig.gpioPinNo, (0 == instHandle->csHighPolarity) ? Bit_SET : Bit_RESET); /* Make the GPIO pin high */
				}
				else
				{
					GPIO_Init(SPI3_NSS_GPIO_PORT,(GPIO_InitTypeDef *)&gpioSpi3NssOutInit);
					GPIO_WriteBit(SPI3_NSS_GPIO_PORT,SPI3_NSS_PIN, (0 == instHandle->csHighPolarity) ? Bit_SET : Bit_RESET); /* Make the GPIO pin high */
				}
				#endif
			}
			spiConfigureOpMode(instHandle);
			break ;
			 
			 
			/* 3-pin configuration (DI, DO, CLK pins are used)            */
		 case Spi_PinOpMode_3PIN:
			if (spiRegs==SPI1)
			{
				#ifdef USE_SPI1_GPIO
				GPIO_StructInit(&gpioInitSt);
				gpioInitSt.GPIO_Pin=SPI1_MISO_PIN;
				gpioInitSt.GPIO_Mode=GPIO_Mode_AF;
				gpioInitSt.GPIO_Speed=GPIO_Speed_100MHz;
				GPIO_Init(SPI1_MISO_GPIO_PORT,&gpioInitSt);
				GPIO_PinAFConfig(SPI1_MISO_GPIO_PORT,SPI1_MISO_PIN_SOURCE,GPIO_AF_SPI1);
				
				gpioInitSt.GPIO_Pin=SPI1_MOSI_PIN;
				GPIO_Init(SPI1_MOSI_GPIO_PORT,&gpioInitSt);
				GPIO_PinAFConfig(SPI1_MOSI_GPIO_PORT,SPI1_MOSI_PIN_SOURCE,GPIO_AF_SPI1);
				
				gpioInitSt.GPIO_Pin=SPI1_CLK_PIN;
				GPIO_Init(SPI1_CLK_GPIO_PORT,&gpioInitSt);
				GPIO_PinAFConfig(SPI1_CLK_GPIO_PORT,SPI1_CLK_PIN_SOURCE,GPIO_AF_SPI1);
				#endif
			}
			else if (spiRegs==SPI2)
			{
				#ifdef USE_SPI2_GPIO
				GPIO_StructInit(&gpioInitSt);
				gpioInitSt.GPIO_Pin=SPI2_MISO_PIN;
				gpioInitSt.GPIO_Mode=GPIO_Mode_AF;
				gpioInitSt.GPIO_Speed=GPIO_Speed_100MHz;
				GPIO_Init(SPI2_MISO_GPIO_PORT,&gpioInitSt);
				GPIO_PinAFConfig(SPI2_MISO_GPIO_PORT,SPI2_MISO_PIN_SOURCE,GPIO_AF_SPI2);
				
				gpioInitSt.GPIO_Pin=SPI2_MOSI_PIN;
				GPIO_Init(SPI2_MOSI_GPIO_PORT,&gpioInitSt);
				GPIO_PinAFConfig(SPI2_MOSI_GPIO_PORT,SPI2_MOSI_PIN_SOURCE,GPIO_AF_SPI2);
				
				gpioInitSt.GPIO_Pin=SPI2_CLK_PIN;
				GPIO_Init(SPI2_CLK_GPIO_PORT,&gpioInitSt);
				GPIO_PinAFConfig(SPI2_CLK_GPIO_PORT,SPI2_CLK_PIN_SOURCE,GPIO_AF_SPI2);
				#endif
			}
			else if (spiRegs==SPI3)
			{
				#ifdef USE_SPI3_GPIO
				GPIO_StructInit(&gpioInitSt);
				gpioInitSt.GPIO_Pin=SPI3_MISO_PIN;
				gpioInitSt.GPIO_Mode=GPIO_Mode_AF;
				gpioInitSt.GPIO_Speed=GPIO_Speed_100MHz;
				GPIO_Init(SPI3_MISO_GPIO_PORT,&gpioInitSt);
				GPIO_PinAFConfig(SPI3_MISO_GPIO_PORT,SPI3_MISO_PIN_SOURCE,GPIO_AF_SPI3);
				
				gpioInitSt.GPIO_Pin=SPI3_MOSI_PIN;
				GPIO_Init(SPI3_MOSI_GPIO_PORT,&gpioInitSt);
				GPIO_PinAFConfig(SPI3_MOSI_GPIO_PORT,SPI3_MOSI_PIN_SOURCE,GPIO_AF_SPI3);
				
				gpioInitSt.GPIO_Pin=SPI3_CLK_PIN;
				GPIO_Init(SPI3_CLK_GPIO_PORT,&gpioInitSt);
				GPIO_PinAFConfig(SPI3_CLK_GPIO_PORT,SPI3_CLK_PIN_SOURCE,GPIO_AF_SPI3);
				#endif
			}
			spiConfigureOpMode(instHandle);
			break ;

			default:
			status = IODEV_EBADARGS;
		}
		
		/* Enable SPI for further transaction                             */
		SPI_Cmd(spiRegs, ENABLE);
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
static int spiSetupClock(Spi_ChanObj *chanHandle)
{
	uint32_t apbclock = 0x00;
	RCC_ClocksTypeDef RCC_ClocksStatus;
	uint16_t		prescale = 0;
	int             status   = IODEV_COMPLETED;
	//int             count    = 0;
	Spi_Object		*instHandle = NULL;
	register SPI_TypeDef *spiRegs;

	assert(NULL != chanHandle);

	instHandle = chanHandle->instHandle;

	assert(NULL != instHandle);

	spiRegs=instHandle->deviceInfo.baseAddress;
	/* clock is enabled in master mode only                               */
	if (Spi_CommMode_MASTER == instHandle->spiHWconfig.masterOrSlave)
	{
		/* Prescale value calculation                                     */
		RCC_GetClocksFreq(&RCC_ClocksStatus);
		
		if ((spiRegs == SPI2) || (spiRegs == SPI3))
		{
		  apbclock = RCC_ClocksStatus.PCLK1_Frequency;
		}
		else
		{
		  apbclock = RCC_ClocksStatus.PCLK2_Frequency;
		}
		prescale = spiPrescaler(apbclock,chanHandle->busFreq);

		/* prescalar is a 8 bit register and can hold maximum 255         */
		if (!IS_SPI_BAUDRATE_PRESCALER(prescale) )
		{
			status = IODEV_EBADARGS;
		}
	}


	if (IODEV_COMPLETED == status)
	{
		spiRegs->CR1=(spiRegs->CR1 & ~SPI_BaudRatePrescaler_256) | prescale;
	}
	return (status); /* success  or error code                                */
}


static uint16_t spiPrescaler(uint32_t inFreq, uint32_t busFreq)
{
	uint32_t tmp;

	assert(busFreq!=0);

	tmp= inFreq/busFreq;

	if (tmp<=2)
		return SPI_BaudRatePrescaler_2;
	else if (tmp<=4)
		return SPI_BaudRatePrescaler_4;
	else if (tmp<=8)
		return SPI_BaudRatePrescaler_8;
	else if (tmp<=16)
		return SPI_BaudRatePrescaler_16;
	else if (tmp<=32)
		return SPI_BaudRatePrescaler_32;
	else if (tmp<=64)
		return SPI_BaudRatePrescaler_64;
	else if (tmp<=128)
		return SPI_BaudRatePrescaler_128;
	else if (tmp<=256)
		return SPI_BaudRatePrescaler_256;
	else 
		return SPI_BaudRatePrescaler_256;
		
}


/**
 *  \brief  Configure the chipselect pin so it will act as per spi protocol
 *         to select the one of slaves.It will also configure the delay
 *         register of following type of delay.
 *
 *         The Chip-select-active-to-ENA-signal-active-time-out
 *         The Transmit-data-finished-to-ENA-pin-inactive-time-out
 *         The Transmit-end-to-chip-select-inactive-delay
 *         The Chip-select-active-to-transmit-start-delay
 *
 *  \param  Spihandle   [IN]    SPI driver object for respective instance.
 *
 */
static void spiConfigureOpMode(const Spi_Object *instHandle)
{
}



/**
 *  \brief  This to configure the delay for stale sdata
 *
 *  \Param  None
 *
 *  \return None
 *
 */
void spiWaitForStaleData(void)
{
    volatile uint32_t count = 0x2FFFu;
    
    /* Waits for some period of time to stale the data                        */
    while (0 != count)
    {
        count--;
    }
}


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
static int spiCheckTimeOut(portTickType startValue,portTickType timeout)
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


#ifdef Spi_DMA_ENABLE
#define DISCRETE_DMA_STRUCT_INIT

int spiLocalDmaTransfer(void *handle, uint32_t chipSelect, Spi_DataFormat dataFormat, uint32_t flags)
{
    uint8_t *pTransBuffer/*   = NULL*/;
    uint8_t *pCurrBuffer/*   = NULL*/;
	//register SPI_TypeDef *spiRegs=NULL;
    //int                   dstIndex     = 0;
    //int                   srcIndex     = 0;
    Spi_Object           *instHandle/*   = NULL*/;
    Spi_ChanObj          *chanHandle/*   = NULL*/;
    int                 retVal       = IODEV_COMPLETED;
    //uint16_t                acntFlag     = 1;
    Spi_DataParam        *dataParam/*   = NULL*/;
	//uint16_t	i;

	DMA_Stream_TypeDef	*hDmaTxStream/*   = NULL*/;
	DMA_Stream_TypeDef	*hDmaRxStream/*   = NULL*/;
	DMA_InitTypeDef dmaTxInit;
	DMA_InitTypeDef dmaRxInit;
	//struct sDmaResSet *hDmaResSet=NULL;
	//DMA_RESOURCE *hDmaRes=NULL;
	struct sDmaParam *hDmaTxParam/*   = NULL*/;
	struct sDmaParam *hDmaRxParam/*   = NULL*/;
	//uint32_t key;

    assert(NULL != handle);

    instHandle = (Spi_Object *)handle ;

    chanHandle = instHandle->currentActiveChannel;
    dataParam  = (Spi_DataParam *)chanHandle->activeIOP->addr;

	if (instHandle->spiHWconfig.configDatafmt[dataFormat].charLength > 8u)
		chanHandle->charLength16Bits = 1;
	else
		chanHandle->charLength16Bits = 0;
		
	

    /* Buffer assingment to local pointer is done to perform transcieve       *
     * operation                                                              */
    pTransBuffer = (uint8_t *)chanHandle->transBuffer;
    pCurrBuffer = (uint8_t *)chanHandle->currBuffer;

	hDmaTxParam=&((struct sDmaResSet *)instHandle->hDma)->set[TX_DMA];
	hDmaRxParam=&((struct sDmaResSet *)instHandle->hDma)->set[RX_DMA];
	hDmaTxStream=dmaStreamInfo.dmac[hDmaTxParam->dmacId].pStream[hDmaTxParam->stream];
	hDmaRxStream=dmaStreamInfo.dmac[hDmaRxParam->dmacId].pStream[hDmaRxParam->stream];

	/* setup rx transfer */

	#ifdef DISCRETE_DMA_STRUCT_INIT
	
	/*-------------- Reset DMA init structure parameters values ----------------*/
	/* Initialize the DMA_Channel member */
	//dmaRxInit.DMA_Channel = 0;
	
	/* Initialize the DMA_PeripheralBaseAddr member */
	//dmaRxInit.DMA_PeripheralBaseAddr = 0;
	
	/* Initialize the DMA_Memory0BaseAddr member */
	//dmaRxInit.DMA_Memory0BaseAddr = 0;
	
	/* Initialize the DMA_DIR member */
	//dmaRxInit.DMA_DIR = DMA_DIR_PeripheralToMemory;
	
	/* Initialize the DMA_BufferSize member */
	dmaRxInit.DMA_BufferSize = 0;
	
	/* Initialize the DMA_PeripheralInc member */
	dmaRxInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	
	/* Initialize the DMA_MemoryInc member */
	//dmaRxInit.DMA_MemoryInc = DMA_MemoryInc_Disable;
	
	/* Initialize the DMA_PeripheralDataSize member */
	dmaRxInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	
	/* Initialize the DMA_MemoryDataSize member */
	dmaRxInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	
	/* Initialize the DMA_Mode member */
	dmaRxInit.DMA_Mode = DMA_Mode_Normal;
	
	/* Initialize the DMA_Priority member */
	//dmaRxInit.DMA_Priority = DMA_Priority_Low;
	
	/* Initialize the DMA_FIFOMode member */
	dmaRxInit.DMA_FIFOMode = DMA_FIFOMode_Disable;
	
	/* Initialize the DMA_FIFOThreshold member */
	dmaRxInit.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	
	/* Initialize the DMA_MemoryBurst member */
	dmaRxInit.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	
	/* Initialize the DMA_PeripheralBurst member */
	dmaRxInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	#else
	DMA_StructInit(&dmaRxInit);
	#endif
	dmaRxInit.DMA_Channel = dmaChanTable[hDmaRxParam->channel];
	dmaRxInit.DMA_PeripheralBaseAddr = (uint32_t)&instHandle->deviceInfo.baseAddress->DR;
	dmaRxInit.DMA_Memory0BaseAddr = (uint32_t)pTransBuffer;
	dmaRxInit.DMA_DIR = DMA_DIR_PeripheralToMemory;
    /* transcieveFlags == True i.e transcieve operation so increment          *
     * the data buffer else use buffer as local and dumped the data           *
     * onto same location(dont increment buffer).                             */
	dmaRxInit.DMA_MemoryInc=(chanHandle->transcieveFlags) ? DMA_MemoryInc_Enable : DMA_MemoryInc_Disable;
	dmaRxInit.DMA_Priority=DMA_Priority_Low;

	if (chanHandle->charLength16Bits)
	{
		dmaRxInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
		dmaRxInit.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;
	}

	if (chanHandle->currBufferLen <= 0xFFFF)
		dmaRxInit.DMA_BufferSize = chanHandle->currBufferLen;
	else
	{
		retVal = IODEV_EBADIO;
	}

	/* setup tx transfer */
	#ifdef DISCRETE_DMA_STRUCT_INIT
	/*-------------- Reset DMA init structure parameters values ----------------*/
	/* Initialize the DMA_Channel member */
	//dmaTxInit.DMA_Channel = 0;
	
	/* Initialize the DMA_PeripheralBaseAddr member */
	//dmaTxInit.DMA_PeripheralBaseAddr = 0;
	
	/* Initialize the DMA_Memory0BaseAddr member */
	//dmaTxInit.DMA_Memory0BaseAddr = 0;
	
	/* Initialize the DMA_DIR member */
	//dmaTxInit.DMA_DIR = DMA_DIR_PeripheralToMemory;
	
	/* Initialize the DMA_BufferSize member */
	dmaTxInit.DMA_BufferSize = 0;
	
	/* Initialize the DMA_PeripheralInc member */
	dmaTxInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	
	/* Initialize the DMA_MemoryInc member */
	//dmaTxInit.DMA_MemoryInc = DMA_MemoryInc_Disable;
	
	/* Initialize the DMA_PeripheralDataSize member */
	dmaTxInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	
	/* Initialize the DMA_MemoryDataSize member */
	dmaTxInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	
	/* Initialize the DMA_Mode member */
	dmaTxInit.DMA_Mode = DMA_Mode_Normal;
	
	/* Initialize the DMA_Priority member */
	//dmaTxInit.DMA_Priority = DMA_Priority_Low;
	
	/* Initialize the DMA_FIFOMode member */
	dmaTxInit.DMA_FIFOMode = DMA_FIFOMode_Disable;
	
	/* Initialize the DMA_FIFOThreshold member */
	dmaTxInit.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	
	/* Initialize the DMA_MemoryBurst member */
	dmaTxInit.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	
	/* Initialize the DMA_PeripheralBurst member */
	dmaTxInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	#else
	DMA_StructInit(&dmaTxInit);
	#endif
	dmaTxInit.DMA_Channel =  dmaChanTable[hDmaTxParam->channel];
	dmaTxInit.DMA_PeripheralBaseAddr = (uint32_t)&instHandle->deviceInfo.baseAddress->DR;
	dmaTxInit.DMA_Memory0BaseAddr = (uint32_t)pCurrBuffer;
	dmaTxInit.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	dmaTxInit.DMA_MemoryInc=DMA_MemoryInc_Enable;
	dmaTxInit.DMA_Priority=DMA_Priority_Low;

	if (chanHandle->charLength16Bits)
	{
		dmaTxInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
		dmaTxInit.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;
	}
	if (chanHandle->currBufferLen <= 0xFFFF)
		dmaTxInit.DMA_BufferSize = chanHandle->currBufferLen;
	else
	{
		retVal = IODEV_EBADIO;
	}
	if (IODEV_COMPLETED == retVal)
	{
		DMA_Init(hDmaRxStream,&dmaRxInit);
		DMA_Cmd(hDmaRxStream,ENABLE);
		DMA_ITConfig(hDmaRxStream,DMA_IT_TC,ENABLE);
		DMA_Init(hDmaTxStream,&dmaTxInit);

		spiLocalControlDataConfig(chanHandle, dataParam->chipSelect, dataParam->dataFormat,dataParam->flags);
		
		DMA_Cmd(hDmaTxStream,ENABLE);
		DMA_ITConfig(hDmaTxStream,DMA_IT_TC,ENABLE);
		SPI_I2S_DMACmd(instHandle->deviceInfo.baseAddress,SPI_I2S_DMAReq_Rx,ENABLE);
		SPI_I2S_DMACmd(instHandle->deviceInfo.baseAddress,SPI_I2S_DMAReq_Tx,ENABLE);
	}
	

	return (retVal);
}

int spiLocalDmaChannel_Request(Spi_Object *instHandle)
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
		installInterruptHandler((uint16_t)nvicInit.NVIC_IRQChannel,stSpiDmaRxIsr,instHandle);
		instHandle->hRxDmaHandler=stSpiDmaRxIsr;
		instHandle->hRxDmaHandlerArg=instHandle;
		NVIC_Init(&nvicInit);
			

		/* install Tx DMA interrupt handler */
		nvicInit.NVIC_IRQChannel = dmaStreamInfo.dmac[hDmaTxParam->dmacId].nIrq[hDmaTxParam->stream];
		nvicInit.NVIC_IRQChannelPreemptionPriority = 15;
		nvicInit.NVIC_IRQChannelSubPriority = 1;
		nvicInit.NVIC_IRQChannelCmd = ENABLE;
		installInterruptHandler((uint16_t)nvicInit.NVIC_IRQChannel,stSpiDmaTxIsr,instHandle);
		instHandle->hTxDmaHandler=stSpiDmaTxIsr;
		instHandle->hTxDmaHandlerArg=instHandle;
		NVIC_Init(&nvicInit);
	}
	return (IODEV_COMPLETED);
}


#define TX_DMA_XFER_COMPLETE	0x04
#define RX_DMA_XFER_COMPLETE	0x08


int stSpiDmaTxIsr(Spi_Object *instHandle)
{
	Spi_ChanObj *chanHandle/*  = NULL*/;
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
		SPI_I2S_DMACmd(instHandle->deviceInfo.baseAddress,SPI_I2S_DMAReq_Tx,DISABLE);
		ioPacket = chanHandle->activeIOP;
		if (ioPacket!=NULL)
		{
			chanHandle->activeIOP->status|=TX_DMA_XFER_COMPLETE;
			bytesRemain=DMA_GetCurrDataCounter(pstream);
			instHandle->stats.txBytes += ioPacket->size-bytesRemain;
			if (chanHandle->activeIOP->status & RX_DMA_XFER_COMPLETE)
			{
				completeIo|=spiCompleteIOdmaCallback(instHandle);
			}
			
		}
	}

	return completeIo;
}

int stSpiDmaRxIsr(Spi_Object *instHandle)
{
	#ifdef USE_SPI_DRIVER_TIMESTAMPS
	TIMESTAMP ts=readTimestamp();
	TIMESTAMP *tsp;
	#endif
	Spi_ChanObj *chanHandle/*  = NULL*/;
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
		SPI_I2S_DMACmd(instHandle->deviceInfo.baseAddress,SPI_I2S_DMAReq_Rx,DISABLE);
		
		ioPacket = chanHandle->activeIOP;
		if (ioPacket!=NULL)
		{
			ioPacket->status|=RX_DMA_XFER_COMPLETE;
			bytesRemain=DMA_GetCurrDataCounter(pstream);
			instHandle->stats.rxBytes += ioPacket->size-bytesRemain;
			if (chanHandle->activeIOP->status & TX_DMA_XFER_COMPLETE)
			{
				ioPacket->status=IODEV_COMPLETED;
				completeIo|=spiCompleteIOdmaCallback(instHandle);
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
int spiCompleteIOdmaCallback (Spi_Object *instHandle)
{
    register Spi_ChanObj *chanHandle/* = NULL*/;
    register Spi_DataParam *dataParam/*  = NULL*/;
    int transFlags = 0;
    register IODEV_Packet *ioPacket/*   = NULL*/;
	register SPI_TypeDef *spiRegs;
	int completeIo=0;
	register Spi_HWConfigData *hwc;
	#ifdef EXTENDED_CHIP_SELECT
	register uint16_t	cnt;
	register uint8_t	mask;
	register uint8_t	tmpMask;
	register uint16_t	csMax;
	register uint16_t *extGpioPinNo;
	GPIO_TypeDef	*gpioPort;
	uint16_t		gpioPin;
	uint32_t		gpioClk;
	#endif
	
    assert(NULL != instHandle);

    chanHandle = instHandle->currentActiveChannel;
    dataParam  = (Spi_DataParam *)chanHandle->activeIOP->addr;
    spiRegs=instHandle->deviceInfo.baseAddress;
    hwc=&instHandle->spiHWconfig;
    /* check if CSHOLD needs to be enable after the transmission also.        *
     * If No or if there is any eror disable CSHOLD value, else do nothing    */

    if ((dataParam->flags & Spi_CSHOLD_FOR_MULTI_TRANSCEIVE) != Spi_CSHOLD_FOR_MULTI_TRANSCEIVE) 
    {

        if (0 == instHandle->csHighPolarity)
        {
			#ifdef EXTENDED_CHIP_SELECT
			if (/*instHandle->spiHWconfig.*/hwc->extGpiochipSelect)
			{
				tmpMask=/*instHandle->spiHWconfig.*/hwc->extGpiochipSelect->enableMask;
				csMax=sizeof(/*instHandle->spiHWconfig.*/hwc->extGpiochipSelect->extGpioPinNo)/sizeof(/*instHandle->spiHWconfig.*/hwc->extGpiochipSelect->extGpioPinNo[0]);
				extGpioPinNo=/*instHandle->spiHWconfig.*/hwc->extGpiochipSelect->extGpioPinNo;
				for (cnt=0,mask=1;cnt<csMax;mask<<=1, cnt++)
				{
					if (tmpMask & mask)
					{
		                /* Make the GPIO pin high                                      */
						gpioPort = /*instHandle->spiHWconfig.*/hwc->extGpiochipSelect->extGpioPinPort[cnt];
						gpioPin = extGpioPinNo[cnt];
						GPIO_WriteBit(gpioPort,gpioPin, Bit_SET);
					}	
				}
			}
			else
			#endif
			{
				/* Make the GPIO pin high									   */
	            if ((0 != /*instHandle->spiHWconfig.*/hwc->gpioChipselectFlag) && (/*instHandle->spiHWconfig.*/hwc->gpioPinPort))
					GPIO_WriteBit(instHandle->spiHWconfig.gpioPinPort,instHandle->spiHWconfig.gpioPinNo, Bit_SET);
				else if (spiRegs==SPI1)
				{
					#ifdef USE_SPI1_GPIO
					GPIO_WriteBit(SPI1_NSS_GPIO_PORT,SPI1_NSS_PIN, Bit_SET);
					#endif
				}
				else if (spiRegs==SPI2)
				{
					#ifdef USE_SPI2_GPIO
					GPIO_WriteBit(SPI2_NSS_GPIO_PORT,SPI2_NSS_PIN, Bit_SET);
					#endif
				}
				else if (spiRegs==SPI3)
				{
					#ifdef USE_SPI3_GPIO
					GPIO_WriteBit(SPI3_NSS_GPIO_PORT,SPI3_NSS_PIN, Bit_SET);
					#endif
				}
					
			}
        }
        else
        {
			#ifdef EXTENDED_CHIP_SELECT
			if (/*instHandle->spiHWconfig.*/hwc->extGpiochipSelect)
			{
				tmpMask=/*instHandle->spiHWconfig.*/hwc->extGpiochipSelect->enableMask;
				csMax=sizeof(/*instHandle->spiHWconfig.*/hwc->extGpiochipSelect->extGpioPinNo)/sizeof(/*instHandle->spiHWconfig.*/hwc->extGpiochipSelect->extGpioPinNo[0]);
				extGpioPinNo=/*instHandle->spiHWconfig.*/hwc->extGpiochipSelect->extGpioPinNo;
				for (cnt=0,mask=1;cnt<csMax;mask<<=1, cnt++)
				{
					if (tmpMask & mask)
					{
		                /* Make the GPIO pin low */
						gpioPort = /*instHandle->spiHWconfig.*/hwc->extGpiochipSelect->extGpioPinPort[cnt];
						gpioPin = extGpioPinNo[cnt];
						GPIO_WriteBit(gpioPort,gpioPin, Bit_RESET);
					}	
				}
			}
			else
			#endif
			{
				/* Make the GPIO pin low									 */
	            if ((0 != /*instHandle->spiHWconfig.*/hwc->gpioChipselectFlag) || (/*instHandle->spiHWconfig.*/hwc->gpioPinPort))
					GPIO_WriteBit(/*instHandle->spiHWconfig.*/hwc->gpioPinPort,/*instHandle->spiHWconfig.*/hwc->gpioPinNo, Bit_RESET);
				else if (spiRegs==SPI1)
				{
					#ifdef USE_SPI1_GPIO
					GPIO_WriteBit(SPI1_NSS_GPIO_PORT,SPI1_NSS_PIN, Bit_RESET);
					#endif
				}
				else if (spiRegs==SPI2)
				{
					#ifdef USE_SPI2_GPIO
					GPIO_WriteBit(SPI2_NSS_GPIO_PORT,SPI2_NSS_PIN, Bit_RESET);
					#endif
				}
				else if (spiRegs==SPI3)
				{
					#ifdef USE_SPI3_GPIO
					GPIO_WriteBit(SPI3_NSS_GPIO_PORT,SPI3_NSS_PIN, Bit_RESET);
					#endif
				}
					
			}
        }
    }

    /* Mask off interrupts                                                    */
	//SPI_I2S_ITConfig(spiRegs,SPI_I2S_IT_TXE,DISABLE);
	SPI_I2S_ITConfig(spiRegs,SPI_I2S_IT_RXNE,DISABLE);
	SPI_I2S_ITConfig(spiRegs,SPI_I2S_IT_ERR,DISABLE);

    /* call the application completion callback function registered           *
     * with us during opening of the channel                                  */
    if (NULL != chanHandle->cbFxn)
    {
        /* Invoke Application callback for this channel                       */
        completeIo|=(*chanHandle->cbFxn)((void *)chanHandle->cbArg, chanHandle->activeIOP);
    }

    chanHandle->activeIOP = NULL;

    /*get the channel with highest priority                                   */
    spiLocalGetNextChannel(instHandle, &(instHandle->currentActiveChannel));

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
            dataParam = (Spi_DataParam *)ioPacket->addr;

            if (NULL == dataParam->inBuffer)
                transFlags = 0;
            else
                transFlags = 1;

            /* Mask all interrupts,& populate data to spi object structure    *
             * so driver can extact all information at dma configuartion      *
             * and dma Rx & Tx callback                                       */

            /* Transmit data                                                  */
			while (SPI_I2S_GetFlagStatus(spiRegs,SPI_I2S_FLAG_RXNE))
			{
				/* Read before Write to remove any stale data			  */
				*chanHandle->transBuffer = (uint8_t)(SPI_I2S_ReceiveData(spiRegs));
			}

			if (/*instHandle->spiHWconfig.*/hwc->configDatafmt[dataParam->dataFormat].charLength > 8u)
				chanHandle->charLength16Bits = 1;
			else
				chanHandle->charLength16Bits = 0;
				

            chanHandle->currError           = 0;
            chanHandle->currFlags           = dataParam->flags;
            chanHandle->transcieveFlags     = transFlags;
            chanHandle->currBuffer          = dataParam->outBuffer;
            chanHandle->currBufferLen       = dataParam->bufLen;

            /* Call this to complete edma configuration and transfer          */
            spiLocalDmaTransfer(instHandle,dataParam->chipSelect,
                    dataParam->dataFormat,dataParam->flags);
        }
    }
	return completeIo;
}


#endif
