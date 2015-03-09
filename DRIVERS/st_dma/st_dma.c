/**
* @file st_dma.c
* @brief dma manger
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 21.05.2012
*/

#include <stddef.h>
#include <stdint.h>
#include <freertos.h>
#include <semphr.h>
#include <iodev.h>
#include "stm32f2xx.h"
#include "st_dma.h"



/**/

/* UART5 dma set */
const struct sDmaSet Uart5_Rx_DmaSet =
{
	NULL,	/* link */
	6,		/* stream */
	4		/* channel */
};

const struct sDmaSet Uart5_Tx_DmaSet =
{
	(struct sDmaSet *)&Uart5_Rx_DmaSet,	/* link */
	7,		/* stream */
	4		/* channel */
};
/**/



DMA_RESOURCE Uart5_DmaResource=
{
	NULL,		/**< sem */
	-1,		/** < Allocated resource identifier */
	{ 
		(struct sDmaSet *)&Uart5_Tx_DmaSet,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL
	},

};

/* USART2 dma set */
const struct sDmaSet Usart2_Rx_DmaSet =
{
	NULL,	/* link */
	5,		/* stream */
	4		/* channel */
};

const struct sDmaSet Usart2_Tx_DmaSet =
{
	(struct sDmaSet *)&Usart2_Rx_DmaSet,	/* link */
	6,		/* stream */
	4		/* channel */
};
/**/

DMA_RESOURCE Usart2_DmaResource=
{
	NULL,		/**< sem */
	-1,		/** < Allocated resource identifier */
	{ 
		(struct sDmaSet *)&Usart2_Tx_DmaSet,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL
	},

};

/* USART3 dma set */
const struct sDmaSet Usart3_Rx_DmaSet =
{
	NULL,	/* link */
	1,		/* stream */
	4		/* channel */
};

const struct sDmaSet Usart3_Tx_DmaSet =
{
	(struct sDmaSet *)&Usart3_Rx_DmaSet,	/* link */
	3,		/* stream */
	4		/* channel */
};


DMA_RESOURCE Usart3_DmaResource=
{
	NULL,		/**< sem */
	-1,		/** < Allocated resource identifier */
	{ 
		(struct sDmaSet *)&Usart3_Tx_DmaSet,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL
	},

};

/**/

/* UART4 dma set */
const struct sDmaSet Uart4_Rx_DmaSet =
{
	NULL,	/* link */
	2,		/* stream */
	4		/* channel */
};

const struct sDmaSet Uart4_Tx_DmaSet =
{
	(struct sDmaSet *)&Uart4_Rx_DmaSet,	/* link */
	4,		/* stream */
	4		/* channel */
};
/**/

DMA_RESOURCE Uart4_DmaResource=
{
	NULL,		/**< sem */
	-1,		/** < Allocated resource identifier */
	{ 
		(struct sDmaSet *)&Uart4_Tx_DmaSet,
			NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL
	},

};



/* SPI1 dma set */
const struct sDmaSet Spi1_Rx_DmaSet =
{
	NULL,	/* link */
	2,		/* stream */
	3		/* channel */
};

const struct sDmaSet Spi1_Tx_DmaSet =
{
	(struct sDmaSet *)&Spi1_Rx_DmaSet,	/* link */
	3,		/* stream */
	3		/* channel */
};
/**/


DMA_RESOURCE Spi1_DmaResource=
{
	NULL,		/* sem */
	-1,		/* Allocated resource identifier */
	{ 
		(struct sDmaSet *)&Spi1_Tx_DmaSet,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL
	},

};


/* USART1 dma set */
const struct sDmaSet Usart1_Rx_DmaSet =
{
	NULL,	/* link */
	5,		/* stream */
	4		/* channel */
};

const struct sDmaSet Usart1_Tx_DmaSet =
{
	(struct sDmaSet *)&Usart1_Rx_DmaSet,	/* link */
	7,		/* stream */
	4		/* channel */
};
/**/


/* USART6 dma set */
const struct sDmaSet Usart6_Rx_DmaSet =
{
	NULL,	/* link */
	1,		/* stream */
	5		/* channel */
};

const struct sDmaSet Usart6_Tx_DmaSet =
{
	(struct sDmaSet *)&Usart6_Rx_DmaSet,	/* link */
	6,		/* stream */
	5		/* channel */
};
/**/


/* Hash_in dma set */
const struct sDmaSet Hash_In_DmaSet =
{
	NULL,	/* link */
	7,		/* stream */
	2		/* channel */
};

/**/


DMA_RESOURCE Usart6_DmaResource=
{
	NULL,		/**< sem */
	-1,		/** < Allocated resource identifier */
	{ 
		(struct sDmaSet *)&Usart6_Tx_DmaSet,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL
	},

};

/* ADC3 dma set */
const struct sDmaSet Adc3_DmaSet =
{
	NULL,	/* link */
	0,		/* stream */
	2		/* channel */
};

DMA_RESOURCE Adc3_DmaResource=
{
	NULL,		/**< sem */
	-1,		/** < Allocated resource identifier */
	{ 
		(struct sDmaSet *)&Adc3_DmaSet,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL
	},

};

/**/

/* Crypto dma set */
const struct sDmaSet Crypt_Out_DmaSet =
{
	NULL,	/* link */
	5,		/* stream */
	2		/* channel */
};

const struct sDmaSet Crypt_In_DmaSet =
{
	(struct sDmaSet *)&Crypt_Out_DmaSet,	/* link */
	6,		/* stream */
	2		/* channel */
};
/**/




#ifdef KUKU
DMA_RESOURCE Crypt_DmaResource=
{
	NULL,		/**< sem */
	-1,		/** < Allocated resource identifier */
	{ 
		(struct sDmaSet *)&Crypt_In_DmaSet,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL
	}
};
#endif


/* I2C1 dma set */
const struct sDmaSet I2C1_Rx_DmaSet =
{
	NULL,	/* link */
	0,		/* stream */
	1		/* channel */
};

const struct sDmaSet I2C1_Tx_DmaSet =
{
	(struct sDmaSet *)&I2C1_Rx_DmaSet,	/* link */
	7,		/* stream */
	1		/* channel */
};
/**/


DMA_RESOURCE I2C1_DmaResource=
{
	NULL,		/**< sem */
	-1,		/** < Allocated resource identifier */
	{ 
		(struct sDmaSet *)&I2C1_Tx_DmaSet,
		NULL,	
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL
	},

};





const DMA_RESOURCE_SET dmacRes[2]=
{
	/* DMAC #1 */
	{ 
		{
			&Usart2_DmaResource,
			&Usart3_DmaResource,
			&I2C1_DmaResource,
			NULL,
			NULL,
			NULL,
			NULL,
			NULL
		}
	},
	
	/* DMAC #2 */
	{
		{
			&Usart6_DmaResource,
			NULL,
			NULL,
			NULL,
			NULL,
			NULL,
			NULL,
			NULL
		}
	}
};

const struct sDmaStreamInfo dmaStreamInfo =
{
	{
		/* DMAC #1 */
		{
			{
				DMA1_Stream0,
				DMA1_Stream1,
				DMA1_Stream2,
				DMA1_Stream3,
				DMA1_Stream4,
				DMA1_Stream5,
				DMA1_Stream6,
				DMA1_Stream7
			},
			{
				DMA1_Stream0_IRQn,
				DMA1_Stream1_IRQn,
				DMA1_Stream2_IRQn,
				DMA1_Stream3_IRQn,
				DMA1_Stream4_IRQn,
				DMA1_Stream5_IRQn,
				DMA1_Stream6_IRQn,
				DMA1_Stream7_IRQn
			}
		},
		
		/* DMAC #2 */
		{
			{
				DMA2_Stream0,
				DMA2_Stream1,
				DMA2_Stream2,
				DMA2_Stream3,
				DMA2_Stream4,
				DMA2_Stream5,
				DMA2_Stream6,
				DMA2_Stream7
			},
			{
				DMA2_Stream0_IRQn,
				DMA2_Stream1_IRQn,
				DMA2_Stream2_IRQn,
				DMA2_Stream3_IRQn,
				DMA2_Stream4_IRQn,
				DMA2_Stream5_IRQn,
				DMA2_Stream6_IRQn,
				DMA2_Stream7_IRQn
			}
		}
	}
};

const uint32_t dmaChanTable[8]={DMA_Channel_0, DMA_Channel_1, DMA_Channel_2, DMA_Channel_3, DMA_Channel_4, DMA_Channel_5, DMA_Channel_6, DMA_Channel_7};
const uint32_t dmaTCflagid[8]={DMA_FLAG_TCIF0,DMA_FLAG_TCIF1,DMA_FLAG_TCIF2,DMA_FLAG_TCIF3,DMA_FLAG_TCIF4,DMA_FLAG_TCIF5,DMA_FLAG_TCIF6,DMA_FLAG_TCIF7};

#ifdef KUKU
{
	/* DMAC #1 */
	{ 
		{
				DMA1_Stream0,
				DMA1_Stream1,
				DMA1_Stream2,
				DMA1_Stream3,
				DMA1_Stream4,
				DMA1_Stream5,
				DMA1_Stream6,
				DMA1_Stream7
		},
		{
			DMA1_Stream0_IRQn,
			DMA1_Stream1_IRQn,
			DMA1_Stream2_IRQn,
			DMA1_Stream3_IRQn,
			DMA1_Stream4_IRQn,
			DMA1_Stream5_IRQn,
			DMA1_Stream6_IRQn,
			DMA1_Stream7_IRQn
		}
	},
	
	/* DMAC #2 */
	{
		{
			DMA2_Stream0,
			DMA2_Stream1,
			DMA2_Stream2,
			DMA2_Stream3,
			DMA2_Stream4,
			DMA2_Stream5,
			DMA2_Stream6,
			DMA2_Stream7
		},
		{
			DMA2_Stream0_IRQn,
			DMA2_Stream1_IRQn,
			DMA2_Stream2_IRQn,
			DMA2_Stream3_IRQn,
			DMA2_Stream4_IRQn,
			DMA2_Stream5_IRQn,
			DMA2_Stream6_IRQn,
			DMA2_Stream7_IRQn
		}
	}
};
#endif

static int16_t findDmaResourceId( DMA_RESOURCE *hDma, uint16_t stream, uint16_t channel);

void initDmaManager(void)
{
	uint16_t dmacID;
	uint16_t dmacResID;
	DMA_RESOURCE_SET *hDmaRes=NULL;
	DMA_RESOURCE *hDma=NULL;
	
	for(dmacID=0;dmacID<(sizeof(dmacRes)/sizeof(DMA_RESOURCE_SET));dmacID++)
	{
		hDmaRes= (DMA_RESOURCE_SET *)&dmacRes[dmacID];
		for (dmacResID=0; dmacResID<MAX_DMA_RESOURCE_SETS; dmacResID++)
		{
			hDma=hDmaRes->dmaRes[dmacResID];
			if (hDma!=NULL)
				hDma->sem=(xSemaphoreHandle)xSemaphoreCreateCounting(1,1);
		}
	}
	
}

DMA_RESOURCE *allocDmaResource(uint16_t dmacID, uint16_t stream, uint16_t channel, portTickType timeout, signed portBASE_TYPE *pxHigherPriorityTaskWoken)
{
	DMA_RESOURCE *hDma=NULL;
	int16_t resId= -1;
	if ((hDma=findDmaResource(dmacID, stream, channel, &resId))!=NULL)
	{
		if (hDma->sem==NULL)
			return NULL;
		if (pxHigherPriorityTaskWoken==NULL)
		{
			if (xSemaphoreTake(hDma->sem,timeout)==pdFALSE)
				return NULL;
			hDma->allocResourceId=resId;
		}
		else
		{
			if (xQueueReceiveFromISR(( xQueueHandle ) ( hDma->sem ),NULL,pxHigherPriorityTaskWoken)==pdFALSE)
				return NULL;
			hDma->allocResourceId=resId;
		}
	}
	return hDma;
}

int freeDmaResource(DMA_RESOURCE *hDma, signed portBASE_TYPE *pxHigherPriorityTaskWoken)
{
	int retVal=pdTRUE;

	if (hDma==NULL)
		return IODEV_EBADARGS;
	if (hDma->sem==NULL)
		return IODEV_EBADARGS;
	hDma->allocResourceId=-1;
	if (pxHigherPriorityTaskWoken==NULL)
		retVal=xSemaphoreGive(hDma->sem);
	else
		retVal=xSemaphoreGiveFromISR(hDma->sem,pxHigherPriorityTaskWoken);
	return retVal;
}

DMA_RESOURCE *findDmaResource(uint16_t dmacID, uint16_t stream, uint16_t channel, int16_t *id)
{
	DMA_RESOURCE_SET *hDmaRes=NULL;
	DMA_RESOURCE *hDma=NULL;
	uint16_t idx;
	int16_t resId= -1;

	if ((sizeof(dmacRes)/sizeof(DMA_RESOURCE_SET))<=dmacID)
	{
		if (id!=NULL)
			*id= -1;
		return NULL;
	}
	hDmaRes= (DMA_RESOURCE_SET *)&dmacRes[dmacID];
	for (idx=0;idx<MAX_DMA_RESOURCE_SETS;idx++)
	{
		hDma=hDmaRes->dmaRes[idx];
		if (hDma==NULL)
		{
			if (id!=NULL)
				*id= -1;
			return NULL;
		}
		resId=findDmaResourceId(hDma, stream, channel);
		if (0<=resId)
		{
			if (id!=NULL)
				*id= resId;
			return hDma;
		}
	}
	if (id!=NULL)
		*id= resId;
	return NULL;
	
}

int16_t findDmaResourceId( DMA_RESOURCE *hDma, uint16_t stream, uint16_t channel)
{
	int16_t id;
	struct sDmaSet *set;

	for(id=0;id<MAX_DMA_RESOURCE_SETS;id++)
	{
		if ((set=hDma->set[id])==NULL)
			return -1;
		while (set!=NULL)
		{
			if ((set->stream==stream) && (set->channel==channel))
				return id;
			 set=set->link;
		}
	}
	return -1;
}

