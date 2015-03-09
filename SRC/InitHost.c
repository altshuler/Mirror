/**
* @file inthost.c
* @brief meters internal network services.
*
* @author Andrei Mamtsev
*
* @version 0.0.1
* @date 09.01.2013
*
*/
/**************************************************************************/
/* Standard Includes */
/**************************************************************************/
#include <stddef.h>
#include <string.h>
/**************************************************************************/
/* RTOS Includes */
/**************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
/**************************************************************************/
/* Library Includes */
/**************************************************************************/
//#include "stm32f10x_map.h"
//#include "stm32f10x_usart.h"
//#include "stm32f10x_dma.h"
//#include "stm32f10x_nvic.h"

/**************************************************************************/
/* Driver includes */
/**************************************************************************/

/**************************************************************************/
/* Src includes */
/**************************************************************************/
#include "buff.h"
#include "IntHost.h"
#include "hostcomm.h"
#include "Msg_type.h"

/**************************************************************************/
/*Declaration of global variables*/
/**************************************************************************/
xQueueHandle intHostRXQueue = NULL;
xQueueHandle intHostTXQueue = NULL;

//struct sHostInterface intHost;

#define SERVER1_RX_BUFFERS				5	/**< Number of buffers */
#define BUF1_RX_SIZE					128//64//128	/**< Size of buffer data area */	//E.A    was 64

uint8_t MemAreaRXPool1[BUF1_RX_SIZE*SERVER1_RX_BUFFERS];
MEMBUF_POOL hostRxPool;
#define HOST_PACKET_TIMEOUT 100
/**************************************************************************/
/*Extern Declarations*/
/**************************************************************************/
extern xSemaphoreHandle UartRxSem[];

/**
* @fn int initIntNetwork(struct sIntNetwork *net)
*
* This function initializes the internal RS485 network
*
* @author Eli Schneider
*
* @param p pointer to network control data structure
* @param p pointer to network device data structure
*
* @date 10.01.2011
*/
int initHostInterface(struct sHostInterface *iface, void *dev)
{
	int status=0;
	
	iface->dev=dev;
 	iface->rxPacketTimeout=HOST_PACKET_TIMEOUT;
	iface->state=HOST_STATE_IDLE;
	memset(&iface->rxPack, 0, sizeof(iface->rxPack));
	memset(&iface->txPack, 0, sizeof(iface->txPack));
	iface->rxPack.pool= &hostRxPool;
	status=initMemBufPool(iface->rxPack.pool, MemAreaRXPool1, BUF1_RX_SIZE*SERVER1_RX_BUFFERS, BUF1_RX_SIZE,SERVER1_RX_BUFFERS);
	if(!status)
	{
		// Failed to create the buffer pool.
		status=-1;
	}
	iface->rxPack.bufSize=iface->rxPack.pool->bufSize-sizeof(struct sPacketBufHdr);
	iface->rxPack.rxState=HOST_RX_ESC;
	
	memset(&iface->txPack, 0, sizeof(iface->txPack));
	initHostTxStat(&iface->txPack.stat);
	return status;
}


#ifdef KUKU
unsigned int getHostInterfaceState(struct sHostInterface *iface)
{
	unsigned int retVal;

	portENTER_CRITICAL() ;
	retVal=iface->state;
	portEXIT_CRITICAL();
	return retVal;
}

unsigned int setHostInterfaceState(struct sHostInterface *iface, unsigned int val)
{
	unsigned int retVal;
	
	portENTER_CRITICAL() ;
	retVal=iface->state;
	iface->state=val;
	portEXIT_CRITICAL();
	return retVal;
}

/**
* @fn int sendHostPacket(void *packetBuf, struct sHostInterface *if)
*
* This function sends a packet  on host tx channel using PDMA 
*
* @author Eli Schneider
*
* @param packetBuf pointer to packet buffer
* @param iface pointer to host interface
*
* @return 0=success, otherwise, error code
*
* @date 17.03.2010
*/
int sendHostPacket(void *packetBuf, struct sHostInterface *iface, uint32_t ch)
{
 	uint8_t *txBuf;
 	USART_TypeDef *USARTHOST;
	uint32_t DMA1ChannelTxIRQn;
	//DMA_Channel_TypeDef     *DMA1ChannelTx;
	//DMA_Stream_TypeDef		*DMA1StreamTx;
	uint32_t USARTHOSTTxDRBase;
	uint32_t USARTHOSTIRQn;

	
	if (packetBuf==NULL)
		return -1;
	txBuf=(uint8_t *)PACKETBUF_OFFSET_DATA(((PACKETBUF_HDR*)packetBuf),((PACKETBUF_HDR*)packetBuf)->offset);
	#ifdef KUKU
	if(ch==1)
	{
		USARTHOST=USART_HOST_RS422;              	//USART6
		// USARTHOSTIRQChannel=USART_HOST_IRQChannel_RS485;      //USART1_IRQChannel
		USARTHOSTTxDRBase=USART_HOST_Tx_DR_Base_RS485;      //USART1_DR_Base
		DMA1ChannelTx=DMA1_Channel_Tx_RS485 ;          // DMA1_Channel4
		DMA1ChannelTxIRQn=DMA1_Channel_Tx_IRQn_RS485 ;     // DMAChannel4_IRQChannel
		USARTHOSTIRQn=USART_HOST_IRQn_RS485  ;    		//USART1_IRQn
	}
	else if (ch==3)
	{
		RS422_Tx_EN_RX_DIS;
		USARTHOST=USART_HOST_RS422 ;               	
		// USARTHOSTIRQChannel=USART_HOST_IRQChannel_RS485 ;    
		USARTHOSTTxDRBase=USART_HOST_Tx_DR_Base_RS422;      
		DMA1ChannelTx=DMA1_Channel_Tx_RS422 ;         
		DMA1ChannelTxIRQn=DMA1_Channel_Tx_IRQn_RS422 ;     
		USARTHOSTIRQn=USART_HOST_IRQn_RS422 ;     		
	}
	
	
	#ifdef USART_HOST_USART3
		RS422_Tx_EN_RX_DIS;
	#endif
	#ifdef USART_HOST_USART2
		RS422_Tx_EN_RX_DIS;
	#endif
	#ifdef USART_HOST_USART1
		ISL3175_DE_ENABLE;
	#endif
    #endif

	DMA_Configuration(txBuf,4);




#ifdef KUKU

    /* Configure DMA1_Channel_Tx interrupt */
    NVIC_EnableIRQ(DMA1ChannelTxIRQn/*DMA1_Channel_Tx_IRQn*/);
   
    /* Configure USARTx interrupt */
    NVIC_EnableIRQ(USARTHOSTIRQn/*USART_HOST_IRQn*/);

	/* Configure the DMA */
	DMA_Configuration( /*DMA1_Channel_Tx*/DMA1ChannelTx,(uint32_t)txBuf,USARTHOSTTxDRBase/*USART_HOST_Tx_DR_Base*/,((PACKETBUF_HDR*)packetBuf)->dlen,DMA_DIR_PeripheralDST);

	/* Enable USART_Tx DMA Tansmit request */
	USART_DMACmd(USARTHOST/*USART_HOST*/, USART_DMAReq_Tx, ENABLE);
	
	/* Enable DMA1 Channel_Tx */
	DMA_Cmd(/*DMA1_Channel_Tx*/DMA1ChannelTx, ENABLE);
	/* Enable the USART_Tx */
	USART_Cmd(USARTHOST/*USART_HOST*/, ENABLE);
	xSemaphoreTake(UartRxSem[0],portMAX_DELAY);
#endif
 	return 0;
}


/*UART1>>DMAChannel4_IRQHandler ; 		UART2>>DMAChannel7_IRQHandler ;   	UART3>>DMAChannel2_IRQHandler */
/*UART1>>DMA1_FLAG_TC4 ; 		    	 	UART2>>DMA1_FLAG_TC7 ;  			UART3>>DMA1_FLAG_TC2*/

//#ifdef USART_HOST_USART1
void DMAChannel4_IRQHandler (void) 
{
	int x=0;
	
	portBASE_TYPE xHigherPriorityTaskWoken;
	
    /*  Clear DMA1_Channel4 Transfer Complete Flag*/
    DMA_ClearFlag(DMA1_FLAG_TC4); // DMA1_FLAG_TC2
	NVIC_ClearPendingIRQ(DMA1_Channel_Tx_IRQn_RS485);
    NVIC_DisableIRQ(DMA1_Channel_Tx_IRQn_RS485);
	/* Disable DMA1 Channel_Tx */
	DMA_Cmd(DMA1_Channel_Tx_RS485, DISABLE);
	xSemaphoreGiveFromISR(UartRxSem[0],&xHigherPriorityTaskWoken);
	
	if( xHigherPriorityTaskWoken )
	{
		// Actual macro used here is port specific.
		taskYIELD();
	}
	if(Dumpall_sm.state)
	{
		sendToServiceFromIsr((CMD_DUMPALL), MAKE_MSG_HDRTYPE(0,MSG_SRC_CMD,MSG_TYPE_EVENT));

	}
}
#ifdef USART_HOST_USART2
void DMAChannel7_IRQHandler (void) 
{
	int x=0;
	
	portBASE_TYPE xHigherPriorityTaskWoken;
	
    /*  Clear DMA1_Channel4 Transfer Complete Flag*/
    DMA_ClearFlag(DMA1_FLAG_TC7); // DMA1_FLAG_TC2
	NVIC_ClearPendingIRQ(DMA1_Channel_Tx_IRQn); 
    NVIC_DisableIRQ(DMA1_Channel_Tx_IRQn);
	/* Disable DMA1 Channel_Tx */
	DMA_Cmd(DMA1_Channel_Tx, DISABLE);
	xSemaphoreGiveFromISR(UartRxSem[0],&xHigherPriorityTaskWoken);
	if( xHigherPriorityTaskWoken )
	{
		// Actual macro used here is port specific.
		taskYIELD();
	}
}
#endif

//#ifdef USART_HOST_USART3
void DMAChannel2_IRQHandler (void) 
{
	int x=0;
	
	portBASE_TYPE xHigherPriorityTaskWoken;
	
    /*  Clear DMA1_Channel4 Transfer Complete Flag*/
    DMA_ClearFlag(DMA1_FLAG_TC2); // DMA1_FLAG_TC2
	NVIC_ClearPendingIRQ(DMA1_Channel_Tx_IRQn_RS422); 
    NVIC_DisableIRQ(DMA1_Channel_Tx_IRQn_RS422);
	/* Disable DMA1 Channel_Tx */
	DMA_Cmd(DMA1_Channel_Tx_RS422, DISABLE);
	xSemaphoreGiveFromISR(UartRxSem[0],&xHigherPriorityTaskWoken);
	if( xHigherPriorityTaskWoken )
	{
		// Actual macro used here is port specific.
		taskYIELD();
	}
	
	if(Dumpall_sm.state)
	{
		sendToServiceFromIsr((CMD_DUMPALL), MAKE_MSG_HDRTYPE(0,MSG_SRC_CMD,MSG_TYPE_EVENT) );

	}
}
#endif

