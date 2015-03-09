/**
* @file intctl.c
* @brief gps interface.
*
* @author Evgeny Altshuler
*
* @version 0.0.1
* @date 27.07.2014
*
*/
#include <stddef.h>
#include <string.h>
#include <assert.h>
/**************************************************************************/
/* RTOS Includes */
/**************************************************************************/
#include <sysport.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include "buff.h"
#include "drive_comm.h"
#include "msg_type.h"
#include "membuf.h"
//#include "timebase.h"
#include "interface.h"
#include "drive_task.h"


//extern xSemaphoreHandle UartRxSem[];

extern xQueueHandle ctlOutQ[N_CTL];


//struct sCtlInterface intCtl[N_CTL];

#ifdef KUKU
#define CTL_SERVER_RX_BUFFERS				5	/**< Number of buffers */
#define CTL_BUF_RX_SIZE					128	/**< Size of buffer data area */

uint8_t MemAreaRXPoolCtl[N_CTL][CTL_BUF_RX_SIZE*CTL_SERVER_RX_BUFFERS];
MEMBUF_POOL ctlRxPool[N_CTL];
#define CTL_PACKET_TIMEOUT 100

/**
* @fn int initCtlInterface(struct sHostInterface *iface, void *dev)
*
* This function initializes the internal RS485 network
*
* @author Eli Schneider
*
* @param iface pointer to interface control data structure
* @param dev pointer to network device data structure
*
* @date 10.01.2011
*/
int initCtlInterface(struct sCtlInterface *iface, uint16_t ctlId)
{
	int status=0;

	
    assert(ctlId < N_CTL);
	
	iface->outDev=NULL;
	iface->outDev=NULL;
 	iface->rxPacketTimeout=GPS_PACKET_TIMEOUT;
	iface->state=CTL_STATE_IDLE;
	memset(&iface->rxPack, 0, sizeof(iface->rxPack));
	memset(&iface->txPack, 0, sizeof(iface->txPack));
	iface->rxPack.pool= &ctlRxPool[ctlId];
	status=initMemBufPool(iface->rxPack.pool, &MemAreaRXPoolCtl[N_CTL], CTL_BUF_RX_SIZE*CTL_SERVER_RX_BUFFERS, GPS_BUF_RX_SIZE,GPS_SERVER_RX_BUFFERS);
	if(!status)
	{
		// Failed to create the buffer pool.
		status=-1;
	}
	iface->rxPack.bufSize=iface->rxPack.pool->bufSize-sizeof(struct sPacketBufHdr);
	iface->rxPack.rxState=CTL_RX_STATE_SYNC_1;
	
	memset(&iface->txPack, 0, sizeof(iface->txPack));
	initCtlTxStat(&iface->txPack.stat);
	return status;
}
#endif

unsigned int getCtlInterfaceState(struct sCtlInterface *iface)
{
	unsigned int retVal;
	uint32_t key;

	key=__disableInterrupts();
	retVal=iface->state;
	__restoreInterrupts(key);
	return retVal;
}

/**
* @fn unsigned int setCtlInterfaceState(struct sGpsInterface *iface, unsigned int val)
*
* This function sends a packet  on host tx channel using PDMA 
*
* @author Eli Schneider
*
* @param iface pointer to host interface
* @param val pointer to new state value
*
* @return previous state 
*
* @date 17.03.2010
*/
unsigned int setCtlInterfaceState(struct sCtlInterface *iface, unsigned int val)
{
	unsigned int retVal;
	uint32_t key;
	
	key=__disableInterrupts();
	retVal=iface->state;
	iface->state=val;
	__restoreInterrupts(key);
	return retVal;
}


/**
* @fn int sendPacketToCtl(uint16_t ctlId, void *packet, uint16_t hdr, uint32_t timeout)
*
* This function sends a packet  to the ctls transmission server 
*
* @author Eli Schneider
*
* @param ctlId pointer to packet
* @param packet pointer to packet
* @param hdr message header
* @param timeout message send timeout
*
* @return send status
*
* @date 30.05.2012
*/
int sendPacketToCtl(uint16_t ctlId, void *packet, uint16_t hdr, uint32_t timeout)
{
	MSG_HDR msg;

    assert(ctlId < N_CTL);
	
	msg.hdr.all=hdr;
	msg.data=0;
	msg.buf=packet;
	return xQueueSend(ctlOutQ[ctlId],&msg,timeout);
}

