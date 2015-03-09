/*******************************************************************************
 * @file Drive_task.c
 * @ Drive Control servers   tasks 
 *
 * @author Evgeni Altshuler
 *
 * @version 0.0.1
 * @date 27.07.2014
 *
*******************************************************************************/


#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <sysport.h>
#include <freertos.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

//#include <timebase.h>
#include <dev.h>
#include <gio.h>
#include <uart.h>
#include <dma.h>
#include <assert.h>

#include "crc16.h"
#include "drive_task.h"
#include "drive_comm.h"
#include "packetbuf.h"
#include "msg_type.h"
#include "interface.h"
#include "itoa.h"
#include "AbsEncoderSSI.h"
#include "handlers.h"
#include "data.h"

#define CTL_RX_BUFFER_SIZE	(12+sizeof(PACKETBUF_HDR))
#define N_CTL_RX_BUFFERS	4
#define CTL_RX_LOCAL_BUFFER_SIZE	24
#define CTL_RX_LOCAL_BUFFERS	4

#define CTL_RX_TIMEOUT	100

#define DRIVE_TASK_TX_DELAY ( portTickType ) 100


#define COMM_TYPE_GET	(1<<4)
#define COMM_TYPE_SET	(0<<4)

#define DATA_TYPE_FLOAT	(1<<5)
#define DATA_TYPE_INT	(0<<5)

#define FWD_COLOR (0)
#define CMD_COLOR	(1)
#define SCAN_COLOR (2)

#define FWD_FRAME_COLOR (FWD_COLOR<<6)
#define CMD_FRAME_COLOR	(CMD_COLOR<<6)
#define SCAN_FRAME_COLOR (SCAN_COLOR<<6)

#define FRAME_COLOR (3<<6)


//UART-2
#define CTL1_UART_TX_DMAC_ID 0
#define CTL1_UART_TX_STREAM_ID 6
#define CTL1_UART_TX_CHANNEL_ID 4

#define CTL1_UART_RX_DMAC_ID 0
#define CTL1_UART_RX_STREAM_ID 5
#define CTL1_UART_RX_CHANNEL_ID 4

//UART-3
#define CTL2_UART_TX_DMAC_ID 0
#define CTL2_UART_TX_STREAM_ID 3
#define CTL2_UART_TX_CHANNEL_ID 4

#define CTL2_UART_RX_DMAC_ID 0
#define CTL2_UART_RX_STREAM_ID 1
#define CTL2_UART_RX_CHANNEL_ID 4


const struct sCtlServerParam  ctlServerConfig[N_CTL]= 
{
	/* Controller #1 */
	{
		0,					/* TODO: ctlId */
		1,					/* devId */
		Uart_BaudRate_115_2K,	/* baud */
		Uart_NumStopBits_1,	/* stopBits */
		Uart_CharLen_8,		/* charLen */
		Uart_Parity_NONE	/* parity */
	},

	/* Controller #2 */
	{
		1,					/* TODO: ctlId */
		2,					/* devId */
		Uart_BaudRate_115_2K, /* baud */
		Uart_NumStopBits_1, /* stopBits */
		Uart_CharLen_8, 	/* charLen */
		Uart_Parity_NONE	/* parity */
	}
};

//extern struct sGpsInterface intGps[N_CTL];

/* UART handle for input channel */
GIO_Handle hCtlUart_IN[N_CTL]={NULL,NULL};

/* UART handle for output channel */
GIO_Handle hCtlUart_OUT[N_CTL]={NULL,NULL};

xQueueHandle ctlOutQ[N_CTL]={NULL,NULL};
xQueueHandle ctlInQ[N_CTL]={NULL,NULL};

extern Uart_Params *uartParams[];
extern GIO_Handle  uartInputHandle[];
extern GIO_Handle  uartOutputHandle[];


const struct sDmaResSet ctlUartDma[N_CTL]=
{
	{
		{
			{
				CTL1_UART_TX_DMAC_ID,
				CTL1_UART_TX_STREAM_ID,
				CTL1_UART_TX_CHANNEL_ID
			},
			{
				CTL1_UART_RX_DMAC_ID,
				CTL1_UART_RX_STREAM_ID,
				CTL1_UART_RX_CHANNEL_ID
			}
		}
	},
	{
		{
			{
				CTL2_UART_TX_DMAC_ID,
				CTL2_UART_TX_STREAM_ID,
				CTL2_UART_TX_CHANNEL_ID
			},
			{
				CTL2_UART_RX_DMAC_ID,
				CTL2_UART_RX_STREAM_ID,
				CTL2_UART_RX_CHANNEL_ID
			}
		}
	}
};

__IO uint16_t T3_CCR1_Val = 300;/*300,30000*/
__IO uint16_t T4_CCR1_Val = 30000;/*30000*/
__IO uint16_t CCR2_Val = 1500;
__IO uint16_t CCR3_Val = 800;
__IO uint16_t CCR4_Val = 150;
__IO uint16_t Brake_PWM_Val = 10000;

uint16_t Timer_4_Period = 0;
uint16_t Timer_9_Period = 0;
uint16_t Channel1Pulse = 0;
uint16_t Channel2Pulse = 0;
uint16_t Channel3Pulse = 0;
uint16_t Channel4Pulse = 0;

xSemaphoreHandle Timer_3_Sem ;
xSemaphoreHandle Timer_4_Sem ;

xQueueHandle 					DriveIntQueue;

float CurrentOffset=0;
extern xQueueHandle             hCmdMbx;

__IO uint32_t Timer1Clk=300000;

struct sDriverStatus DriveStatus=DRIVER_INIT_STATUS;
extern uSSI AbsEncoderXData;	
extern uSSI AbsEncoderYData;

uint8_t ForwardFlag_1=0;
uint8_t ForwardFlag_2=0;


uint32_t interpScanSkipCount=0;

struct sCtlInterface intCtl[N_CTL];

extern MEMBUF_POOL cmdBuffers;
extern uint32_t AbsEncXOffset;
extern uint32_t AbsEncYOffset;

struct sFwdStat 
{
	uint32_t fwd_tx;
	uint32_t fwd_rx;
	uint32_t scan_tx;
	uint32_t scan_rx;
	uint32_t cmd_tx;
	uint32_t cmd_rx;
	uint32_t other_tx;
	uint32_t other_rx;
	
	uint32_t err_len_rx;
	uint32_t err_crc_rx;
	uint32_t scan_skip;
} ;
struct sFwdStat drv_1,drv_2;

char DRV_CMD_MOTOR[3]={'S','V','O'};
char DRV_CMD_POS[3]={'P','O','S'};
char DRV_CMD_MOVE[3]={'M','O','V'};			
char DRV_CMD_REF_MODE[3]={'R','O','N'};		
char DRV_CMD_VELOCITY[3]={'V','E','L'};	


static char * DriveCommands[] = {

DRV_CMD_MOTOR,
DRV_CMD_POS,
DRV_CMD_MOVE,
DRV_CMD_REF_MODE,
DRV_CMD_VELOCITY,
};

static int ctlRxCallback(void *arg, int status, void *addr, size_t size);
void handleCtlIncomingData(char *Buffer, size_t len, struct sFromCtl_packetizer *CtlRxPack, uint16_t fwd, uint16_t chan);
void initCtlDevParams(void);
void initCtlxDevParams(Uart_Params *devp, struct sCtlServerParam *param);

//static int ctl1_tx_buffcall(void *buf, void *arg);
//static int ctl2_tx_buffcall(void *buf, void *arg);


Uart_Params ctlDevParams[N_CTL];

void initCtlxDevParams(Uart_Params *devp, struct sCtlServerParam *param)
{
	memcpy(devp, &UART_PARAMS, sizeof(Uart_Params));
	devp->baudRate=param->baud;
	devp->stopBits=param->stopBits;
	devp->charLen=param->charLen;
	devp->parity=param->parity;
}

void initCtl1DevParams(void)
{
	initCtlxDevParams(&ctlDevParams[0], &ctlServerConfig[0]);
	stUsartInit();
}

void initCtl2DevParams(void)
{
	initCtlxDevParams(&ctlDevParams[1], &ctlServerConfig[1]);
	stUsartInit();
}


void ctlRxServerTask(void *para)
{
	GIO_Attrs gioAttrs = GIO_ATTRS;
	GIO_AppCallback gioAppCallback;
	MEMBUF_POOL rxBufPool;
	Usart_ChanParams chanParams;
	QUE_Obj localFreeList;
	MSG_HDR ctl_in_msg;
	int ctlRxTskStatus= 0;
	size_t idx	 =	0;
	size_t len	 =	0;
	int status	 =	0;
	uint16_t chan;
	uint16_t fwd;
	uint32_t key;
	void *p;
	
	char devName[16];
	char devIdName[5];
	
	struct sCtlServerParam *ctlCommParam=NULL;
	
	

	ctlCommParam=(struct sCtlServerParam *)para;

	assert (NULL != ctlCommParam);

  	strcpy(devName,"/Uart");
	strcat(devName, itoa(ctlCommParam->devId,devIdName,10));
	QUE_new(&localFreeList);

	/*
	** Create rx buffers pool
	*/
	p=pvPortMalloc((CTL_RX_BUFFER_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_CTL_RX_BUFFERS);
	initMemBufPool(&rxBufPool,p,(CTL_RX_BUFFER_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_CTL_RX_BUFFERS, CTL_RX_BUFFER_SIZE+sizeof(PACKETBUF_HDR),N_CTL_RX_BUFFERS);

	/*
	** Initialize channel packetizer
	*/
	intCtl[ctlCommParam->ctlId].rxPack.pool= &rxBufPool;
	intCtl[ctlCommParam->ctlId].rxPack.packetType=MT_PACKET_FROM_CTL+ctlCommParam->ctlId;
	//BUF_stat(&HostRxBuffers, &stat);
	intCtl[ctlCommParam->ctlId].rxPack.bufSize=rxBufPool.bufSize-sizeof(PACKETBUF_HDR);
	intCtl[ctlCommParam->ctlId].rxPack.prevRxState=intCtl[ctlCommParam->ctlId].rxPack.rxState=CTL_RX_STATE_SYNC_1;
	intCtl[ctlCommParam->ctlId].rxPack.rxIdx=0;
	intCtl[ctlCommParam->ctlId].rxPack.rxPayloadLength=intCtl[ctlCommParam->ctlId].rxPack.rxPayloadReceived=0;
	memset(&intCtl[ctlCommParam->ctlId].rxPack.stat, 0, sizeof(intCtl[ctlCommParam->ctlId].rxPack.stat));
	intCtl[ctlCommParam->ctlId].rxPack.buf=getPacketBuffer(intCtl[ctlCommParam->ctlId].rxPack.pool,FIRST_PACKET_SEGMENT|LAST_PACKET_SEGMENT,intCtl[ctlCommParam->ctlId].rxPack.packetType,UNDEFINED_FORMAT,0);
	intCtl[ctlCommParam->ctlId].rxPack.timerId=0;
	
	/*
	if (intHost.rxPack.buf)
		intHost.rxPack.buf->if_id=0;
	*/
	
	/*
	** Create a list of available local buffers
	*/
	p=pvPortMalloc((CTL_RX_LOCAL_BUFFER_SIZE)*CTL_RX_LOCAL_BUFFERS);
	assert(p!=NULL);
	if (p!=NULL)
	{
		for (idx=0;idx<CTL_RX_LOCAL_BUFFERS;idx++)
		{
			QUE_enqueue(&localFreeList,&(((uint8_t *)p)[0]));
			p= &(((uint8_t *)p)[CTL_RX_LOCAL_BUFFER_SIZE]);
		}
	}
	
	/*
	* Initialize channel attributes.
	*/
	gioAttrs.nPackets = CTL_RX_LOCAL_BUFFERS+2;

	#if defined(USE_DMA) && defined(USE_RX_DMA_USART234)
	chanParams.hDma = &ctlUartDma[ctlCommParam->ctlId].set[RX_DMA];
	#else
	chanParams.hDma = NULL;
	#endif
	/* Initialize UART
	 */
	hCtlUart_IN[ctlCommParam->ctlId] = GIO_create(devName,IODEV_INPUT,&ctlRxTskStatus,&chanParams,&gioAttrs);
	
	key=__disableInterrupts();
	uartInputHandle[ctlServerConfig[ctlCommParam->ctlId].devId]=hCtlUart_IN[ctlCommParam->ctlId];
	__restoreInterrupts(key);
	
	if (hCtlUart_IN[ctlCommParam->ctlId])
	{
		// Load the IODDEV_READ commands with buffers into the driver
		while (!QUE_empty(&localFreeList))
		{
			len=CTL_RX_LOCAL_BUFFER_SIZE;
			p=QUE_dequeue(&localFreeList);
			gioAppCallback.fxn=ctlRxCallback;
			gioAppCallback.arg= ctlInQ[ctlCommParam->ctlId];
			status = GIO_submit(hCtlUart_IN[ctlCommParam->ctlId], IODEV_READ, p,&len, &gioAppCallback); // Non blocking Read Gio 
			if (status==IODEV_ENOPACKETS)
				break;
			else if (status==IODEV_COMPLETED)
			{
			}
			else if (status==IODEV_PENDING)
			{
			}
			else
			{
			}
		}
	
	
		for (;;)
		{
			if (xQueueReceive(ctlInQ[ctlCommParam->ctlId], &ctl_in_msg, CTL_RX_TIMEOUT))
			{
				if (ctl_in_msg.hdr.bit.type==MSG_TYPE_DATA_BLK)
				{
					if (ctl_in_msg.buf)
					{
					
						chan=intCtl[ctlCommParam->ctlId].rxPack.packetType-MT_PACKET_FROM_CTL+DRIVER_1_ID;
						if (chan==DRIVER_1_ID)
						{
							
							key=__disableInterrupts();
							fwd=ForwardFlag_1;
							__restoreInterrupts(key);
						}
						else if (chan==DRIVER_2_ID)
						{
							key=__disableInterrupts();
							fwd=ForwardFlag_2;
							__restoreInterrupts(key);
						}
						else
							fwd=0;
						// Handle Controller comm reception
						handleCtlIncomingData((char *)ctl_in_msg.buf,ctl_in_msg.data,&intCtl[ctlCommParam->ctlId].rxPack, fwd, chan); // Handle reception on frame level
						// Issue a new driver read command
						len=CTL_RX_LOCAL_BUFFER_SIZE;
						gioAppCallback.fxn=ctlRxCallback;
						gioAppCallback.arg= ctlInQ[ctlCommParam->ctlId];
						status = GIO_submit(hCtlUart_IN[ctlCommParam->ctlId], IODEV_READ, ctl_in_msg.buf,&len, &gioAppCallback); //return buf to the driver
						if (status==IODEV_ENOPACKETS)
						{
						}
						else if (status==IODEV_COMPLETED)
						{
						}
						else if (status==IODEV_PENDING)
						{
						}
						else
						{
						}
					}
				}
			}
			else
			{
				// Host reception timeout
				// Reset Host packetizer if not in packet synchronization state
				handleRxTimeoutFromCtl(&intCtl[ctlCommParam->ctlId].rxPack);
			}

		}
	}
	else 
	{
		for (;;)
			vTaskDelay(10);
	}
}




void ctlTxServerTask(void *para)
{
	MSG_HDR ctl_out_msg;
	PACKETBUF_HDR *p=NULL;
	PACKETBUF_HDR *next=NULL;
	//PACKETBUF_HDR hdr;
	uint32_t key;
	
	GIO_Attrs gioAttrs = GIO_ATTRS;
	Usart_ChanParams chanParams;
	int status	 =	0;
	//uint16_t fcs;
	char devName[16];
	char devIdName[5];
	
	struct sCtlServerParam *ctlCommParam=NULL;
	
	ctlCommParam=(struct sCtlServerParam *)para;

	assert (NULL != ctlCommParam);

  	strcpy(devName,"/Uart");
	strcat(devName, itoa(ctlCommParam->devId,devIdName,10));

	initCtlTxStat(&intCtl[ctlCommParam->ctlId].txPack.stat);
	
	/*
	* Initialize channel attributes.
	*/
	gioAttrs.nPackets = 2;
	

	#if defined(USE_DMA) && defined(USE_TX_DMA_USART234)
	chanParams.hDma = &ctlUartDma[ctlCommParam->ctlId].set[TX_DMA];
	#else
	chanParams.hDma = NULL;
	#endif
	/* 
	 * Initialize UART
	 */
	hCtlUart_OUT[ctlCommParam->ctlId] = GIO_create(devName,IODEV_OUTPUT,NULL,&chanParams,&gioAttrs);
	
	key=__disableInterrupts();
	uartOutputHandle[ctlServerConfig[ctlCommParam->ctlId].devId]=hCtlUart_OUT[ctlCommParam->ctlId];
	__restoreInterrupts(key);
	
	for (;;)
	{
		if (xQueueReceive(ctlOutQ[ctlCommParam->ctlId],&ctl_out_msg,portMAX_DELAY)==pdPASS)
		{
			if (ctl_out_msg.hdr.bit.type==MSG_TYPE_PACKET)
			{
				if (ctl_out_msg.buf)
				{
					p=(PACKETBUF_HDR *)ctl_out_msg.buf;
					if (p)
					{
						//if (p->h.cbFunc)
						//	(*p->h.cbFunc)(&ctl_out_msg, p->h.cbArg);
						while (p)
						{
							next=p->h.link; // Detach from list
							p->h.link=NULL;
							if (p->dlen)
							{
								
								if (hCtlUart_OUT[ctlCommParam->ctlId])
								{
									// For debugging only
									//localPbuf=(volatile Uint8 *)PACKET_DATA(p);
									//localBufLen=p->Length;
									//
									status = GIO_write(hCtlUart_OUT[ctlCommParam->ctlId], PACKETBUF_DATA(p), &p->dlen);
									if (status==IODEV_COMPLETED)
									{
										updateCtlTxStat(&intCtl[ctlCommParam->ctlId].txPack.stat, p);
									}
									else if (status==IODEV_EBADIO)
									{
										GIO_delete(hCtlUart_OUT[ctlCommParam->ctlId]);

										// For debugging only 
										//vTaskDelay(50);
										//
										
										hCtlUart_OUT[ctlCommParam->ctlId] = GIO_create(devName,IODEV_OUTPUT,NULL,&chanParams,&gioAttrs);
										key=__disableInterrupts();
										//uartOutputHandle[gpsServerConfig.devId]=hCtlUart_OUT[ctlCommParam->ctlId];
										__restoreInterrupts(key);
									}
									
								}
							}
							retMemBuf(p);
							p=next;
						}
					}
					


					
				}
			}
		}
		else if (ctl_out_msg.hdr.bit.type==MSG_TYPE_CMD)
		{
		}
	}
}

void handleCtlIncomingData(char *Buffer, size_t len, struct sFromCtl_packetizer *CtlRxPack, uint16_t fwd, uint16_t chan)
{
	PACKETBUF_HDR *p=NULL;	// packet buffer
    size_t idx   =  0;
	

	for (idx=0;idx<len; idx++)
	{
		p=handleRxFromCtl(Buffer[idx],0,CtlRxPack);
		if (p)
		{	
			if (p->dlen==0)
				retMemBuf(p); // can't forward the buffer due to full dswitch mailbox, return it
			else
			{
				if (sendPacketToDriveInt(p,MSGHDR_CTLRX_PACKET(CtlRxPack->packetType-MT_PACKET_FROM_CTL),portMAX_DELAY,chan)==pdFAIL)
				{
					retMemBuf(p); // can't forward the buffer due to full dswitch mailbox, return it
				}
			}
		}
	}

}



/**************************************************************************/
/* Creating Server test as task for RTOS */
/**************************************************************************/



void DriveInterpTask(void *para)
{
	MSG_HDR drive_in_msg;
	//MSG_HDR msg1;
	PACKETBUF_HDR *pktBuf=NULL;
	PACKETBUF_HDR *TmpBuf=NULL;
	//PACKETBUF_HDR *TmpPktBuf=NULL;
	//PACKETBUF_HDR *TmpPktBuf1=NULL;
	//PACKETBUF_HDR *TmpPktBuf2=NULL;
	struct sDriverCmd DriverCmd1;
	struct sDriverCmd DriverCmd2;
	char command[12];
	uint16_t tmp;
	int32_t Current;
	uint32_t key;
	uint32_t interpScanSkipAcc=0;
	uint8_t color;
	uint16_t chksum;
	
	#ifdef TASK_STACK_CHECK
	unsigned portBASE_TYPE uxHighWaterMark;
	
	/* Inspect our own high water mark on entering the task. */
	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
	#endif

	interpScanSkipCount=0;

	#ifdef KUKU
	/* Initialize  interface queue*/
	DriveIntQueue = xQueueCreate( DRIVE_INT_QUEUE_SIZE, sizeof(MSG_HDR));
	if( DriveIntQueue == NULL )
	{
		// Failed to create the queue.
		while(1)
		{
			vTaskSuspend(NULL);
			vTaskDelay(10);
		}
	}
	#endif

	#ifdef KUKU
	/* synchronize with CTL transmission/reception servers */
	while (1)
	{
		for (tmp=0;tmp<N_CTL;tmp++)
		{
			if ((hCtlUart_OUT[tmp]==NULL) || (hCtlUart_IN[tmp]==NULL))
				break;
		}
		if (tmp<N_CTL)
			vTaskDelay(2);
		else
			break;
	}
	#endif
	
	key=__disableInterrupts();
	/* TIM3 configuration */
	TIM_Config();
	//TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 , ENABLE); //Enable Driver 1 Status and timeout Timers
	__restoreInterrupts(key);

	memset(&drv_1,0,sizeof(drv_1));
	memset(&drv_2,0,sizeof(drv_2));
	while (1)
	{
		if (xQueueReceive( DriveIntQueue, &drive_in_msg,( portTickType ) portMAX_DELAY ))
		{
		
			if (drive_in_msg.hdr.bit.type==MSG_TYPE_PACKET)
			{
				pktBuf=(PACKETBUF_HDR *)drive_in_msg.buf;
				
				if (pktBuf)
				{
					chksum=*(uint16_t *)(PACKETBUF_OFFSET_DATA(pktBuf,pktBuf->dlen-2));
					if(chksum==calcHostFrameCrc((&(PACKETBUF_DATA(pktBuf))[0]),pktBuf->dlen-4))
					{
						if (drive_in_msg.hdr.bit.source==MSG_SRC_HCMD) //Message is coming from network host
						{
							*(PACKETBUF_OFFSET_DATA(pktBuf,4))= (*(PACKETBUF_OFFSET_DATA(pktBuf,4)) & ~FRAME_COLOR) | FWD_FRAME_COLOR; // Force forward color
							if((drive_in_msg.data)==DRIVER_1_ID)
							{
							
								if (pdFAIL==sendPacketToDrive(pktBuf, portMAX_DELAY, DRIVER_1_ID))
									retMemBuf(pktBuf);
								else
								{
									interpScanSkipCount++;
									drv_1.fwd_tx++;
								}
							}
							if((drive_in_msg.data)==DRIVER_2_ID)
							{	
								if (pdFAIL==sendPacketToDrive(pktBuf, portMAX_DELAY, DRIVER_2_ID))
									retMemBuf(pktBuf);
								else
								{
									interpScanSkipCount++;
									drv_2.fwd_tx++;
								}
							}
							else if(drive_in_msg.data==DRIVER_1_2_ID)
							{
								memcpy(&command[1], PACKETBUF_DATA(pktBuf), pktBuf->dlen);
								command[0]=(char)pktBuf->dlen;
								TmpBuf=makeSinglePacketResponse(&cmdBuffers, (PAYLOAD_HEADER*)command, RESP_BUFFER_GET_TIMEOUT);
										
								interpScanSkipAcc=0;							
								if (pdFAIL==sendPacketToDrive(pktBuf, portMAX_DELAY, DRIVER_1_ID))
									retMemBuf(pktBuf);
								else
								{
									interpScanSkipAcc++;
									drv_1.fwd_tx++;
								}
							
								if 	(TmpBuf)
								{
									if (pdFAIL==sendPacketToDrive(TmpBuf, portMAX_DELAY, DRIVER_2_ID))
										retMemBuf(TmpBuf);
									else
									{
										interpScanSkipAcc++;
										drv_2.fwd_tx++;
									}
								}
								interpScanSkipCount+=(interpScanSkipAcc) ? 1 : 0;
							}
						}
						else if (drive_in_msg.hdr.bit.source==MSG_SRC_CTL1RX) //Message is coming from controller 1
						{
							/*TODO: incoming packet need to be parsed*/
							color= *PACKETBUF_OFFSET_DATA(pktBuf,4) & FRAME_COLOR;
						
							tmp=*(uint16_t *)(PACKETBUF_OFFSET_DATA(pktBuf,2)); //Command*/
							if((tmp==3205)||(tmp==205))
							GPIO_ResetBits(LED3_GPIO_PORT, LED3_PIN);

							if(color==FWD_FRAME_COLOR)
							{
								drv_1.fwd_rx++;
								drive_in_msg.hdr.bit.source=MSG_SRC_INTERP;
								drive_in_msg.hdr.bit.type=MSG_TYPE_PACKET;
								drive_in_msg.data=DRIVER_1_ID;
								if (pdFAIL==xQueueSend(hCmdMbx,&drive_in_msg,portMAX_DELAY))
									retMemBuf(pktBuf);
							}
							else if(color==SCAN_FRAME_COLOR)
							{
								drv_1.scan_rx++;
//								DriveStatus.Status1 = *(uint16_t *)(PACKETBUF_OFFSET_DATA(pktBuf,5));
//								if(DriveStatus.Status1&DRIVE_ERROR_BITS)
//								{
//
//								}
								
								//if(tmp==3201)
								//{
								//	PrepareFirstCommand(DRV_STATE_SET_TORQUE,&DriverCmd2,SCAN_FRAME_COLOR);
								//	DriveStatus.Status1 = *(uint16_t *)(PACKETBUF_OFFSET_DATA(pktBuf,5)); //Calculate Current Offset*/
								//	Current= (*(int32_t *)(PACKETBUF_OFFSET_DATA(pktBuf,5)))>>16;
								//	DriverCmd2.DrvData.data2= _IQ15toF(Current)*50.0-CurrentOffset;// new Rayon FW   full scale -193.548
									
								//	if(pdPASS==SendCmdToDrive(DRIVER_2_ID, &DriverCmd2))
								//		drv_2.scan_tx++;
								//	
								//}
								//retMemBuf(pktBuf);
							}
							else if (color==CMD_FRAME_COLOR)
							{
								drv_1.cmd_rx++;
								retMemBuf(pktBuf);
							}
							else
							{
								drv_1.other_rx++;
								retMemBuf(pktBuf);
							}		
						}
						else if (drive_in_msg.hdr.bit.source==MSG_SRC_CTL2RX) //Message is coming from controller 2
						{
							color= *PACKETBUF_OFFSET_DATA(pktBuf,4) & FRAME_COLOR;
							tmp=*(uint16_t *)(PACKETBUF_OFFSET_DATA(pktBuf,2)); 
							if(tmp==284)
								GPIO_ResetBits(LED3_GPIO_PORT, LED3_PIN);
							

							if(color==FWD_FRAME_COLOR)
							{
								drv_2.fwd_rx++;
								drive_in_msg.hdr.bit.source=MSG_SRC_INTERP;
								drive_in_msg.hdr.bit.type=MSG_TYPE_PACKET;
								drive_in_msg.data=DRIVER_2_ID;
								if (pdFAIL==xQueueSend(hCmdMbx,&drive_in_msg,portMAX_DELAY))
									retMemBuf(pktBuf);
							}
							else if(color==SCAN_FRAME_COLOR)
							{
								drv_2.scan_rx++;
								retMemBuf(pktBuf);
							}
							else if (color==CMD_FRAME_COLOR)
							{
								drv_2.cmd_rx++;
								retMemBuf(pktBuf);
							}
							else
							{
								drv_2.other_rx++;
								retMemBuf(pktBuf);
							}
						}
					}
				}
			}
			else if(drive_in_msg.hdr.bit.type==MSG_TYPE_EVENT)
			{
				if (drive_in_msg.hdr.bit.source==MSG_SRC_ISR_TIM) //Message is coming from TMR ISR
				{
					if (interpScanSkipCount==0)
					{
						//PrepareFirstCommand(drive_in_msg.data, &DriverCmd1, SCAN_FRAME_COLOR);
						
						if(pdPASS==SendCmdToDrive(DRIVER_1_ID, drive_in_msg.data))
							drv_1.scan_tx++;
					}
					else
					{
						interpScanSkipCount--;
						drv_1.scan_skip++;
					}
				}
			}
			else if(drive_in_msg.hdr.bit.type==MSG_TYPE_CMD)
			{
				if ((drive_in_msg.hdr.bit.source==MSG_SRC_HCMD_1)||
					(drive_in_msg.hdr.bit.source==MSG_SRC_ISR_EMERG_1)||
					(drive_in_msg.hdr.bit.source==MSG_SRC_ENC))
				{		
					//PrepareFirstCommand(drive_in_msg.data, &DriverCmd1, CMD_FRAME_COLOR);
					DriveStatus.Drive1PacketSent=1;
					xSemaphoreTake(Timer_3_Sem,1);
					if(pdPASS==SendCmdToDrive(DRIVER_1_ID, drive_in_msg.data))
					{
						interpScanSkipCount++;
						drv_1.cmd_tx++;
						
						xSemaphoreTake(Timer_3_Sem,1);
					}
					DriveStatus.Drive1PacketSent=0;
				}
				else if ((drive_in_msg.hdr.bit.source==MSG_SRC_HCMD_2)||
					(drive_in_msg.hdr.bit.source==MSG_SRC_ISR_EMERG_2))
				{		
					//PrepareFirstCommand(drive_in_msg.data, &DriverCmd2, CMD_FRAME_COLOR);
					if(pdPASS==SendCmdToDrive(DRIVER_2_ID, drive_in_msg.data))
					{
						interpScanSkipCount++;
						drv_2.cmd_tx++;
					}
				}
			}

			#ifdef TASK_STACK_CHECK
			/* Calling the function will have used some stack space, we would therefore now expect
			uxTaskGetStackHighWaterMark() to return a value lower than when it was called on
			entering the task. */
			uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
			#endif
		}
	}
}



int sendPacketToDriveInt(void *packet, uint16_t hdr, uint32_t timeout, uint16_t dst_chan)
{
	MSG_HDR msg;
	
	msg.hdr.all=hdr;
	msg.data=dst_chan;
	msg.buf=packet;
	return xQueueSend(DriveIntQueue,&msg,timeout);
}


int sendPacketToDrive(void *packet, uint32_t timeout, uint16_t dst_chan)
{
	MSG_HDR msg;
	PACKETBUF_HDR *pkt;

	
	pkt = (PACKETBUF_HDR*)packet;	
	msg.data=dst_chan;
	msg.buf=packet;

	if (dst_chan == DRIVER_1_ID)
	{
		msg.hdr.all=MAKE_MSG_HDRTYPE(0, MSG_SRC_INTERP, MSG_TYPE_PACKET); //using len in msghdr as forwarding flag
		if(ctlOutQ[0])
			return xQueueSend(ctlOutQ[0],&msg,timeout);
		else
			return pdFAIL;	
	}
	else if (dst_chan == DRIVER_2_ID)
	{
		msg.hdr.all=MAKE_MSG_HDRTYPE(0, MSG_SRC_INTERP, MSG_TYPE_PACKET);
		if(ctlOutQ[1])
			return xQueueSend(ctlOutQ[1],&msg,timeout);
		else
			return pdFAIL;	
		
	}
	else
		return pdFAIL;

	
}



int SendCmdToDrive(uint16_t channel,  uint16_t data)
{
	char command[20];
	//char parameters;
	PACKETBUF_HDR *DriverPckt=NULL;

	
	command[0] = BuildDrivePckt(&command[1], data, channel);
	
	DriverPckt=makeSinglePacketResponse(&cmdBuffers, (PAYLOAD_HEADER*)command, RESP_BUFFER_GET_TIMEOUT);
	if(DriverPckt!=NULL)
	{
		if (sendPacketToDrive(DriverPckt, portMAX_DELAY, channel)==0)
		{
			retMemBuf(DriverPckt);
			return pdFAIL;
		}
	}
	else
		return pdFAIL;
		
	return pdPASS;
}





unsigned char  BuildDrivePckt(char *buf, uint16_t data, uint16_t chan)
{
	char *temp;
	char len;
	char *param=NULL;
	float pos;
	int status;

	
	if (buf!=NULL)
	{
		temp=buf;
		
		switch (data)
		{
			case DRV_STATE_MOTOR_ON:
				
				memcpy(temp, DriveCommands[MOTOR_COMMAND], 3);
				temp+=3;
				*temp=SPACE;
				temp++;
				*temp=AXIS_1;
				temp++;
				*temp=SPACE;
				temp++;
				*temp=ON;
				temp++;
				*temp=LF;
				temp++;
				len=temp-buf;
				
			break;
		
			case DRV_STATE_MOTOR_OFF:
		
				memcpy(temp,DriveCommands[MOTOR_COMMAND], 3);
				temp+=3;
				*temp=SPACE;
				temp++;
				*temp=AXIS_1;
				temp++;
				*temp=SPACE;
				temp++;
				*temp=OFF;
				temp++;
				*temp=LF;
				temp++;
				len=temp-buf;
				
			break;
		
			case DRV_STATE_MOTOR_GET:
				memcpy(temp,DriveCommands[MOTOR_COMMAND], 3);
				temp+=3;
				*temp=GET;
				temp++;
				*temp=LF;
				temp++;
				len=temp-buf;			
				
				
			break;
			
			case DRV_STATE_POS_SET:
				
				memcpy(temp,DriveCommands[POSITION_COMMAND], 3);
				temp+=3;
				*temp=SPACE;
				temp++;
				*temp=AXIS_1;
				temp++;
				*temp=SPACE;
				temp++;
				
				param=ftoa(0.0,&status);
				
				if(status!=FTOA_OK)
					return 0;
				
				while (*param!=0)
				{
					*temp=*param;
					temp++;
					param++;
				}	
				
				*temp=LF;
				temp++;
				len=temp-buf;			
				
				
			break;
			
			case DRV_STATE_POS_GET:
				
				memcpy(temp,DriveCommands[POSITION_COMMAND], 3);
				temp+=3;
				*temp=GET;
				temp++;
				*temp=LF;
				temp++;
				len=temp-buf;				
				
				
			break;
			
			case DRV_STATE_MOVE_SET:
				
				memcpy(temp,DriveCommands[MOVE_COMMAND], 3);
				temp+=3;
				*temp=SPACE;
				temp++;
				*temp=AXIS_1;
				temp++;
				*temp=SPACE;
				temp++;
				
				if(chan==DRIVER_1_ID)
					param=ftoa(-1.0*DriveStatus.TargetPosXCmd,&status);
				else if(chan==DRIVER_2_ID)
					param=ftoa(-1.0*DriveStatus.TargetPosYCmd,&status);
				else
					return 0;
				
				if(status!=FTOA_OK)
					return 0;
				
				while (*param!=0)
				{
					*temp=*param;
					temp++;
					param++;
				}	
				
				*temp=LF;
				temp++;
				len=temp-buf;			
							
				
				
			break;
			
			case DRV_STATE_MOVE_GET:
				
				memcpy(temp,DriveCommands[MOVE_COMMAND], 3);
				temp+=3;
				*temp=GET;
				temp++;
				*temp=LF;
				temp++;
				len=temp-buf;					
				
				
			break;
			
			case DRV_STATE_REF_MODE_ON:
				
				memcpy(temp, DriveCommands[REF_MODE_COMMAND], 3);
				temp+=3;
				*temp=SPACE;
				temp++;
				*temp=AXIS_1;
				temp++;
				*temp=SPACE;
				temp++;
				*temp=ON;
				temp++;
				*temp=LF;
				temp++;
				len=temp-buf;					
						
						
		
			break;
		
			case DRV_STATE_REF_MODE_OFF:
					
				memcpy(temp, DriveCommands[REF_MODE_COMMAND], 3);
				temp+=3;
				*temp=SPACE;
				temp++;
				*temp=AXIS_1;
				temp++;
				*temp=SPACE;
				temp++;
				*temp=OFF;
				temp++;
				*temp=LF;
				temp++;
				len=temp-buf;						
						
		
			break;
		
			case DRV_STATE_REF_MODE_GET:
						
				memcpy(temp,DriveCommands[REF_MODE_COMMAND], 3);
				temp+=3;
				*temp=GET;
				temp++;
				*temp=LF;
				temp++;
				len=temp-buf;								
							
		
			break;
		
			case DRV_STATE_VELOCITY_SET:
						
				memcpy(temp,DriveCommands[VELOCITY_COMMAND], 3);
				temp+=3;
				*temp=SPACE;
				temp++;
				*temp=AXIS_1;
				temp++;
				*temp=SPACE;
				temp++;
				
				param=ftoa(DriveStatus.VelocityCmd, &status);
				
				if(status!=FTOA_OK)
					return 0;
				
				while (*param!=0)
				{
					*temp=*param;
					temp++;
					param++;
				}	
				
				*temp=LF;
				temp++;
				len=temp-buf;			
											
							
		
			break;
		
			case DRV_STATE_VELOCITY_GET:
						
				memcpy(temp,DriveCommands[VELOCITY_COMMAND], 3);
				temp+=3;
				*temp=GET;
				temp++;
				*temp=LF;
				temp++;
				len=temp-buf;								
							
		
			break;
		
		
			default:
				
			break;
			
		}

		return len;
	}
	else 
		return 0;
}

static int ctlRxCallback(void *arg, int status, void *addr, size_t size)
{
	MSG_HDR msg;
	signed portBASE_TYPE xHigherPriorityTaskWoken;
	
	msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_ISR_RX,MSG_TYPE_DATA_BLK);
	msg.data=size;
	msg.buf=addr;
	
	if (!xQueueSendFromISR((xQueueHandle)arg,&msg,&xHigherPriorityTaskWoken))
	{
		// Failure to send message with received buffer
	}
	return 0;
}




int  DriveTimeout(TIM_TypeDef* TIMx,uint32_t timeout)
{
	//uint32_t val;
	uint32_t key;
	
	if((timeout<10)||(timeout>1000000))
		return 0;
	else
	{
		CCR2_Val=(uint16_t)((timeout/10)*3);
		key=__disableInterrupts();
		TIM_ITConfig(TIMx, TIM_IT_CC2, DISABLE);
		TIMx->CCR2 = TIMx->CNT;
		TIMx->SR = (uint16_t)~TIM_IT_CC2;
		TIMx->CCR2 = TIMx->CNT+CCR2_Val;
		TIM_ITConfig(TIMx, TIM_IT_CC2, ENABLE);
		__restoreInterrupts(key);
		return 1;
	}
}





/**
  * @brief  Configure the TIM IRQ Handler.
  * @param  None
  * @retval None
  */
void TIM_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  uint16_t PrescalerValue = 0;

  

  vSemaphoreCreateBinary(Timer_3_Sem);
  if (Timer_3_Sem!=NULL)
		vQueueAddToRegistry( Timer_3_Sem, (signed char *)"Timer_3_Sem");
  xSemaphoreTake(Timer_3_Sem,portMAX_DELAY);
  
  vSemaphoreCreateBinary(Timer_4_Sem);
  if (Timer_4_Sem!=NULL)
		vQueueAddToRegistry( Timer_4_Sem, (signed char *)"Timer_4_Sem");
  //xSemaphoreTake(Timer_4_Sem,portMAX_DELAY);



 
  /* Compute the value to be set in ARR regiter to generate signal frequency at 20 Khz */
  Timer_9_Period = (SystemCoreClock / 20000 ) - 1;
  /* Compute CCR1 value to generate a duty cycle at 50% for channel 1 */
  Channel1Pulse = (uint16_t) (((uint32_t) 5 * (Timer_9_Period - 1)) / 10);
  /* Compute CCR2 value to generate a duty cycle at 50%  for channel 2  */
  Channel2Pulse = (uint16_t) (((uint32_t) 5 * (Timer_9_Period - 1)) / 10);

  /* Compute the value to be set in ARR regiter to generate signal frequency at 20 Khz */
	Timer_4_Period = Timer_9_Period/2;
  /* Compute CCR3 value to generate a duty cycle at 10% for channel 1 */
  Channel3Pulse = (uint16_t) (((uint32_t) 3 * (Timer_4_Period - 1)) / 10);
  /* Compute CCR4 value to generate a duty cycle at 10%  for channel 2  */
  Channel4Pulse = (uint16_t) (((uint32_t) 3 * (Timer_4_Period - 1)) / 10);

  /* TIM9 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9 , ENABLE);
  
  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = Timer_9_Period;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);

  /* Channel 1, 2 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0x0/*Channel1Pulse*/;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  TIM_OC1Init(TIM9, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = 0x0/*Channel2Pulse*/;
  TIM_OC2Init(TIM9, &TIM_OCInitStructure);


  /* TIM9 counter enable */
  TIM_Cmd(TIM9, ENABLE);

  /* TIM9 Main Output Enable */
  TIM_CtrlPWMOutputs(TIM9, ENABLE);

  GPIO_SetBits(BREAK_M1N_GPIO_PORT, BREAK_M1N_PIN | BREAK_M2N_PIN);




  
  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  installInterruptHandler(TIM3_IRQn,__tIM3_IRQHandler,NULL);

  /* Enable the TIM3 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_LOWEST_INTERRUPT_PRIORITY/*0*/;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);




  
  /* -----------------------------------------------------------------------
    TIM3 Configuration: Output Compare Timing Mode:
    
    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM3CLK = 2 * PCLK1  
      PCLK1 = HCLK / 4 
      => TIM3CLK = HCLK / 2 = SystemCoreClock /2
          
    To get TIM3 counter clock at 6 MHz, the prescaler is computed as follows:
       Prescaler = (TIM3CLK / TIM3 counter clock) - 1
       Prescaler = ((SystemCoreClock /2) /6 MHz) - 1
                                              
    CC1 update rate = TIM3 counter clock / CCR1_Val = 146.48 Hz
    ==> Toggling frequency = 73.24 Hz
    
    C2 update rate = TIM3 counter clock / CCR2_Val = 219.7 Hz
    ==> Toggling frequency = 109.8 Hz
    
    CC3 update rate = TIM3 counter clock / CCR3_Val = 439.4 Hz
    ==> Toggling frequency = 219.7 Hz
    
    CC4 update rate = TIM3 counter clock / CCR4_Val = 878.9 Hz
    ==> Toggling frequency = 439.4 Hz

    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f2xx.c file.
     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
     function to update SystemCoreClock variable value. Otherwise, any configuration
     based on this variable will be incorrect.    
  ----------------------------------------------------------------------- */   


  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / Timer1Clk) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
  TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);

  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = T3_CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);

  /* Output Compare Timing Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
  
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable);

   /* Output Compare Timing Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
  
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
 
  /* TIM Interrupts enable */
  TIM_ITConfig(TIM3, /*TIM_IT_CC1 | TIM_IT_CC2 |*/ TIM_IT_CC3 | TIM_IT_CC4, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);




  /* TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
  
  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = Timer_4_Period;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  /* Channel 3, 4 Configuration in PWM mode */
  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  //TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0x0/*Channel3Pulse*/;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  //TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  //TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  TIM_OC3Init(TIM4, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = 0x0/*Channel4Pulse*/;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);


  /* TIM4 counter enable */
  TIM_Cmd(TIM4, ENABLE);

  /* TIM4 Main Output Enable */
  TIM_CtrlPWMOutputs(TIM4, ENABLE);
}






char *ftoa(float f, int *status)
{
	long mantissa, int_part, frac_part;
	short exp2;
	LF_t x;
	char *p;
	static char outbuf[15];

	*status = 0;
	
	if (f == 0.0)
	{
		outbuf[0] = '0';
		outbuf[1] = '.';
		outbuf[2] = '0';
		outbuf[3] = 0;
		
		return outbuf;
	}
	
	x.F = f;

	exp2 = (unsigned char)(x.L >> 23) - 127;
	mantissa = (x.L & 0xFFFFFF) | 0x800000;
	frac_part = 0;
	int_part = 0;

	if (exp2 >= 31)
	{
		*status = FTOA_TOO_LARGE;
		return 0;
	}
	else if (exp2 < -23)
	{
		*status = FTOA_TOO_SMALL;
		return 0;
	}
	else if (exp2 >= 23)
		int_part = mantissa << (exp2 - 23);
	else if (exp2 >= 0)
	{
		int_part = mantissa >> (23 - exp2);
		frac_part = (mantissa << (exp2 + 1)) & 0xFFFFFF;
	}
	else /* if (exp2 < 0) */
		frac_part = (mantissa & 0xFFFFFF) >> -(exp2 + 1);

	p = outbuf;

	if (x.L < 0)
		*p++ = '-';

	if (int_part == 0)
		*p++ = '0';
	else
	{
	p=itoa(int_part, p, 10);
		while (*p)
		p++;
	}
	*p++ = '.';

	if (frac_part == 0)
		*p++ = '0';
	else
{
char m, max;

max = sizeof (outbuf) - (p - outbuf) - 1;
if (max > 8)//was 7
max = 8;
/* print BCD */
for (m = 0; m < max; m++)
{
/* frac_part *= 10; */
frac_part = (frac_part << 3) + (frac_part << 1);

*p++ = (frac_part >> 24) + '0';
frac_part &= 0xFFFFFF;
}
/* delete ending zeroes */
for (--p; p[0] == '0' && p[-1] != '.'; --p)
;
++p;
}
*p = 0;

return outbuf;
}


