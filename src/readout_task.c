/*******************************************************************************
 * @file readout_task.c
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
#include <queue.h>
//#include <timebase.h>
#include <dev.h>
#include <gio.h>
#include <uart.h>
#include <dma.h>
#include <assert.h>
#include "readout_task.h"
#include "drive_comm.h"
#include "packetbuf.h"
#include "msg_type.h"
//#include "interface.h"
#include "itoa.h"
#include "data.h"
#include "endianutils.h"


#define READOUT_RX_LOCAL_BUFFER_SIZE	16
#define READOUT_RX_LOCAL_BUFFERS	2

#define READOUT_RX_TIMEOUT	portMAX_DELAY

//UART-6
#define READOUT_UART_TX_DMAC_ID 1
#define READOUT_UART_TX_STREAM_ID 6
#define READOUT_UART_TX_CHANNEL_ID 5

#define READOUT_UART_RX_DMAC_ID 1
#define READOUT_UART_RX_STREAM_ID 1
#define READOUT_UART_RX_CHANNEL_ID 5

portTickType RS_422_Rate;
extern uSSI AbsEncoderXData;	
extern uSSI AbsEncoderYData;
extern struct sDriverStatus DriveStatus;
extern struct sPedestalParams	SysParams;
extern uint32_t AbsEncXOffset;
extern uint32_t AbsEncYOffset;

struct sReadoutInterface intReadout;

const struct sReadoutServerParam  readoutServerConfig= 
{
	0,					/* TODO: readoutId */
	5,					/* devId */
	Uart_BaudRate_921_6K,	/* baud */
	Uart_NumStopBits_1,	/* stopBits */
	Uart_CharLen_8,		/* charLen */
	Uart_Parity_NONE	/* parity */

};

//extern struct sGpsInterface intGps[N_CTL];

/* UART handle for input channel */
GIO_Handle hReadoutUart_IN=NULL;

/* UART handle for output channel */
GIO_Handle hReadoutUart_OUT=NULL;

xQueueHandle readoutOutQ=NULL;
xQueueHandle readoutInQ=NULL;

extern Uart_Params *uartParams[];
extern GIO_Handle  uartInputHandle[];
extern GIO_Handle  uartOutputHandle[];


const struct sDmaResSet readoutUartDma=
{
	{
		{
			READOUT_UART_TX_DMAC_ID,
			READOUT_UART_TX_STREAM_ID,
			READOUT_UART_TX_CHANNEL_ID
		},
		{
			READOUT_UART_RX_DMAC_ID,
			READOUT_UART_RX_STREAM_ID,
			READOUT_UART_RX_CHANNEL_ID
		}
	}
};

static int readoutRxCallback(void *arg, int status, void *addr, size_t size);
static void initReadoutTxStat(struct sToReadout_packetizerStat *stat);
static void updateReadoutTxStat(struct sToReadout_packetizerStat *stat, PACKETBUF_HDR*p);

//void handleCtlIncomingData(char *Buffer, size_t len, struct sFromCtl_packetizer *CtlRxPack);
void initReadoutDevParams(void);
void initReadoutxDevParams(Uart_Params *devp, struct sReadoutServerParam *param);

Uart_Params readoutDevParams;

void initReadoutxDevParams(Uart_Params *devp, struct sReadoutServerParam *param)
{
	memcpy(devp, &UART_PARAMS, sizeof(Uart_Params));
	devp->baudRate=param->baud;
	devp->stopBits=param->stopBits;
	devp->charLen=param->charLen;
	devp->parity=param->parity;
}

void initReadoutDevParams(void)
{
	initReadoutxDevParams(&readoutDevParams, &readoutServerConfig);
	stUsartInit();
}


void initReadoutTxStat(struct sToReadout_packetizerStat *stat)
{
	memset(stat, 0, sizeof(struct sToReadout_packetizerStat));
}


void readoutRxServerTask(void *para)
{
	GIO_Attrs gioAttrs = GIO_ATTRS;
	GIO_AppCallback gioAppCallback;
	//MEMBUF_POOL rxBufPool;
	Usart_ChanParams chanParams;
	QUE_Obj localFreeList;
	MSG_HDR readout_in_msg;
	int readoutRxTskStatus= 0;
	size_t idx	 =	0;
	size_t len	 =	0;
	int status	 =	0;
	uint32_t key;
	void *p;
	
	char devName[16];
	char devIdName[5];
	
	struct sReadoutServerParam *readoutCommParam=NULL;
	
	

	readoutCommParam=(struct sReadoutServerParam *)para;

	assert (NULL != readoutCommParam);

  	strcpy(devName,"/Uart");
	strcat(devName, itoa(readoutCommParam->devId,devIdName,10));
	QUE_new(&localFreeList);

	/*
	** Create rx buffers pool
	*/
	//p=pvPortMalloc((CTL_RX_BUFFER_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_CTL_RX_BUFFERS);
	//initMemBufPool(&rxBufPool,p,(CTL_RX_BUFFER_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_CTL_RX_BUFFERS, CTL_RX_BUFFER_SIZE+sizeof(PACKETBUF_HDR),N_CTL_RX_BUFFERS);

	
	/*
	if (intHost.rxPack.buf)
		intHost.rxPack.buf->if_id=0;
	*/
	
	/*
	** Create a list of available local buffers
	*/
	p=pvPortMalloc((READOUT_RX_LOCAL_BUFFER_SIZE)*READOUT_RX_LOCAL_BUFFERS);
	assert(p!=NULL);
	if (p!=NULL)
	{
		for (idx=0;idx<READOUT_RX_LOCAL_BUFFERS;idx++)
		{
			QUE_enqueue(&localFreeList,&(((uint8_t *)p)[0]));
			p= &(((uint8_t *)p)[READOUT_RX_LOCAL_BUFFER_SIZE]);
		}
	}
	
	/*
	* Initialize channel attributes.
	*/
	gioAttrs.nPackets = READOUT_RX_LOCAL_BUFFERS+2;

	#if defined(USE_DMA) && defined(USE_RX_DMA_USART234)
	chanParams.hDma = &readoutUartDma.set[RX_DMA];
	#else
	chanParams.hDma = NULL;
	#endif
	/* Initialize UART
	 */
	hReadoutUart_IN = GIO_create(devName,IODEV_INPUT,&readoutRxTskStatus,&chanParams,&gioAttrs);
	
	key=__disableInterrupts();
	uartInputHandle[readoutServerConfig.devId]=hReadoutUart_IN;
	__restoreInterrupts(key);
	
	if (hReadoutUart_IN)
	{
		// Load the IODDEV_READ commands with buffers into the driver
		while (!QUE_empty(&localFreeList))
		{
			len=READOUT_RX_LOCAL_BUFFER_SIZE;
			p=QUE_dequeue(&localFreeList);
			gioAppCallback.fxn=readoutRxCallback;
			gioAppCallback.arg= readoutInQ;
			status = GIO_submit(hReadoutUart_IN, IODEV_READ, p,&len, &gioAppCallback); // Non blocking Read Gio 
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
			if (xQueueReceive(readoutInQ, &readout_in_msg, READOUT_RX_TIMEOUT))
			{
				if (readout_in_msg.hdr.bit.type==MSG_TYPE_DATA_BLK)
				{
					if (readout_in_msg.buf)
					{
						// Handle Controller comm reception
						//handleCtlIncomingData((char *)readout_in_msg.buf,readout_in_msg.data,&intReadout[readoutCommParam->readoutId].rxPack); // Handle reception on frame level
						// Issue a new driver read command
						len=READOUT_RX_LOCAL_BUFFER_SIZE;
						gioAppCallback.fxn=readoutRxCallback;
						gioAppCallback.arg= readoutInQ;
						status = GIO_submit(hReadoutUart_IN, IODEV_READ, readout_in_msg.buf,&len, &gioAppCallback); //return buf to the driver
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
				//handleRxTimeoutFromCtl(&intReadout[readoutCommParam->readoutId].rxPack);
			}

		}
	}
	else 
	{
		for (;;)
			vTaskDelay(10);
	}
}





void readoutTxServerTask(void *para)
{
	MSG_HDR readout_out_msg;
	PACKETBUF_HDR *p=NULL;
	PACKETBUF_HDR *next=NULL;
	uint32_t key;
	int32_t ReadoutData;

	
	GIO_Attrs gioAttrs = GIO_ATTRS;
	Usart_ChanParams chanParams;
	int status	 =	0;
	//uint16_t fcs;
	char devName[16];
	char devIdName[5];
	
	struct sReadoutServerParam *readoutCommParam=NULL;
	
	readoutCommParam=(struct sReadoutServerParam *)para;

	assert (NULL != readoutCommParam);

  	strcpy(devName,"/Uart");
	strcat(devName, itoa(readoutCommParam->devId,devIdName,10));

	initReadoutTxStat(&intReadout.txPack.stat);
	
	/*
	* Initialize channel attributes.
	*/
	gioAttrs.nPackets = 2;
	

	#if defined(USE_DMA) && defined(USE_TX_DMA_USART234)
	chanParams.hDma = &readoutUartDma.set[TX_DMA];
	#else
	chanParams.hDma = NULL;
	#endif
	/* 
	 * Initialize UART
	 */
	hReadoutUart_OUT = GIO_create(devName,IODEV_OUTPUT,NULL,&chanParams,&gioAttrs);
	
	key=__disableInterrupts();
	uartOutputHandle[readoutServerConfig.devId]=hReadoutUart_OUT;
	__restoreInterrupts(key);

	readout_out_msg.buf=(void *)(&ReadoutData);
	p=(PACKETBUF_HDR *)readout_out_msg.buf;
	p->format= READOUT_BINARY_FORMAT;
	p->dlen=4;
	RS_422_Rate=1000/SysParams.RS422MesRate;
	
	for (;;)
	{
		//ReadoutData=GetReadoutData();
		
		if (hReadoutUart_OUT)
		{
	
			status = GIO_write(hReadoutUart_OUT, (void *)(&ReadoutData), &p->dlen);
			if (status==IODEV_COMPLETED)
			{
				updateReadoutTxStat(&intReadout.txPack.stat, p);
			}
			else if (status==IODEV_EBADIO)
			{
				GIO_delete(hReadoutUart_OUT);
				
				hReadoutUart_OUT = GIO_create(devName,IODEV_OUTPUT,NULL,&chanParams,&gioAttrs);
				key=__disableInterrupts();
				//uartOutputHandle[gpsServerConfig.devId]=hCtlUart_OUT[readoutCommParam->readoutId];
				__restoreInterrupts(key);
			}
			
		}
		
		vTaskDelay(RS_422_Rate); //Should be changed according SysParams.RS422MesRate param and/or transferred to TMR ISR.
	}
}




static int readoutRxCallback(void *arg, int status, void *addr, size_t size)
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


void updateReadoutTxStat(struct sToReadout_packetizerStat *stat, PACKETBUF_HDR*p)
{
	if (p)
	{
		switch (p->format)
		{
		 case READOUT_BINARY_FORMAT:
			stat->readout++;
			break;
		 default:
		 	stat->other++;
		}
	}
}

#ifdef KUKU
int32_t GetReadoutData(void)
{
	int32_t enc_data=0;
	uint32_t  tmp_data;
	uint32_t Diff;
	float 	Degrees;

		if(SysParams.State==SYS_STATE_OPERATE)
		{
			
			tmp_data=AbsEncoderXData.raw32Data>>3;

			if(tmp_data>=DriveStatus.TargetPosCmd)
				Diff=tmp_data-DriveStatus.TargetPosCmd;
			else
				Diff=DriveStatus.TargetPosCmd-tmp_data;		
			
			Degrees=CntToDeg(AbsEncoderXData.raw32Data, AbsEncXOffset);
			
			enc_data= (int32_t)(Degrees*100.0);
			enc_data&=0xE3FFFF;
			enc_data|=0x55000000;
			if(SysParams.AngVelocity>0.05)
			{
				enc_data |= PEDESTAL_MOVE;
				enc_data |= PEDESTAL_POS_DIR;
			}
			else if(SysParams.AngVelocity<-0.05)
			{
				enc_data |= PEDESTAL_MOVE;
				enc_data |= PEDESTAL_NEG_DIR;
			}
			else
				enc_data |= PEDESTAL_STOP;

			if(Degrees>90.0)
				enc_data |= PEDESTAL_NEAR_EDGE;
			else
				enc_data |= PEDESTAL_FAR_EDGE;

		}
		else if(SysParams.State==SYS_STATE_INIT)
			enc_data=0x55443322;

		else if(SysParams.State==SYS_STATE_STDBY)
					enc_data=0x0;

		//enc_data=longBE2LE(enc_data);
return enc_data;
}
#endif

