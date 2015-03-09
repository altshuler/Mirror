/**
  ******************************************************************************
  * @file    Eth_Rx_Task.c
  * @author Evgeny Altshuler
  * @version V0.0.1
  * @date    28-January-2014
  * @brief   Main program body
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f2x7_eth.h"
#include "netconf.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
//#include "timers.h"
#include "tcpip.h"
#include "tcp.h"
#include "serial_debug.h"
#include "eth_Rx_Task.h"
#include "hostcomm.h"
#include "msg_type.h"
#include "intHost.h"
#include "payload.h"
#include "data.h"
#include "hcmd.h"
#include <string.h>
#include "ip_param_config.h"
#include "Eeprom.h"
//#include "core_cm3.h"

#define TX_DELAY_TASK ( portTickType ) 100

#define TCP_CONN_CLIENT_NONE 				0
#define TCP_CONN_CLIENT_CONNECTED 			1
#define TCP_CONN_CLIENT_BOUND 				2
#define TCP_CONN_CLIENT_FAIL 				3
#define TCP_CONN_CLIENT_NO_LINK 			4
#define TCP_CONN_CLIENT_DISCONNECT 			5

#define TCP_CONN_TYPE_RX 			1
#define TCP_CONN_TYPE_TX 			2

//enum ETH_CONN_TYPE
//{
//	RxConnType = 1,
//	TxConnType = 2
//}econnType;

#ifdef KUKU
void vEthTxTimeoutTimerCallback( xTimerHandle pxTxTimeoutTimer );
xTimerHandle xEthTxTimeOutTimer;
#endif

extern MEMBUF_POOL 				cmdResBuffers;
extern struct sHostInterface 	intHost;
extern xQueueHandle 			hCmdMbx;
extern xQueueHandle 			intHostTXQueue;


extern struct sIbitStatus		IbitStatus;
extern struct netif 			xnetif; /* network interface structure */

extern __IO uint32_t  			EthInitStatus;
extern __IO uint32_t  			EthLinkStatus;

portTickType TxDelayValue = TX_DELAY;
struct ip_addr server_addr;
struct ip_addr my_addr;
//extern struct sIpSettings ipSettings;
extern struct sAppIpSettings AppipSettings;

struct EthConn
{
	struct netconn *EConn;
	int conn_status;
	uint16_t port;
};

//struct EthConn RxConn;
struct EthConn TxConn;

uint8_t connType = 0; // 1 = Rx; 2 = Tx
void Eth_Connect(uint8_t ethConnType);

static void Tx_connn_callback(struct netconn *, enum netconn_evt, u16_t len);

//char cmdBuffersMemory[CMD_BUFFERS*CMD_BUFFER_SIZE];
//MEMBUF_POOL cmdBuffers;

//char cmdResBuffersMemory[CMD_RES_BUFFERS*CMD_RES_BUFFER_SIZE];
//MEMBUF_POOL cmdResBuffers;



/**
  * @brief  Eth_Connect
  * @param  ethConnType: 1= Rx; 2= Tx; else no connection
  * @retval None
  */
void Eth_Connect(uint8_t ethConnType)
{
	err_t error = 0;
	struct EthConn *TempConnn = NULL;

	//if(ethConnType == TCP_CONN_TYPE_RX)
	//	TempConnn = &RxConn;
	//else if(ethConnType == TCP_CONN_TYPE_TX)
		TempConnn = &TxConn;
	//else
	//	return; // should never get here!!!

	my_addr.addr = xnetif.ip_addr.addr;
	/* set up the IP address of the remote host */
	server_addr.addr = AppipSettings.ServerIPAddress;  //server IP address: 192.168.1.3

	if(EthInitStatus == 0)
	{
		ETH_BSP_Config();
		if(EthInitStatus == 1)
			LwIP_Init();
	}
	else if(TempConnn->conn_status != TCP_CONN_CLIENT_CONNECTED)
	{
		if(TempConnn->EConn != NULL)
			error = netconn_delete(TempConnn->EConn);
	}

	if(EthInitStatus == 1)
	{
		if(TempConnn->conn_status != TCP_CONN_CLIENT_CONNECTED)
		{
			/* create a new connection */
			TempConnn->EConn = netconn_new(NETCONN_TCP);

			if (TempConnn->EConn != NULL)
			{
				error = ERR_OK;
				TempConnn->EConn->recv_timeout = RX_DELAY;

				TempConnn->EConn->callback = NULL/*Tx_connn_callback*/; // optional: get netconn events

				TempConnn->EConn->err = ERR_OK;

				if(ethConnType == TCP_CONN_TYPE_RX) // binding is not required for Tx
					error = netconn_bind(TempConnn->EConn, &my_addr, TempConnn->port);

				if(error == ERR_OK)
				{
					tcp_nagle_disable(TempConnn->EConn->pcb.tcp);
					error = netconn_connect(TempConnn->EConn, &server_addr, TempConnn->port);
					if (error == ERR_OK)
						TempConnn->conn_status = TCP_CONN_CLIENT_CONNECTED;	
					else
						TempConnn->conn_status = TCP_CONN_CLIENT_FAIL;
				}
				else if(error == ERR_USE)
					TempConnn->conn_status = TCP_CONN_CLIENT_BOUND;
				else
					TempConnn->conn_status = TCP_CONN_CLIENT_FAIL;
			}
			else
				TempConnn->conn_status = TCP_CONN_CLIENT_FAIL;
		}
	}
}


/**
  * @brief  Eth_Rx_Task task
  * @param  pvParameters not used
  * @retval None
  */
//void Eth_Rx_Task(void * pvParameters)
//{
//	struct netbuf *buf3 = NULL;
//	uint32_t test;
//	PACKETBUF_HDR *pktBuf=NULL;
//	MSG_HDR msg;
//	size_t idx;
//
//	my_addr.addr = xnetif.ip_addr.addr;
//	/* set up the IP address of the remote host */
//	addr.addr = htonl(0xC0A80103);  //server IP address: 192.168.1.3
//
//	// Initialize RxConn
//	RxConn.conn_status = TCP_CONN_CLIENT_NONE;
//	RxConn.port = 8030;
//
//	while(1)
//	{
//		if(RxConn.conn_status != TCP_CONN_CLIENT_CONNECTED)
//		{
//			//printf("Test");
//			Eth_Connect(TCP_CONN_TYPE_RX);
//		}
//
//		if(RxConn.conn_status == TCP_CONN_CLIENT_CONNECTED)
//		{
//			test = xnetif.ip_addr.addr;
//			/*check if IP address assigned*/
//			if (test !=0)
//			{
//				while( RxConn.conn_status == TCP_CONN_CLIENT_CONNECTED )
//				{
//					buf3 = netconn_recv(RxConn.EConn);
//					if(ERR_IS_FATAL(RxConn.EConn->err))
//					{
//						RxConn.conn_status = TCP_CONN_CLIENT_FAIL;
//						netbuf_delete(buf3);
//						vTaskDelay(10);
//						break;
//					}
//					if ((buf3) != NULL)
//					{
//						for (idx=0;idx<buf3->p->len;idx++)
//						{
//							pktBuf=handleRxFromHost(((*((int8_t *)(buf3->p->payload)+idx))), 0, &intHost.rxPack);
//
//							if(pktBuf)
//							{
//								msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HOSTRX,MSG_TYPE_PACKET);
//								msg.data=0;
//								#ifdef KUKU
//								if(dev==USART1)
//									pktBuf->usartnum=1;
//								else if(dev==USART3)
//									pktBuf->usartnum=3;
//								else
//									pktBuf->usartnum=0;
//								#endif
//								msg.buf=pktBuf;
//								xQueueSend(hCmdMbx,&msg,portMAX_DELAY);
//							}
//						}
//						netbuf_delete(buf3);
//						vTaskDelay(10);
//					}
//					else
//						vTaskDelay(100);
//				}
//				vTaskDelay(100);
//			}
//		}
//		else
//			vTaskDelay(100);
//	}
//}

/**
  * @brief  connn_callback
  *  Optional: function to handle netconn events
  * @retval None
  */
void Tx_connn_callback(struct netconn * myConnn, uint8_t event, u16_t len)
{
	uint32_t test = 0;
	switch(event)
	{
	case 1:
		if(len > 150)
			TxConn.conn_status = TCP_CONN_CLIENT_FAIL;
		test = 1;
		break;
	case 2:
		test = 2;
		break;
	case 3:
		test = 3;
		break;
	case 4:
		test = 4;
		break;
	default:
		test = 0;
		break;
	}
}

/**
  * @brief  Eth_Tx_Task task
  * @param  pvParameters not used
  * @retval None
  */
void Eth_Tx_Task(void * pvParameters)
{
	char pr_buff[150];
	uint8_t len=0;
	//uint32_t test;
	PACKETBUF_HDR *pktBuf=NULL;
	MSG_HDR msg;
	err_t error = 0;
	struct netbuf *buf3 = NULL;
	size_t idx;

	
	#ifdef KUKU
	intHostTXQueue = xQueueCreate( HOST_TX_QUEUE_SIZE, sizeof(MSG_HDR) );
	if( intHostTXQueue == 0 )
	{
		// Failed to create the queue.
		while(1)
		{
			vTaskSuspend(NULL);
			vTaskDelay(10);
		}
	}
	#endif
	
	// Initialize TxConn
	
	//xEthTxTimeOutTimer = xTimerCreate ((const signed char *)"xEthTxTimeOutTimer",300,pdFALSE,0,vEthTxTimeoutTimerCallback);



	while(1)
	{
		//if(TxConn.conn_status != TCP_CONN_CLIENT_CONNECTED)
		//{
		//	Eth_Connect(TCP_CONN_TYPE_TX);
		//}

		if(TxConn.conn_status == TCP_CONN_CLIENT_CONNECTED)
		{
			while (TxConn.conn_status == TCP_CONN_CLIENT_CONNECTED)
			{
				buf3 = netconn_recv(TxConn.EConn);
				if(ERR_IS_FATAL(TxConn.EConn->err))
				{
					netbuf_delete(buf3);
					TxConn.conn_status = TCP_CONN_CLIENT_FAIL;

					//vTaskDelay(10);
					//break;
				}
				if ((buf3) != NULL)
				{
					for (idx=0;idx<buf3->p->len;idx++)
					{
						pktBuf=handleRxFromHost(((*((int8_t *)(buf3->p->payload)+idx))), 0, &intHost.rxPack);

						if(pktBuf)
						{
							if(pktBuf)
							{
								msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HOSTRX,MSG_TYPE_PACKET);
								msg.data=0;
								msg.buf=pktBuf;
								xQueueSend(hCmdMbx,&msg,portMAX_DELAY);
							}
						}
					}
					netbuf_delete(buf3);
				}


				if(TxConn.conn_status == TCP_CONN_CLIENT_CONNECTED)
				{
					if (xQueueReceive( intHostTXQueue, &msg, TxDelayValue ) )
					{
						if (msg.hdr.bit.source==MSG_SRC_HCMD)
						{
							if (msg.hdr.bit.type==MSG_TYPE_PACKET)
							{
								pktBuf=(PACKETBUF_HDR *)msg.buf;
								if (pktBuf)
								{
									vTaskDelay(10);
									//xTimerStart( xEthTxTimeOutTimer, 0 );
									error = netconn_write(TxConn.EConn,(void *)PACKETBUF_DATA(pktBuf) , pktBuf->dlen, NETCONN_COPY);
									if(error != ERR_OK)
									{
										TxConn.conn_status = TCP_CONN_CLIENT_FAIL;
									}
									//xTimerStop(xEthTxTimeOutTimer,0);
								}
								retMemBuf(pktBuf);
							}
							else if (msg.hdr.bit.type==MSG_TYPE_CMD)
							{
								netconn_close(TxConn.EConn);
								TxConn.conn_status = TCP_CONN_CLIENT_DISCONNECT;
								vTaskSuspend(NULL);
							}	
						}						
					}
					else
					{

						if(IbitStatus.Status==IBIT_FINISHED)
							len=PedestalStatus(pr_buff+(sizeof(uint8_t)));
						else
						{
							vTaskDelay(900);
							len=IBITStatus(pr_buff+(sizeof(uint8_t)));
						}

						pr_buff[0]=len;
						pktBuf=makeSinglePacketResponse(&cmdResBuffers,((PAYLOAD_HEADER *)pr_buff),portMAX_DELAY);
						if (pktBuf)
						{
							if( TxConn.conn_status == TCP_CONN_CLIENT_CONNECTED )
							{
								//xTimerStart( xEthTxTimeOutTimer, 0 );
								error = netconn_write(TxConn.EConn,(void *)PACKETBUF_DATA(pktBuf) , pktBuf->dlen, NETCONN_COPY);
								if(error != ERR_OK)
								{
									TxConn.conn_status = TCP_CONN_CLIENT_FAIL;
								}
								//xTimerStop(xEthTxTimeOutTimer,0);
							}
							retMemBuf(pktBuf);
						}
					}
				}
			}
				vTaskDelay(50);
				
		}
		else
			vTaskDelay(50);
	}
}



int sendPacketToHost(void *packet, uint16_t hdr, uint32_t timeout)
{
	MSG_HDR msg;

	msg.hdr.all=hdr;
	msg.data=0;
	msg.buf=packet;
	return xQueueSend(intHostTXQueue,&msg,timeout);
}

void checkEthLink(void *pvParameters)
{
	
	TxConn.conn_status = TCP_CONN_CLIENT_NONE;
	TxConn.port = (uint16_t)AppipSettings.Port;
	Eth_Connect(TCP_CONN_TYPE_TX);

	while (1)
	{
		if (EthLinkStatus != 0)
		{
			TxConn.conn_status = TCP_CONN_CLIENT_FAIL;
			//Eth_Connect(TCP_CONN_TYPE_TX);
			vTaskDelay(TxDelayValue*3);
		}
		else
		{
			if(TxConn.conn_status != TCP_CONN_CLIENT_CONNECTED)
				Eth_Connect(TCP_CONN_TYPE_TX);
			vTaskDelay(TxDelayValue*3);
		}
	}
}

#ifdef KUKU
void vEthTxTimeoutTimerCallback( xTimerHandle pxTxTimeoutTimer )
{
	configASSERT( pxTxTimeoutTimer );

	TxConn.conn_status = TCP_CONN_CLIENT_FAIL;
	Eth_Connect(TCP_CONN_TYPE_TX);
}
#endif

