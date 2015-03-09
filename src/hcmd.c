/*******************************************************************************
 * @file Hcmd.c
 * @ Host command interpriter  
 *
 * @author Andrei Mamtsev
 *
 * @version 0.0.1
 * @date 09.02.2013
 *
**************************************************************************/
/**************************************************************************/
/* Standard Includes */
/**************************************************************************/
#include <stddef.h>
#include <stdint.h>
#include <string.h>
/**************************************************************************/
/* RTOS Includes */
/**************************************************************************/
#include "freertos.h"
#include "semphr.h"
/**************************************************************************/
/* Library Includes */
/**************************************************************************/


/**************************************************************************/
/* Driver includes */
/**************************************************************************/


/**************************************************************************/
/* Src includes */
/**************************************************************************/
#include "membuf.h"
#include "payload.h"
#include "packetbuf.h"
#include "hcmd.h"
//#include "commands.h"
//#include "responses.h"
#include "packetbuf.h"
#include "payload.h"
#include "hostframe.h"
//#include "ver.h"
//#include "board.h"
#include "timestamp.h"
#include "inthost.h"
//#include "service_task.h"
#include "data.h"
//#include "ver.h"
//#include "Weight_task.h"
#include "main.h"
#include "bootinfo.h"
#include "sysport.h"



/**************************************************************************/
/*Declaration of global variables*/
/**************************************************************************/

#define	NO_CMD					0	//    no cmd 	
#define	CMD_DRIVE      			1    //Send Driver Direct command
#define	CMD_SET_STATE			2	//	Set Pedestal State  
#define	CMD_REQ_IBIT_RES		3	//	Request Pedestal Initiated BIT Results
#define	CMD_KEEP_ALIVE_REQ		4	//	Keep Alive Request
#define	CMD_GET_STATUS			5	//	Get Pedestal Status 
#define	CMD_GET_VER				6	//	Get Pedestal Versions 
#define	CMD_GET_CNT				7	//	Get Pedestal Lifetime Counters 
#define	CMD_SET_CNT				8	//	Set Pedestal Lifetime Counters 
#define	CMD_IBIT				9	//	Perform Pedestal IBIT  
#define	RESP_DRIVE				10	//	Driver Direct command responce
#define	PEDESTAL_STATUS			11	//	Pedestal Status
#define	PEDESTAL_CNT			12	//	Pedestal lifetime counters    
#define	PEDESTAL_VER			13	//	Pedestal versions  
#define	PEDESTAL_IBIT_RES		14	//	Pedestal Initiated BIT Results  
#define	PEDESTAL_KA_ACK			15	//	Pedestal Keep Alive Acknowledge    
#define	PEDESTAL_STATE_ACK		16	//	Pedestal state acknowledge
#define	PEDESTAL_IBIT_STATUS	17	//	Pedestal IBIT Status
#define	CMD_BOOT				18	// BootLoader Request
#define	CMD_ENC_OFFSET			19	// Enc Offset
#define	CMD_ETH_TX_DELAY		20	// ETH Tx Delay Command
#define	CMD_SET_NETWORK			21	// Set Network Details
#define	CMD_GET_NETWORK			22	// Get Network Details
#define	PEDESTAL_NETWORK		23	// Pedestal Network Details
#define	CMD_TRAVEL_PIN			24	// Open/Close Travel Pin
#define	CMD_GET_REOSTAT			25	//Get Travel pin Reostat ADC value
#define	CMD_REOSTAT_RSP			26	//Travel pin Reostat ADC value Response




/*************************************************************************\
*************************    Command Table     ****************************
\*************************************************************************/
static uint16_t cmd_table[] = {
	DRV_DIR_CMD,		//Send Driver Direct command
	SET_STATE,			//Set Pedestal State  
	IBIT_RESULT,		//Request Pedestal Initiated BIT Results
	GET_KEEP_ALIVE,		//Pedestal Keep Alive request
	GET_STATUS,			//Get Pedestal Status  
	GET_VER,			//Get Pedestal Versions
	GET_CNT,			//Get Pedestal Lifetime Counters  
	SET_CNT,			//Set Pedestal Counters   
	SET_IBIT,			//Perform Pedestal IBIT 
	DIR_CMD_RSP,		//Driver Direct command responce
	STATUS_RSP,			//Pedestal Status
	CNT_RSP,			//Pedestal lifetime counters   
	VER_RSP,			//Pedestal versions   
	IBIT_RES,			//Pedestal Initiated BIT Results
	KEEP_ALIVE_RSP,		//Pedestal Keep Alive Acknowledge   
	STATE_RSP,			//Pedestal state acknowledge
	IBIT_STATUS,		//Pedestal IBIT Status
	BOOT_REQ,			//BootLoader Request
	SET_ENC_OFFSET,		//Set Abs Encoder Offset
	SET_ETH_DELAY,		//Set ETH Tx Delay
	SET_NET_SETTINGS,	//Set Network Settings
	GET_NET_SETTINGS,	//Get Network Settings
	NET_RSP,			//Network Settings Responce
	SET_TRAVEL_PIN,		//Travel Pin Command
	GET_REOSTAT,		//Get Travel pin Reostat ADC value
	REOSTAT_RSP,		//Travel pin Reostat ADC value Response
	0
};

static char responsePayloadBuffer[150];
static char responseDriveBuffer[100];


xSemaphoreHandle semCmdCompletion=NULL;

/**************************************************************************/
/*Extern Declarations*/
/**************************************************************************/
extern MEMBUF_POOL cmdResBuffers;
extern float result_out;
extern uint8_t pbit_cnt;
extern struct sBootInfoArea bootInfo;
void (*bootloaderReq)(void) __attribute__ ((section (".bootloaderreq")));
extern uint32_t AbsEncXOffset;
extern uint32_t AbsEncYOffset;
extern uSSI AbsEncoderXCnt;
extern uSSI AbsEncoderYCnt;
extern uSSI AbsEncoderXData;
extern uSSI AbsEncoderYData;

extern struct sIpSettings ipSettings;


void cmdCompletionCallback(void *);
int cmdWaitCompletion(uint32_t Timeout);
size_t getRespBufferSize(MEMBUF_POOL *pool);


/**
* @fn initCommandProcessor( void )
*
* Function to initialization command interpriter .
*
* @author Andrei Mamtsev
*
* @param void
*
* @return void
*
* @date 17.02.2013
*/
void initCommandProcessor()
{
	vSemaphoreCreateBinary(semCmdCompletion);
	
	xSemaphoreTake(semCmdCompletion,portMAX_DELAY);
}



/**
* @fn int processDriveForward(void *b,uint32_t ch_num)
*
* Function to forward responce from driver
*
* @author Evgeny Altshuler
*
* @param void *b - 			pointer to payload buffer
* @param uint32_t ch_num - 	number of communication channel
*
* @return int -status
*
* @date 16.03.2014
*/

int processDriveForward(void *b,uint32_t ch_num)
{
	struct sPacketBufHdr *rxbuff=NULL;
	
	
	PAYLOAD_HEADER* pr=(PAYLOAD_HEADER*)responseDriveBuffer;
	PACKETBUF_HDR *resp_packet=NULL;
	
	
	if (b==NULL)
		return 0; // return the buffer on exit
		
	rxbuff=(struct sPacketBufHdr *)(b);
	
	memset(pr, 0, ((sizeof(char))*100));
	pr->length=SetDriveRsp(((char*)PAYLOAD_DATA_START(b)),PAYLOAD_START(pr), ch_num&0xFF);
	resp_packet=makeSinglePacketResponse(&cmdResBuffers,pr,RESP_BUFFER_GET_TIMEOUT);
	
	if (resp_packet)
	{
		if (sendPacketToHost(resp_packet, MAKE_MSG_HDRTYPE(0, MSG_SRC_HCMD, MSG_TYPE_PACKET), portMAX_DELAY)==0)
		{
			retMemBuf(resp_packet);
		}
	}


	return 1;
}






/**
* @fn int processHostCommand(void *b,uint32_t ch_num)
*
* Function to initialization command interpriter .
*
* @author Andrei Mamtsev
*
* @param void *b - 			pointer to payload buffer
* @param uint32_t ch_num - 	number of communication channel
*
* @return int -status
*
* @date 17.02.2013
*/

int processHostCommand(void *b,uint32_t ch_num)
{
	//PAYLOAD_HEADER *pb=(PAYLOAD_HEADER *)b;
	PAYLOAD_HEADER *ph=NULL;
	//struct sCommandHeader *cmd=NULL;
	struct sPacketBufHdr *rxbuff=NULL;
	uint16_t cmd_id=0;
	char channel=0;
	uint32_t key;
	void (*fBootReq)(void);	
	PAYLOAD_HEADER* pr=(PAYLOAD_HEADER*)responsePayloadBuffer;
	//union uAllResponses *resp=(union uAllResponses *)PAYLOAD_RESULT(pr);
	PACKETBUF_HDR *resp_packet=NULL;
	PACKETBUF_HDR *drive_packet=NULL;
	MSG_HDR msg;
	
	//CB_CMD_COMPLETION_FN_t waitForPostResponseCommandCompletion=NULL;
	//uint16_t ret_len;
	
	if (b==NULL)
		return 1; // return the buffer on exit
	ph=(PAYLOAD_HEADER *)PAYLOAD_BUFFER_DATA_STR(b);
	rxbuff=(struct sPacketBufHdr *)(b);
	cmd_id=IndenCmd(rxbuff->rsvd);

	// Execute the command
	switch (cmd_id)
	{
		case CMD_DRIVE:
			memset(pr, 0, ((sizeof(char))*100));
			pr->length=SetDriveCmd(((char*)PAYLOAD_DATA_START(b)),PAYLOAD_START(pr), &channel );
			drive_packet=makeSinglePacketResponse(&cmdResBuffers,pr,RESP_BUFFER_GET_TIMEOUT);
			break;
			
		case CMD_SET_STATE:
			memset(pr, 0, ((sizeof(char))*100));
			pr->length=SetStateAck(((char*)PAYLOAD_DATA_START(b)),PAYLOAD_START(pr));
			resp_packet=makeSinglePacketResponse(&cmdResBuffers,pr,RESP_BUFFER_GET_TIMEOUT);
			break;
			
		case CMD_REQ_IBIT_RES:
			memset(pr, 0, ((sizeof(char))*100));
			pr->length=IBITResults(PAYLOAD_START(pr));
			resp_packet=makeSinglePacketResponse(&cmdResBuffers,pr,RESP_BUFFER_GET_TIMEOUT);
			break;

		case CMD_KEEP_ALIVE_REQ:
			memset(pr, 0, ((sizeof(char))*60));
			pr->length=KeepAliveAck(PAYLOAD_START(pr));
			resp_packet=makeSinglePacketResponse(&cmdResBuffers,pr,RESP_BUFFER_GET_TIMEOUT);
			break;

		case CMD_GET_STATUS:
			memset(pr, 0, ((sizeof(char))*140));
			pr->length=PedestalStatus(PAYLOAD_START(pr));
			resp_packet=makeSinglePacketResponse(&cmdResBuffers,pr,RESP_BUFFER_GET_TIMEOUT);
			break;

		case CMD_GET_VER:
			
			memset(pr, 0, ((sizeof(char))*140));
			pr->length=VersionResp(PAYLOAD_START(pr));
			resp_packet=makeSinglePacketResponse(&cmdResBuffers,pr,RESP_BUFFER_GET_TIMEOUT);
			
			break;

		case CMD_GET_CNT:
			memset(pr, 0, ((sizeof(char))*60));
			pr->length=LifetimeCounterResp(PAYLOAD_START(pr));
			resp_packet=makeSinglePacketResponse(&cmdResBuffers,pr,RESP_BUFFER_GET_TIMEOUT);
			break;

		case CMD_SET_CNT:
			memset(pr, 0, ((sizeof(char))*60));
			pr->length=LifetimeCounterSet(((char*)PAYLOAD_DATA_START(b)),PAYLOAD_START(pr));
			resp_packet=makeSinglePacketResponse(&cmdResBuffers,pr,RESP_BUFFER_GET_TIMEOUT);
			break;

		case CMD_IBIT:
			memset(pr, 0, ((sizeof(char))*60));
			pr->length=IBITStatus(PAYLOAD_START(pr));
			resp_packet=makeSinglePacketResponse(&cmdResBuffers,pr,RESP_BUFFER_GET_TIMEOUT);
			break;

		case CMD_BOOT:
			memset(pr, 0, ((sizeof(char))*60));
			pr->length = FactoryIpResp(PAYLOAD_START(pr),ipSettings.IPAddress);
			resp_packet=makeSinglePacketResponse(&cmdResBuffers,pr,RESP_BUFFER_GET_TIMEOUT);
			if (resp_packet)
			{
				if (sendPacketToHost(resp_packet, MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD,MSG_TYPE_PACKET), portMAX_DELAY)==0)
				{
					retMemBuf(resp_packet);
				}
			}
			vTaskDelay(50);
			
			if ((uint32_t)bootloaderReq==0xffffffff)
			{
				if(bootInfo.bootRqAddr == (~bootInfo.bootRqAddrComp))
				{
					fBootReq = bootInfo.bootRqAddr;
					fBootReq();
					while(1);
				}
			}
			else
			{
				bootloaderReq();
				while(1);
			}		
			break;

		case CMD_ENC_OFFSET:
			msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_1,MSG_TYPE_CMD);
			msg.data=DRV_STATE_MOTOR_OFF;
			msg.buf=NULL;
			xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);
			vTaskDelay(1);
		
			msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_2,MSG_TYPE_CMD);	
			xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);
			vTaskDelay(1);	
			
			key=__disableInterrupts();
			#ifndef RANISHOW
			AbsEncXOffset=AbsEncoderXData.raw32Data>>3;
			#else
			AbsEncXOffset=(uint32_t)((AbsEncoderXData.raw32Data>>25)&0x3FFFFFF);
			AbsEncYOffset=(uint32_t)((AbsEncoderYData.raw32Data>>25)&0x3FFFFFF);
			#endif
			__restoreInterrupts(key);
			SetApplicationData(&AbsEncXOffset,ABS_X_ENC_OFFSET_ADDR);
			vTaskDelay(10);
			SetApplicationData(&AbsEncYOffset,ABS_Y_ENC_OFFSET_ADDR);
			
			msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_1,MSG_TYPE_CMD);
			msg.data=DRV_STATE_MOTOR_ON;
			msg.buf=NULL;
			xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);
			vTaskDelay(1);
		
			msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_2,MSG_TYPE_CMD);	
			xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);
			vTaskDelay(1);	
			break;

		case CMD_ETH_TX_DELAY:
			
			SetGraphCmd((char*)PAYLOAD_DATA_START(b));
			
			break;


		case CMD_SET_NETWORK:

			memset(pr, 0, ((sizeof(char))*100));
			pr->length = SetNetworkDetails(((char*)PAYLOAD_DATA_START(b)),PAYLOAD_START(pr));
			resp_packet=makeSinglePacketResponse(&cmdResBuffers,pr,RESP_BUFFER_GET_TIMEOUT);
			
			break;

		case CMD_GET_NETWORK:

			memset(pr, 0, ((sizeof(char))*100));
			pr->length = NetworkDetailsResp(PAYLOAD_START(pr),STATUS_OK);
			resp_packet=makeSinglePacketResponse(&cmdResBuffers,pr,RESP_BUFFER_GET_TIMEOUT);
			
			break;	


		case CMD_TRAVEL_PIN:
			
			
			break;



		case CMD_GET_REOSTAT:

			
			break;
					
		default:
			
			break;
		}
	if (resp_packet)
	{
		if (sendPacketToHost(resp_packet, MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD,MSG_TYPE_PACKET), portMAX_DELAY)==0)
		{
			retMemBuf(resp_packet);
		}
	}

	
	if (drive_packet)
		{
			if (sendPacketToDriveInt(drive_packet, MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD,MSG_TYPE_PACKET), portMAX_DELAY,channel)==0)
			{
				retMemBuf(drive_packet);
			}
		}

	return 1;
}

/**
* @fn PACKETBUF_HDR *makeSinglePacketResponse(MEMBUF_POOL *pool, PAYLOAD_HEADER *payload,int Format, uint32_t timeout)
*
* Function make packet to response .
*
* @author Andrei Mamtsev
*
* @param MEMBUF_POOL *pool - 	pointer to buffer pool 
* @param PAYLOAD_HEADER *payload - 	pointer to payload
* @param int Format - 	packet format   FCS_FORMAT-2; CCS_FORMAT-1
* @uint32_t timeout - 	timeout
*
* @return PACKETBUF_HDR * -pointer to response packet.
*
* @date 17.02.2013
*/

PACKETBUF_HDR *makeSinglePacketResponse(MEMBUF_POOL *pool, PAYLOAD_HEADER *payload, uint32_t timeout)
{
	PACKETBUF_HDR *packet=NULL;
	
	if ((payload==NULL)||(payload->length==0))
		return NULL;
	packet=getPacketBufferWithWait(pool,FIRST_PACKET_SEGMENT|LAST_PACKET_SEGMENT, HOST_NETWORK, 0, 0, timeout);
	if (packet==NULL)
		return NULL;

	
	memcpy(PACKETBUF_DATA(packet),HOST_SFRAME_START(payload),payload->length);
	packet->dlen=payload->length;

	
	return packet;
}

/**
* @fn size_t getRespBufferSize(MEMBUF_POOL *pool)
*
* Return buffer size.
*
* @author Andrei Mamtsev
*
* @param MEMBUF_POOL *pool - 	pointer to buffer pool 
*
* @return size_t  - 				responce buffer size.
*
* @date 17.02.2013
*/

size_t getRespBufferSize(MEMBUF_POOL *pool)

{
	if (pool==NULL)
		return 0;
	return pool->bufSize;
}

int cmdWaitCompletion(uint32_t timeout)
{
	return xSemaphoreTake(semCmdCompletion,timeout);
}

void cmdCompletionCallback(void *arg)
{
	xSemaphoreGive(semCmdCompletion);
}

/**
* @fn uint16_t IndenCmd(void *b)  
*
* Search command in command table.
*
* @author Andrei Mamtsev
*
* @param void *b - 		pointer to command 
*
* @return uint16_t  - 		responce number of command.
*
* @date 17.02.2013
*/
uint16_t IndenCmd(uint32_t opcode )  
{
	uint16_t cmd_id=0;
		
	while(cmd_table[cmd_id])
	{
		/* search command table */
		if(cmd_table[cmd_id]==(uint16_t)opcode)
			return(cmd_id+1);
		cmd_id++;
	}
	return(NO_CMD);
}

char	strequ(char *s, char *r)
{
	while(*s == *r)
	{
		s++;
		r++;
                if(*r == 0)
			return(1);
	}
	return(0);
}


