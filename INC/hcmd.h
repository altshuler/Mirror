#ifndef _HCMD_H
#define _HCMD_H

#include <stdint.h>
#include "msg_type.h"
#include "membuf.h"
#include "payload.h"
#include "packetbuf.h"

#define CMD_BUFFERS	9
#define CMD_BUFFER_SIZE 20
#define CMD_RES_BUFFERS	4
#define CMD_RES_BUFFER_SIZE 152

// for now, literal definition
#define RESP_BUFFER_GET_TIMEOUT	portMAX_DELAY

/*
** Command/Response payload identifiers
*/
#define	ID_CMD				0x0001
#define ID_RESPONSE			0x0002
#define ID_NOTIFICATION		0x0003

#define DRV_DIR_CMD 		0x1000	//Send Driver Direct command
#define SET_STATE			0x3000	//Set Pedestal State  
#define IBIT_RESULT			0x3010	//Request Pedestal Initiated BIT Results
#define GET_KEEP_ALIVE		0x3020	//Pedestal Keep Alive request
#define GET_STATUS			0x3030	//Get Pedestal Status  
#define GET_VER				0x3040	//Get Pedestal Versions
#define GET_CNT				0x3050	//Get Pedestal Lifetime Counters  
#define SET_CNT				0x3060	//Set Pedestal Counters   
#define SET_IBIT			0x3070	//Perform Pedestal IBIT 
#define DIR_CMD_RSP			0x2000	//Driver Direct command responce
#define	STATUS_RSP			0x4000	//Pedestal Status
#define CNT_RSP				0x4010	//Pedestal lifetime counters   
#define VER_RSP				0x4020	//Pedestal versions   
#define IBIT_RES			0x4100	//Pedestal Initiated BIT Results
#define KEEP_ALIVE_RSP		0x4110	//Pedestal Keep Alive Acknowledge   
#define STATE_RSP			0x4120	//Pedestal state acknowledge
#define IBIT_STATUS			0x4130	//Pedestal IBIT Status
#define BOOT_REQ			0x4140	//BootLoader Request
#define SET_ENC_OFFSET		0x4150	//Set Abs Encoder Offset
#define SET_ETH_DELAY   	0x4160	//Set Ethernet Delay
#define SET_NET_SETTINGS    0x4170	//Set Network Settings
#define GET_NET_SETTINGS    0x4180	//Get Network Settings
#define NET_RSP				0x4190	//Network Settings Responce
#define SET_TRAVEL_PIN		0x4200	//Travel Pin Command
#define GET_REOSTAT			0x4210	//Get Travel pin Reostat ADC value
#define REOSTAT_RSP			0x4220	//Travel pin Reostat ADC value Response



#define RX_DELAY	  ( int ) 1
#define TX_DELAY	  ( portTickType ) 100 -( portTickType )RX_DELAY
#define TX_DELAY_ATP  ( portTickType ) 2 -( portTickType )RX_DELAY


union uArg {
	double			d;
	float			f[sizeof (double)/sizeof(float)];
	unsigned long	ul[sizeof (double)/sizeof(unsigned long)];
	signed long		l[sizeof (double)/sizeof(signed long)];
	unsigned short	us[sizeof(unsigned long)/sizeof(unsigned short)];
	signed short	s[sizeof(signed long)/sizeof(signed short)];
	unsigned char	uc[sizeof(unsigned long)/sizeof(unsigned char)];
	signed char	c[sizeof(signed long)/sizeof(signed char)];
};

struct sCommandHeader
{
	uint16_t session_id;		// Command session identifier (for future use)
	uint16_t cmd_id;			// Command identifier (code)
	uint16_t rsvd[2];
	union uArg arg1;		// Command argument field
	// char data[0];
};

#define COMMAND_DATA(p) &((char *)(p))[sizeof(struct sCommandHeader)]

struct sResponseHeader 
{
	uint8_t Carriage_Return;		// Command session identifier  (for future use)
	uint16_t resp_id;			// Response identifier (code)
	uint16_t cmd_id;			// Command identifier (code)
	uint16_t rsvd[1];
	union uArg arg1;
	// char data[0];
};

#define RESPONSE_DATA(p) &((char *)(p))[sizeof(struct sResponseHeader)]

struct sNotificationHeader
{
	uint16_t notification_id;	// Notification identifier (code)
	// char data[0];
};

#define NOTIFICATION_DATA(p) &((char *)(p))[sizeof(struct sNotificationHeader)]
#define PAYLOAD_BUFFER_DATA_LNGH(p) &((char *)(p))[(sizeof(PAYLOAD_BUFFER_HEADER))]
#define PAYLOAD_BUFFER_DATA_STR(p) &((char *)(p))[(sizeof(PAYLOAD_BUFFER_HEADER))+1]
#define PAYLOAD_DATA_START(p) &((char *)(p))[32]

//#define MSGHDR_HOSTCMDRESP_PACKET MAKE_MSG_HDRTYPE(0,MSG_SRC_CMD,MSG_TYPE_PACKET)

#ifdef __cplusplus
extern "C" 
{
#endif

typedef int (*CB_CMD_COMPLETION_FN_t)(void *cmd, void *arg);

void initCommandProcessor();
// b points to payload buffer 
int processHostCommand(void *b,uint32_t ch_num);
int processDriveForward(void *b,uint32_t ch_num);
uint16_t IndenCmd(uint32_t opcode ) ; 
char	strequ(char *s, char *r);
PACKETBUF_HDR *makeSinglePacketResponse(MEMBUF_POOL *pool, PAYLOAD_HEADER *payload, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif

