#ifndef _HOSTCOMM_H
#define _HOSTCOMM_H

#include <stdint.h>
#include "packetbuf.h"
#include "timestamp.h"
#include "membuf.h"

#define LOCAL_RX_BUFFER_SIZE 100



struct sFromHost_packetizerStat
{
	uint32_t no_buffers;
	uint32_t timeout;
	uint32_t host_bin;
	uint32_t err_length;
	uint32_t err_frame;
};

struct sToHost_packetizerStat
{
	uint32_t host_ccs;
	uint32_t host_fcs;
	uint32_t other;
};

struct sFromHost_packetizer
{
	uint8_t	 FieldLen;		// field length				E.A line added
	uint8_t	 FieldCnt;		//field bytes counter			E.A line added
	uint16_t chksum;		//calculated checksum		E.A line added
	uint16_t ReceivedChksum;//calculated checksum		E.A line added
	uint32_t FrameSize;		//Frame size				E.A line added
	uint32_t rxState;
	uint32_t prevRxState;
	MEMBUF_POOL *pool;
	uint32_t bufSize; // Length of buffer data area
	uint32_t rxPayloadLength; // Number of bytes to receive in payload reception states of length oriented protocols
	uint32_t rxPayloadReceived; // Number of received payload bytes
	uint32_t rxIdx;				// Index of current character
	char prevRxChar;
	uint8_t CommProtType;
	PACKETBUF_HDR *buf;
	PACKETBUF_HDR *lastPutInBuf;
	uint32_t lastPutInBufIdx;
	uint32_t addr_l;
	uint32_t addr_h;
	uint16_t addr_cnt;
	struct sFromHost_packetizerStat stat;
};

struct sToHost_packetizer
{
	struct sToHost_packetizerStat stat;
};

struct sHostInterface
{
	void *dev;					/**< Pointer to network device driver */
	int rxPacketTimeout;		/**< Packet reception timeout */
	unsigned int state;			/**< media interface state */
	struct sFromHost_packetizer rxPack;	/**< Host Rx packetizer structure */
	struct sToHost_packetizer txPack;	/**< Host Tx packetizer structure */
};

#define HOST_STATE_IDLE					0 /**< Host interface: Idle */
#define HOST_STATE_TX					1 /**< Host interface: Transmitting, driver enabled */
#define HOST_STATE_RX_WAIT_WINDOW		2 /**< Host interface: Response reception window */
#define HOST_STATE_RX					3 /**< Host interface: Receiving */
#define HOST_STATE_TX_DEFER				4 /**< Host interface: Transmission defer window */


//
/*#define HOST_FRAME_SYNC	0x5D493C8BUL
#define HOST_FRAME_SYNC_LO (HOST_FRAME_SYNC&0xff)
#define HOST_FRAME_SYNC_MID_LO ((HOST_FRAME_SYNC>>8)&0xff)
#define HOST_FRAME_SYNC_MID_HI ((HOST_FRAME_SYNC>>16)&0xff)
#define HOST_FRAME_SYNC_HI ((HOST_FRAME_SYNC>>24)&0xff)*/

#define HOST_ESC 		27
#define HOST_STX 		2
#define HOST_ETX 		3

#define HOST_ENQ 		5
#define HOST_SYNC0 		0x15 //5D493C8B
#define HOST_SYNC1 		0x8B
#define HOST_SYNC2 		0x3C
#define HOST_SYNC3 		0x49
#define HOST_SYNC4 		0x5D

#define RES_CR 			13
#define RES_EOF 		4

#define MAX_ADDRESS 		49
#define MIN_ADDRESS 		126

#define MAX_FRAME_PAYLOAD_LENGHT 		27

// HOST Frame reception states
// Reception of first sync character HOST-BIN('\x8b')
#define HOST_RX_STATE_SYNC_1	0
// Reception of second sync character HOST-BIN('\x3C')
#define HOST_RX_STATE_SYNC_2	1
// Reception of third sync character HOST-BIN('\x49')
#define HOST_RX_STATE_SYNC_3	2
// Reception of fourth sync character HOST-BIN('\x5d')
#define HOST_RX_STATE_SYNC_4	3

#define HOST_RX_ESC		0
#define HOST_RX_ADDRESS	1
#define HOST_RX_STX		2
#define HOST_RX_FRAME_PAYLOAD	3
#define HOST_RX_ETX		4
#define HOST_RX_BCC		5

#define HOST_RX_SYNC1		6
#define HOST_RX_SYNC2		7
#define HOST_RX_SYNC3		8
#define HOST_RX_SYNC4		9
#define HOST_RX_FCS_BCC   10




#define HOST_RX_SYNC_LO			0
#define HOST_RX_SYNC_HI			1
#define HOST_CHECKSM			2
#define HOST_FRAME_SIZE			3
#define HOST_OPCODE				4
#define HOST_SOURCE_ID			5
#define HOST_DEST_ID			6
#define HOST_RESERVED_1			7
#define HOST_RESERVED_2			8
#define HOST_MESSAGE_CNT		9
#define HOST_RESERVED_3			10
#define HOST_RESERVED_4			11
#define HOST_MESSAGE_BODY		12


#define SHORT_LEN				2
#define LONG_LEN				4

#define SYNC_HI					0xAA
#define SYNC_LO					0x55

#define SYNC					0xAA55u


// HOST binary
// Reception of record identifier LOW
#define HOST_RX_FRAME_NUM_LO	4
// Reception of record identifier LOW
#define HOST_RX_FRAME_NUM_HI	5
// Reception of record identifier LOW
#define HOST_RX_FRAME_LEN_LO	6
// Reception of record identifier LOW
#define HOST_RX_FRAME_LEN_HI	7
// Reception of host binary Payload
//#define HOST_RX_FRAME_PAYLOAD	8
// Reception of host binary frame check sequence (FCS)
#define HOST_RX_FRAME_FCS_LO	9
#define HOST_RX_FRAME_FCS_HI	10

extern char net_addr;
extern unsigned long SN_L;
extern unsigned short	SN_H;

#ifdef __cplusplus
extern "C" 
{
#endif

void uart0_HostRxHandler(void);
void uart1_HostRxHandler(void);
int handleRxTimeoutFromHost(struct sFromHost_packetizer *p);
PACKETBUF_HDR *handleRxFromHost(char rxChar, TIMESTAMP rxTS, struct sFromHost_packetizer *p);
void initHostTxStat(struct sToHost_packetizerStat *stat);
void updateHostTxStat(struct sToHost_packetizerStat *stat, PACKETBUF_HDR *p);
int isHostCmdPacket(PACKETBUF_HDR *p);
int HandleRxTimeoutFromHost(struct sFromHost_packetizer *p);
char upper_to_lower(char *pt,char len); 
void hostPutInBuffer(struct sFromHost_packetizer *p, char rxChar, TIMESTAMP rxTS);


#ifdef __cplusplus
}
#endif



#endif

