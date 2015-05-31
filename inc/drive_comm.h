/**
* @file drive_comm.h
* @brief MIRROR drive_comm definitions
*
* @author Evgeny Altshuler
*
* @version 0.0.1
* @date 9.03.2014
*/
#ifndef _DRIVECOMM_H
#define _DRIVECOMM_H
	
#include <stdint.h>
#include "packetbuf.h"
#include "timestamp.h"
#include "membuf.h"
#include "hostcomm.h"
#include "main.h"


struct sFromCtl_packetizerStat
{
	uint32_t no_buffers;
	uint32_t rbin;
	uint32_t timeout;
	uint32_t err_length;
	uint32_t err_frame;
};

struct sToCtl_packetizerStat
{
	uint32_t rbin;
	uint32_t other;
};

struct sFromCtl_packetizer
{
	uint16_t rxState;
	uint16_t prevRxState;
	
	MEMBUF_POOL *pool;
	uint32_t bufSize; /**< Length of buffer data area */
	uint32_t rxPayloadLength; /**< Number of bytes to receive in payload reception states of length oriented protocols */
	uint32_t rxPayloadReceived; /**< Number of received payload bytes */
	uint32_t rxIdx;				/**< Index of current character */
	char prevRxChar;
	unsigned char packetType;	/**< packet type */
	PACKETBUF_HDR *buf;
	PACKETBUF_HDR *lastPutInBuf;
	uint32_t lastPutInBufIdx;
	struct sFromCtl_packetizerStat stat;
	uint16_t timerId;
	void (*startTimer)(uint16_t, uint16_t);
	void (*stopTimer)(uint16_t);
	void (*onTimer)(uint16_t);
	uint16_t timeout;
};

struct sToCtl_packetizer
{
	struct sToCtl_packetizerStat stat;
};

struct sCtlInterface
{
	void *outDev;					/**< Pointer to interface device driver output channel */
	void *inDev;					/**< Pointer to interface device driver input channel */
	int rxPacketTimeout;		/**< Packet reception timeout */
	unsigned int state;			/**< media interface state */
	struct sFromCtl_packetizer rxPack;	/**< CTL Rx packetizer structure */
	struct sToCtl_packetizer txPack;	/**< CTL Tx packetizer structure */
};

#define CTL_STATE_IDLE					0 /**< CTL interface: Idle */
#define CTL_STATE_TX					1 /**< CTL interface: Transmitting, driver enabled */
#define CTL_STATE_RX_WAIT_WINDOW		2 /**< CTL interface: Response reception window */
#define CTL_STATE_RX					3 /**< CTL interface: Receiving */
#define CTL_STATE_TX_DEFER				4 /**< CTL interface: Transmission defer window */

#define MT_PACKET_FROM_CTL		0x20
#define MT_PACKET_FROM_CTL1		0x20
#define MT_PACKET_FROM_CTL2		0x21

#define IS_MT_PACKET_FROM_CTL(x) (((x)&~0xf)==(MT_PACKET_FROM_CTL))



#define SERIAL_RX_BUFFER_SIZE 20
		
// CTL Frame reception states
#define PREAMBLE_LO 	0x8B
#define PREAMBLE_HI 	0x3C

#define MIN_CMD			100  // Opcode 100 is for error cmd (also can be "already ON")
#define MAX_CMD			4244
#define PAYLOAD_LEN		7
#define PACKET_LEN		PAYLOAD_LEN+4


#define PREAMBLE_LSB	0
#define PREAMBLE_MSB	1
#define COMMAND_LSB		2
#define COMMAND_MSB		3
#define ATTRIBUTE		4
#define DATA_3			5
#define DATA_2			6
#define DATA_1			7
#define DATA_0			8
#define CRC_LSB			9
#define CRC_MSB			10

#define CTL_RX_STATE_SYNC_1	(0)
#define CTL_RX_STATE_SYNC_2	(1)
#define CTL_COMMAND_LSB	(2)
#define CTL_COMMAND_MSB	(3)
#define CTL_ATTRIBUTE	(4)
#define CTL_RX_RAYON_PAYLOAD	(5)
#define CTL_RX_RAYON_CRC_LO	(6)
#define CTL_RX_RAYON_CRC_HI	(7)


#define DRIVE_STATE_IDLE					0 /**< Drive interface: Idle */
#define DRIVE_STATE_TX						1 /**< Drive interface: Transmitting, driver enabled */
#define DRIVE_STATE_RX_WAIT_WINDOW			2 /**< Drive interface: Response reception window */
#define DRIVE_STATE_RX						3 /**< Drive interface: Receiving */
#define DRIVE_STATE_TX_DEFER				4 /**< Drive interface: Transmission defer window */

	
#ifdef __cplusplus
extern "C" 
{
#endif

	int handleRxTimeoutFromCtl(struct sFromCtl_packetizer *p);
	PACKETBUF_HDR *handleRxFromCtl(char rxChar, TIMESTAMP rxTS, struct sFromCtl_packetizer *p);
	void initCtlTxStat(struct sToCtl_packetizerStat *stat);
	void updateCtlTxStat(struct sToCtl_packetizerStat *stat, PACKETBUF_HDR *p);
	int handleRxTimeoutFromCtl(struct sFromCtl_packetizer *p);






	void uart1_RxHandler(void);
	void uart2_RxHandler(void);
	void uart3_RxHandler(void);
	PACKETBUF_HDR *handleRxFromDrive(char rxChar, TIMESTAMP rxTS, struct sFromHost_packetizer *p);
	
	
#ifdef __cplusplus
	}
#endif
	
	
	
#endif


