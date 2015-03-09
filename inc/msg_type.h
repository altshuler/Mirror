#ifndef __MSG_TYPE_H
#define __MSG_TYPE_H

#include <stdint.h>

#define MSG_TYPE_PACKET				0x00	/* Communication packet attached to message */
#define MSG_TYPE_CMD				0x01	/* Command in and/or attached to message */
#define MSG_TYPE_RESP				0x02	/* Response in and/or attached to message */
#define MSG_TYPE_NOTIFY				0x03	/* Notification in and/or attached to message */
#define MSG_TYPE_DATA_BLK			0x04	/* Data block attached to message. Number of elements in data field of the message */
#define MSG_TYPE_EVENT				0x05	/* Event indication attached to message */
#define MSG_TYPE_DRV_1				0x06	/* Packet From/To Driver 1 */
#define MSG_TYPE_DRV_2				0x07	/* Packet From/To Driver 2 */
#define MSG_TYPE_X_ENC				0x08	/* Packet To Axis X Encoder Task */
#define MSG_TYPE_Y_ENC				0x09	/* Packet To Axis Y Encoder Task */



/*
** Message source identifiers
*/
#define MSG_SRC_ISR_TIM		0		/* Message from Timer interrupt service routine */
#define MSG_SRC_ISR_RX		1		/*  Message from UART RX interrupt service routine */
#define MSG_SRC_HOSTRX		2		/* Message from host receiver task */
#define MSG_SRC_HCMD		3		/* Message from hcmd task */
#define MSG_SRC_HCMD_1		4		/* Message from hcmd task, ctl 1 interface */
#define MSG_SRC_HCMD_2		5		/* Message from hcmd task, ctl 2 interface */
#define MSG_SRC_INTERP		6		/* Message from Driver interpreter task */
#define MSG_SRC_ENC			7		/* Message from Encoder task */
#define MSG_SRC_ISR_EMERG	8		/* Message from Emergency Int  */
#define MSG_SRC_ISR_EMERG_1	9		/* Message from Emergency Int, ctl 1 interface */
#define MSG_SRC_ISR_EMERG_2	10		/* Message from Emergency Int, ctl 2 interface */
#define MSG_SRC_CTL1RX		11		/**< Message from CTL receiver task */
#define MSG_SRC_CTL1TX		12		/**< Message from CTL transmitter task */
#define MSG_SRC_CTL2RX		13		/**< Message from CTL receiver task */
#define MSG_SRC_CTL2TX		14		/**< Message from CTL transmitter task */
#define MSG_SRC_READOUTTX	15		/**< Message from readout  task */






struct sMsgHdrTypeBits
{
	uint16_t type:8;
	uint16_t source:5;
	uint16_t len:3;
};

union uMsgHdrType
{
	uint16_t all;
	struct sMsgHdrTypeBits bit;
};

#define MAKE_MSG_HDRTYPE(len,source,type) (uint16_t)(((((uint16_t)(len))&0x7)<<13)|((((uint16_t)(source))&0x1F)<<8)|(((uint16_t)(type))&0xFF))
typedef struct sMsgHdr
{
	union uMsgHdrType hdr;
	uint16_t data;
	void *buf;
} MSG_HDR;



#endif

