/**
* @file packetbuf.h
* @brief packet buffers system.
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 09.01.2011
*
*/

#ifndef _PACKETBUF_H
#define _PACKETBUF_H

#include "membuf.h"
#include "buff.h"

#define FIRST_PACKET_SEGMENT		0x01
#define LAST_PACKET_SEGMENT			0x02
#define TX_ECHO_PACKET				0x04


#define IP_NETWORK					0x01
#define MAIN_NETWORK				0x02
#define HOST_NETWORK				0x03


#define UNDEFINED_FORMAT			0
#define RAYON_BINARY_FORMAT			1
#define READOUT_BINARY_FORMAT			2
//#define FCS_FORMAT 					3
#define SYNC_FORMAT 				4

struct sPacketBufHdr
{
	struct sBuffHdr h;	/**< message buffer header */
	unsigned long rsvd; /**< reserved for message level information */ 
	unsigned char flags;	/**< reserved for message level information */
	unsigned char type; /**< packet type */
	unsigned short format; /**< packet format identifier */
	size_t doffset; /**< data chunk offset */
	size_t dlen;	/**< data chunk length */
	size_t offset;	/**< data offset in buffer */
};

typedef struct sPacketBufHdr PACKETBUF_HDR;

#define PACKETBUF_DATA(ptr) (unsigned char *)(&((PACKETBUF_HDR *)ptr)[1])
#define PACKETBUF_OFFSET_DATA(ptr,offset) &((PACKETBUF_DATA(ptr))[offset])

PACKETBUF_HDR *getPacketBuffer(void *memPool, unsigned char flags, unsigned char type, unsigned short format, unsigned short doffset);
PACKETBUF_HDR *getPacketBufferWithWait(void *memPool, unsigned char flags, unsigned char type, unsigned short format, unsigned short doffset, unsigned int timeout);

#endif


