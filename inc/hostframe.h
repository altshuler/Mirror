#ifndef _HOSTFRAME_H
#define _HOSTFRAME_H

#include <stdint.h>
#include "pbuffer.h"
//#include "crc16.h"

//#define HOST_FRAME_MAGIC_NUMBER	0x5D493C8BUL

struct sHostFrameHeader
{
	uint8_t length;			// frame data length in bytes (0 to 65365). 
	//uint8_t CR;	
	//uint16_t frameNum;		// Frame number 
};

typedef struct sHostFrameHeader HOST_FRAME_HEADER;

#define HOST_FRAME_RESULT(p) &((char *)(p))[sizeof(HOST_FRAME_HEADER)]
#define HOST_SFRAME_START(p) &((char *)(p))[sizeof(uint8_t)]

#define HOST_FRAME_TYPE(p) &((char *)(p))[1]

struct sHostFrameTrailer
{
	uint8_t eof;				// Frame check sequence -CRC16.
};

typedef struct sHostFrameTrailer HOST_FRAME_TRAILER;

int validHostFrame(HOST_FRAME_HEADER *hostFrame, size_t max_size);
PAYLOAD_BUFFER_HEADER *extractPayloadsFromHostFrame(HOST_FRAME_HEADER *frame, PAYLOAD_BUFFER_HEADER *cmd_reminder, PAYLOAD_BUFFER_HEADER **pcmd_reminder, MEMBUF_POOL *payloadPool);

#ifdef KUKU
static inline uint16_t calcHostFrameFcs(void *hostFrame)
{
	//return crc16(0x0000, (unsigned char *)hostFrame,sizeof(HOST_FRAME_HEADER)+((HOST_FRAME_HEADER *)hostFrame)->length);
}

static inline int checkHostFrameCCS(void *hostFrame, uint16_t e_val)
{

	return (crc(/*0x0000*/e_val, (unsigned char *)hostFrame,((HOST_FRAME_HEADER *)hostFrame)->length)==0) ? 1 : 0;
}
#endif


#define setHostFrameNum(p,n) ((HOST_FRAME_HEADER *)(p))->frameNum=(uint16_t)(n)
#define setHostFrameFcs(p,len, ch) ((HOST_FRAME_TRAILER *)((HOST_FRAME_RESULT(p))+len))->eof=(uint8_t)(ch)

//#define setHostFrameFcs(p, crc) (((HOST_FRAME_TRAILER *)(&(HOST_FRAME_DATA(p)[((HOST_FRAME_HEADER *)(p))->length])))->fcs)=(uint16_t)(crc)



#endif

