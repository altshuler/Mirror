#ifndef _PBUFFER_H
#define _PBUFFER_H

#include <stdint.h>
#include "membuf.h"
#include "payload.h"

typedef struct sPayloadBufferHeader
{
	struct sPayloadBufferHeader *link;	/* Link to next buffer in list */
	uint16_t	length;					/* Length of payload in the buffer (used for payload reassembly) */
} PAYLOAD_BUFFER_HEADER;

#define PAYLOAD_BUFFER_DATA(p) &((char *)(p))[sizeof(PAYLOAD_BUFFER_HEADER)]

#ifdef __cplusplus
extern "C" 
{
#endif

PAYLOAD_BUFFER_HEADER *getPayloadBuffer(MEMBUF_POOL *pool);
#define retPayloadBuffer(p) retMemBuf(p)
PAYLOAD_BUFFER_HEADER *getPayloadBufferWithWait(MEMBUF_POOL *pool, uint32_t timeout);

#ifdef __cplusplus
}
#endif


#endif

