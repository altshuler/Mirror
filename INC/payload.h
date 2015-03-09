#ifndef _PAYLOAD_H
#define _PAYLOAD_H

#include <stdint.h>
#include <stddef.h>

//#define PAYLOAD_MAGIC_NUMBER 0xEA214C6D

struct sPayloadHeader 
{
	//uint16_t payloadID;		// Payload identifier (type)
	uint8_t length;		// Payload data length in bytes
	//uint8_t CR;	// PAYLOAD_MAGIC_NUMBER=0xEA214C6D
};

typedef struct sPayloadHeader PAYLOAD_HEADER;

#define PAYLOAD_RESULT(p) &((char *)(p))[sizeof(PAYLOAD_HEADER)]
#define PAYLOAD_START(p) &((char *)(p))[sizeof(uint8_t)]
#define PAYLOAD_BUFFER(p) &((char *)(p))[sizeof(PAYLOAD_BUFFER_HEADER)]
//#define setFCSFrameType(p,len)  ((char *)(p)[(len+sizeof(PAYLOAD_HEADER))]
//#define setFCSFrameType(p, ch) ((uint8_t)(&(HOST_SFRAME_START(p)[((HOST_FRAME_HEADER *)(p))->length]))=(uint8_t)(ch))
#define setXCSFrameType(p, ch) *(char *)(p+(((HOST_FRAME_HEADER *)(p))->length)+sizeof(PAYLOAD_HEADER))=ch;
#define XCSFRAMETYPE(p) &((char *)(p))[sizeof(uint8_t)+(((HOST_FRAME_HEADER *)(p))->length)]
#define XCSFRAMEEND(p) &((char *)(p))[(((HOST_FRAME_HEADER *)(p))->length)]
//#define setCCSFrameType(p, ch) *(char *)(p+(((HOST_FRAME_HEADER *)(p))->length))=ch;


int validPayload(PAYLOAD_HEADER *payload, size_t actualSize);

#endif

