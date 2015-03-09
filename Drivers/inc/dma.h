/**
* @file dma.h
* @brief dma manager
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 21.05.2012
*/

#ifndef _DMA_H
#define _DMA_H


#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TX_DMA	0
#define RX_DMA	1

struct sDmaParam
{
	uint16_t	dmacId;
	uint16_t	stream;
	uint16_t	channel;
};

struct sDmaResSet
{
	struct sDmaParam set[2];

};

#ifdef __cplusplus
}
#endif


#endif



