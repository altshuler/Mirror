/**
* @file st_dma.h
* @brief dma manger
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 21.05.2012
*/
#ifndef _ST_DMA_H
#define _ST_DMA_H

#include <stddef.h>
#include <stdint.h>
#include <freertos.h>
#include <semphr.h>
#include "stm32f2xx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_DMA_RESOURCE_SETS	8

struct sDmaStreamDmacInfo
{
	DMA_Stream_TypeDef *pStream[8];
	uint16_t nIrq[8];
};

struct sDmaStreamInfo
{
	struct sDmaStreamDmacInfo dmac[2];
};

struct sDmaSet
{
	struct sDmaSet *link;
	uint16_t stream;		/**< dma stream 0..7 */
	uint16_t channel;		/**< dma channel 0..7 */
};


typedef struct sDMA_Resource
{
	xSemaphoreHandle sem;		/**< Allocation semaphore */
	int16_t allocResourceId;		/** < Allocated resource identifier */
	struct sDmaSet *set[MAX_DMA_RESOURCE_SETS]; 
} DMA_RESOURCE, *DMA_RESOURCE_HANDLE;

typedef struct sDMA_ResourceSet
{
	DMA_RESOURCE_HANDLE dmaRes[MAX_DMA_RESOURCE_SETS];
} DMA_RESOURCE_SET;

void initDmaManager(void);
DMA_RESOURCE *allocDmaResource(uint16_t dmacID, uint16_t stream, uint16_t channel, portTickType timeout, signed portBASE_TYPE *pxHigherPriorityTaskWoken);
int freeDmaResource(DMA_RESOURCE *hDma, signed portBASE_TYPE *pxHigherPriorityTaskWoken);
DMA_RESOURCE *findDmaResource(uint16_t dmacID, uint16_t stream, uint16_t channel, int16_t *id);

extern const struct sDmaStreamInfo dmaStreamInfo;
extern const uint32_t dmaChanTable[];
extern const uint32_t dmaTCflagid[];




#ifdef __cplusplus
}
#endif


#endif

