/**
* @file membuf.c
* @brief Equal size memory buffers system
*
* @author Eli Schneider
*
* @version 0.0.2
* @date 15.05.2012
*/
#include <stddef.h>
#include <assert.h>
#include "sysport.h"
#include "membuf.h"

int initMemBufPool(MEMBUF_POOL *pool, void *memArea, size_t memSize, size_t bufSize, size_t bufNum)
{
	struct sMemBufHeader *buf;
	uint32_t key;
	
	key=__disableInterrupts();
	pool->freeList=NULL;
	pool->bufSize=bufSize;
	pool->freeBuf=0;
	pool->nBuf=0;
	if (bufSize==0 || memSize<bufSize)
	{
		__restoreInterrupts(key);
		return -1;
	}
	pool->freeList= (struct sMemBufHeader *)memArea;
	if (memArea!=NULL)
	{
		buf=(struct sMemBufHeader *)memArea;
		pool->nBuf++;
		
		memSize-=(bufSize+MEM_BUF_HEADER_SIZE);
		while ((bufSize+MEM_BUF_HEADER_SIZE)<=memSize)
		{
			buf->link=((char *)buf)+(bufSize+MEM_BUF_HEADER_SIZE);
			pool->nBuf++;
			memSize-=(bufSize+MEM_BUF_HEADER_SIZE);
			buf=(struct sMemBufHeader *)(buf->link);
		}
		buf->link=NULL;
		pool->freeBuf=pool->nBuf;
	}
	__restoreInterrupts(key);
	return 0;
}

void *getMemBuf(MEMBUF_POOL *pool)
{
	register struct sMemBufHeader *buf;
	register uint32_t key;
	
	key=__disableInterrupts();
	
	if (pool->freeList)
	{
		buf=pool->freeList;
		pool->freeList=(struct sMemBufHeader *)buf->link;
		#ifdef MEMBUF_GET_CHECK_BOUNDS
		assert((uint32_t)pool->freeList!=0xa5a5a5a5);
		#endif
		pool->freeBuf--;
		__restoreInterrupts(key);
		buf->link=pool;
		return buf+1;//buf+1;
	}
	else 
	{
		__restoreInterrupts(key);
		return NULL;
	}
}

int retMemBuf(void *buf)
{
	register struct sMemBufHeader *b;
	register MEMBUF_POOL *pool;
	register uint32_t key;

	if (buf==NULL)
		return	0;
	b=((struct sMemBufHeader *)buf)-1;//buf)-1;
	if (b->link==NULL)
		return -1;
	
	pool=(MEMBUF_POOL *)b->link;
	key=__disableInterrupts();
	b->link=pool->freeList;
	pool->freeList=b;
	#ifdef MEMBUF_RET_CHECK_BOUNDS
	assert((uint32_t)pool->freeList!=0xa5a5a5a5);
	#endif
	pool->freeBuf++;
	__restoreInterrupts(key);
	return 0;
}


