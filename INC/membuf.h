/**
* @file membuf.h
* @brief Equal size memory buffers system
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 26.05.2010
*/
#ifndef _MEMBUF_H
#define _MEMBUF_H

#include <stddef.h>

#ifdef MEMBUF_POOL_STAT
struct sMemPoolStat 
{
	
};
#endif

struct sMemBufHeader
{
	void *link;	/**< In memory pool free list -pointer to next buffer. When allocated - pointer to memory pool. NULL points to nothing */
};

#define MEM_BUF_HEADER_SIZE	(sizeof(struct sMemBufHeader))

struct sMemBufPool
{
	struct sMemBufHeader *freeList; /**< Memory pool free list. NULL points to nothing */
	size_t	bufSize; /**< Memory pool buffer size */	
	size_t	nBuf;	/**< Number of buffers in pool */	
	size_t	freeBuf;	/**< Number of free buffers in pool */	
};

typedef struct sMemBufPool MEMBUF_POOL;

int initMemBufPool(MEMBUF_POOL *, void *, size_t, size_t , size_t);
void *getMemBuf(MEMBUF_POOL *);
int retMemBuf(void *);

static inline size_t memPoolBuffers(MEMBUF_POOL *pool)
{
	return pool->nBuf;
}

static inline size_t memPoolFreeBuffers(MEMBUF_POOL *pool)
{
	return pool->freeBuf;
}

static inline MEMBUF_POOL *membufPool(void *buf)
{
	return (buf==NULL) ? NULL : (((struct sMemBufHeader *)buf)-1)->link;
}

#endif /* _MEMBUF_H */


