/**
* @file irqhndl.c
* @brief generic irq handlers table
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 16.04.2012
*/

#include <stddef.h>
#include <stdint.h>
#include <sysport.h>
#include "irqhndl.h"

#define MAX_IRQn (HASH_RNG_IRQn+1)

IRQ_HNDL_T irqHandler[MAX_IRQn];
void *irqHandlerArg[MAX_IRQn];

static int defIrqHandler(void *);

void initIrqHandlerTable(void)
{
	uint32_t key;
	uint16_t i;
	
	key=__disableInterrupts();
	for (i=0;i<MAX_IRQn;i++)
	{
		irqHandler[i]=defIrqHandler;
		irqHandlerArg[i]=NULL;
	}
	__restoreInterrupts(key);
}

void installInterruptHandler(uint16_t vecnum, void *handler, void *arg)
{
	uint32_t key;
	
	if (vecnum<MAX_IRQn)
	{
		key=__disableInterrupts();
		irqHandler[vecnum]=(IRQ_HNDL_T)handler;
		irqHandlerArg[vecnum]=arg;
		__restoreInterrupts(key);
	}
}

void uninstallInterruptHandler(uint16_t vecnum)
{
	uint32_t key;
	
	if (vecnum<MAX_IRQn)
	{
		key=__disableInterrupts();
		irqHandler[vecnum]=defIrqHandler;
		irqHandlerArg[vecnum]=NULL;
		__restoreInterrupts(key);
	}
}

int getInterruptHandler(uint16_t vecnum, void **pHandler, void **pArg)
{
	if (MAX_IRQn<=vecnum)
		return -1;
	if (pHandler!=NULL)
		*pHandler=irqHandler[vecnum];
	if (pArg!=NULL)
		*pArg=irqHandlerArg[vecnum];
	
	return 0;
}

int defIrqHandler(void *arg)
{
	for (;;);
	return 0;
}


