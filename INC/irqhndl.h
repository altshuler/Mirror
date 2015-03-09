/**
* @file irqhndl.h
* @brief generic irq handlers
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 16.04.2012
*/
#ifndef _IRQHNDL_H
#define _IRQHNDL_H



#include "stm32f2xx.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*IRQ_HNDL_T)(void *arg);

extern IRQ_HNDL_T irqHandler[];
extern void *irqHandlerArg[];

void initIrqHandlerTable(void);
void installInterruptHandler(uint16_t vecnum, void *handler, void *arg);
void uninstallInterruptHandler(uint16_t vecnum);

#ifdef __cplusplus
}
#endif



#endif
