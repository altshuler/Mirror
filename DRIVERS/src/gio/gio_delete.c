/**
* @file gio_create.c
* @brief general io driver destruction
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 24.05.2012
*/
#include <stddef.h>
#include <stdint.h>
#include <freertos.h>
#include <que.h>
	
#include <gio.h>
#include <iodev.h>
	
#include "_gio.h"
	
	
	
/*
*	======== GIO_delete ========
*/
int GIO_delete(GIO_Handle gioChan)
{
	IODEV_Packet *packet;

	/* flush and delete low-level device ... */
	if (gioChan->fxns != NULL && gioChan->iodChan != NULL)
	{
		GIO_flush(gioChan);
		gioChan->fxns->iodDeleteChan(gioChan->iodChan);
	}

	/* delete semaphore or alternate sync object ... */
	if (gioChan->syncObj != NULL) 
	{
		GIO->SEMDELETE(gioChan->syncObj);
	}

	/* free frames ... */
	packet = QUE_get(&gioChan->freeList);
	while (packet != (IODEV_Packet *)(&gioChan->freeList)) 
	{
		_GIO_rmPacket(packet);
		packet = QUE_get(&gioChan->freeList);
	}

	/* free GIO object. */
	
	vPortFree(gioChan);

	return (IODEV_COMPLETED);
}

