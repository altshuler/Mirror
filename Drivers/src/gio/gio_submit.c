/**
* @file gio_submit.c
* @brief general io driver in/out
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 08.05.2012
*/




#include <stddef.h>
#include <stdint.h>
#include <que.h>

#include <gio.h>
#include <iodev.h>

#include "_gio.h"



/*
 *  ======== GIO_submit ========
 */
int GIO_submit(GIO_Handle gioChan, unsigned int cmd, void *bufp, size_t *psize, GIO_AppCallback *appCallback)
{
    int         status;
    int        semStat;
    IODEV_Packet  *packet;

    if (appCallback == NULL) 
	{
        /* synchronous operation, use dedicated packet */
        packet = &gioChan->syncPacket;
    }
    else 
	{
        /* asynchronous operation, get packet from freelist */
        packet = QUE_get(&gioChan->freeList);
        if (packet == (IODEV_Packet *)(&gioChan->freeList)) 
		{
            return (IODEV_ENOPACKETS);
        }
    }

    /* initialize size variable if psize == NULL */
    if (psize == NULL) 
	{
        packet->size = 0;
        psize = &packet->size;
    }

    packet->cmd = cmd;
    packet->addr = bufp;
    packet->size = *psize;
    packet->status = IODEV_COMPLETED;
    /* 
     * 'appCallback' will be NULL for synchronous calls. 
     * 'packet->misc' is used in callback function to call callback (async)
     * or post semaphore (sync).
     */
    packet->misc = (unsigned int)appCallback;

    /* call down into mini-driver */
    status = gioChan->fxns->iodSubmitChan(gioChan->iodChan, packet);


    if ((status == IODEV_COMPLETED) || (status < 0)) 
	{
        if (status == IODEV_COMPLETED) 
		{
            *psize = packet->size;
            status = packet->status;
        }

        /* If async then place packet back on free list */    
        if (appCallback != NULL) 
		{
            QUE_putJ(&gioChan->freeList, packet);
        }

        return (status);
    }

    /*
     * Call SEMPEND Fxn only if synchronous i/o and no error returned
     *   from mdSubmitChan().
     */
    if (appCallback == NULL) 
	{

        if (status < 0) 
		{    /* error occured */
            *psize = 0;
            return (status);
        }

        /* synchronous I/O -- call global blocking function */
        semStat = GIO->SEMPEND(gioChan->syncObj, gioChan->timeout);

        if (semStat) 
		{
            *psize = packet->size;
            status = packet->status;
        }
        else 
		{    /* timeout occurred */
            *psize = 0;
            
            /* 
	             * NOTE: A channel timeout needs special handling. Timeouts are
	             * usually due to some serious underlying device or system state
	             * and may require the channel, or possibly the device,to be reset.
	             * Because the mini-driver may still own the IODEV_Packet here
	             * driver's will need to perform timeout processing. We will call
	             * the mini-driver's control fxn with the IODEV_CHAN_TIMEDOUT command
	             * code.
	             */
			if ((status = gioChan->fxns->iodControlChan(gioChan->iodChan, IODEV_CHAN_TIMEDOUT, NULL)) != IODEV_COMPLETED)
			{ 
				return (IODEV_ETIMEOUTUNREC); /* Fatal: may have lost IOP */
			}

			return (IODEV_ETIMEOUT);
        }
    }

    return (status);
}



