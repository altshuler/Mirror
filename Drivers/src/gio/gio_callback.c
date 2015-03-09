/**
* @file gio_callback.c
* @brief general io driver blocking callback function
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 08.05.2012
*/

#include <stddef.h>
#include <gio.h>
#include <iodev.h>
#include <que.h>

#include "_gio.h"


/*
 *  ======== _GIO_iodevCallback ========
 *  This function is called by the mini-driver when I/O completes.
 */
void _GIO_iodevCallback(void *cbArg, IODEV_Packet *packet)
{
    GIO_Handle          gioChan = (GIO_Handle)cbArg;
    GIO_AppCallback     *appCallback = (GIO_AppCallback *)packet->misc;
    int                 status;
    void                *addr;
    size_t              size;
 
    if (appCallback == NULL) {
        /* this was a synchronous call -- post semaphore (or alternate sync) */
        GIO->SEMPOST(gioChan->syncObj);
    }
    else {
        status = packet->status;
        addr = packet->addr;
        size = packet->size;
        
        /* recycle packet back onto free list */
        QUE_putJ(&gioChan->freeList, packet);

        /* callback into application with status and size */
        (*appCallback->fxn)(appCallback->arg, status, addr, size);
    }
}

