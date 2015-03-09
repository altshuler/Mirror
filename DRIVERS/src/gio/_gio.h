/**
* @file _gio.h
* @brief GIO private header file
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 07.05.2012
*/
#ifndef __GIO_H
#define __GIO_H


#include <iodev.h>
#include <freertos.h>


#ifdef __cplusplus
extern "C" {
#endif

#define _GIO_mkPacket()         pvPortMalloc(sizeof(IODEV_Packet));
#define _GIO_rmPacket(packet)   vPortFree(packet);

/*
 * IO-driver's i/o completion callback routine.
 */
extern void _GIO_iodevCallback(void *cbArg, IODEV_Packet *packet);


#ifdef __cplusplus
}
#endif


#endif



