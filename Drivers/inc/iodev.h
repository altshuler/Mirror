/**
* @file iodev.h
* @brief io system device driver definitions
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 07.05.2012
*/
#ifndef _IODEV_H
#define _IODEV_H

#include <dev.h>

#ifdef __cplusplus
extern "C" {
#endif


/* I/O driver modes */
#define IODEV_INPUT    0x0001
#define IODEV_OUTPUT   0x0002
#define IODEV_INOUT    (IODEV_INPUT | IODEV_OUTPUT)


/*
 *  IODEV Status Codes.
 */
#define  IODEV_COMPLETED     0     /* I/O completed successfully */
#define  IODEV_PENDING       1      /* I/O queued and pending */

/* 
 * I/O request flushed. Queued writes will be completed w/ IOM_COMPLETED.
 *   Queued read requests return w/ IOM_FLUSHED.
 */
#define  IODEV_FLUSHED      2 

/*
 * I/O aborted. Non-completed read or write requests return w/ IOM_ABORTED.
 */
#define  IODEV_ABORTED      3      


/*
 * IODEV Error Codes
 */
#define  IODEV_EBADIO        -1      /* Generic failure condition */
#define  IODEV_ETIMEOUT      -2      /* Timeout occurred */
#define  IODEV_ENOPACKETS    -3      /* No packets available for I/O */
#define  IODEV_EFREE         -4      /* Unable to free resources */
#define  IODEV_EALLOC        -5      /* Unable to alloc resource */
#define  IODEV_EABORT        -6      /* I/O was aborted before completed */
#define  IODEV_EBADMODE      -7      /* Illegal device mode */
#define  IODEV_EOF           -8      /* End-of-File was encountered */
#define  IODEV_ENOTIMPL      -9      /* Operation not implemented or supported */
#define  IODEV_EBADARGS      -10     /* Illegal arguments specified */
#define  IODEV_ETIMEOUTUNREC -11     /* Unrecoverable timeout occurred */
#define  IODEV_EINUSE        -12     /* Device already in use */

/*
 *  IODEV_Packet structures are managed by the IODEV module.  IODEV packets are the
 *  basis for all I/O operations. 'cmd' field contains the command id for the
 *  mini-driver.  'status' is filled in by the mini-driver and contains the
 *  status of the commmand.
 */
typedef DEV_Frame IODEV_Packet;

/*
 *  This is the io-driver's callback function.  The io-driver will call
 *  a function of this type whenever an I/O operation completes.
 */
typedef int    (*IODEV_TiomCallback)(void * arg, IODEV_Packet *packet);

/*
 *  The following function prototypes define the io-driver functions.
 */
typedef int  (*IODEV_TiodBindDev)(void **devp, int devid, void *devParams);
typedef int  (*IODEV_TiodUnBindDev)(void * devp);
typedef int  (*IODEV_TiodControlChan)(void * chanp, unsigned int cmd, void *args);
typedef int  (*IODEV_TiodCreateChan)(void **chanp, void *devp, char *name, int mode,
                      void *chanParams, IODEV_TiomCallback cbFxn, void *cbArg);
typedef int  (*IODEV_TiodDeleteChan)(void *chanp);
typedef int  (*IODEV_TiodSubmitChan)(void *chanp, IODEV_Packet *packet);


/*
 *  Function table structure used for actual linkage between the
 *  I/O module and each mini-driver.
 */
typedef struct IODEV_Fxns
{
    IODEV_TiodBindDev      iodBindDev;
    IODEV_TiodUnBindDev    iodUnBindDev;
    IODEV_TiodControlChan  iodControlChan;
    IODEV_TiodCreateChan   iodCreateChan;
    IODEV_TiodDeleteChan   iodDeleteChan;
    IODEV_TiodSubmitChan   iodSubmitChan;
} IODEV_Fxns;

/*
 * Use this mini-driver stub fxn definition if a fxn is not implemented.
 * This fxn always returns status of IODEV_ENOTIMPL.
 */
extern int IODEV_iodNotImpl(void);

#define IODEV_BINDDEVNOTIMPL     (IODEV_TiodBindDev)IODEV_iodNotImpl
#define IODEV_UNBINDDEVNOTIMPL   (IODEV_TiodUnBindDev)IODEV_iodNotImpl
#define IODEV_CONTROLCHANNOTIMPL (IODEV_TiodControlChan)IODEV_iodNotImpl
#define IODEV_CREATECHANNOTIMPL  (IODEV_TiodCreateChan)IODEV_iodNotImpl
#define IODEV_DELETECHANNOTIMPL  (IODEV_TiodDeleteChan)IODEV_iodNotImpl
#define IODEV_SUBMITCHANNOTIMPL  (IODEV_TiodSubmitChan)IODEV_iodNotImpl


/*
 *  -------- command codes for IOM_Packet --------
 */
#define IODEV_READ        0
#define IODEV_WRITE       1
#define IODEV_ABORT       2
#define IODEV_FLUSH       3

#define IODEV_USER        128  /* 0-127 are reserved for system */

/*
 *   -------- Command codes reserved for control --------
 */
#define IODEV_CHAN_RESET          0 /* reset channel only */
#define IODEV_CHAN_TIMEDOUT       1 /* channel timeout occured */
#define IODEV_DEVICE_RESET        2 /* reset entire device */

#define IODEV_CNTL_USER   128  /* 0-127 are reserved for system */



#ifdef __cplusplus
}
#endif


#endif


