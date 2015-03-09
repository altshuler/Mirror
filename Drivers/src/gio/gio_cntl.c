/**
* @file gio_cntl.c
* @brief general io driver control
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 08.05.2012
*/

#include <gio.h>

/*
 *  ======== GIO_control ========
 */
int GIO_control(GIO_Handle gioChan, unsigned int cmd, void *args)
{
    /* call io-driver control fxn */
    return (gioChan->fxns->iodControlChan(gioChan->iodChan, cmd, args));
}


