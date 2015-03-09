/**
* @file gio.c
* @brief general io driver 
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 08.05.2012
*/
	
#include <gio.h>
#include <freertos.h>
	
const GIO_Attrs GIO_ATTRS = {
	2,				/* nPackets */
	portMAX_DELAY,	/* timeout */
};


/*
 *	======== GIO_init ========
 */
void GIO_init()
{
}

