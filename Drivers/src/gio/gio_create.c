/**
* @file gio_create.c
* @brief general io driver creation
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 08.05.2012
*/

#include <stddef.h>
#include <string.h>
#include <dev.h>
#include <freertos.h>
#include <que.h>

#include <gio.h>
#include <iodev.h>

#include "_gio.h"


/*
 *  ======== GIO_create ========
 */
GIO_Handle GIO_create(char *name, int mode, int *status, void *optArgs, GIO_Attrs *attrs)
{
	GIO_Handle  gioChan;
	IODEV_Packet  *packet;
	DEV_Device  *entry;
	int         i;
	int         tmpStat;

	if (attrs == NULL) 
		attrs = &GIO_ATTRS;

	/*
	* status param is used to pass additional device status back to caller.
	*/
	if (status == NULL) 
		status = &tmpStat;	  /* no longer need to check if status valid ptr */

	*status = IODEV_COMPLETED;

	/*
	*  Find device structure in device table for device with name 'name'.
	*  DEV_match() returns the remaining name string for use by the
	*  mini-driver's create() function.
	*/
	name = DEV_matchJ(name, &entry);
	if (entry == NULL) 
	{
		//SYS_error(name, SYS_ENODEV); /* sys error - no device found */
		return (NULL);
	}
	#ifdef KUKU
	if (entry->type != DEV_IODEVTYPE) 
	{
		//SYS_error("IOM", SYS_EINVAL); /* sys error - invalid device parameter */
		return (NULL);
	}
	#endif

	/*  allocate and 0-fill IODEV object */
	gioChan = (GIO_Handle)pvPortMalloc(sizeof(GIO_Obj));
	if (gioChan == NULL) 
	{
		*status = IODEV_EALLOC;  
		return (NULL);
	}
	else 
		memset(gioChan, 0, sizeof(GIO_Obj));

	/* initialize queue structures */
	QUE_new(&gioChan->freeList);

	/*
	* Allocate packets for asynch I/O.
	*/
	for (i=0; i < attrs->nPackets; i++) 
	{
		packet = _GIO_mkPacket();
		if (packet == NULL) 
		{
			*status = IODEV_EALLOC;
			GIO_deleteJ(gioChan);
			return (NULL);
		}
		QUE_putJ(&gioChan->freeList, packet);
	}

	/*
	* Create semaphore or other synchronization object.  'gioChan->syncObj' is
	* used to wait for I/O to complete when GIO_submit() is called with
	* NULL *appCallback parameter. 
	*/
	gioChan->syncObj = GIO->SEMCREATE(1, NULL);

	if (gioChan->syncObj == NULL) 
	{
		*status = IODEV_EALLOC;
		GIO_deleteJ(gioChan);
		return (NULL);
	}

	gioChan->fxns = (IODEV_Fxns *)entry->fxns;
	gioChan->mode = mode;
	gioChan->timeout = attrs->timeout;

	*status = gioChan->fxns->iodCreateChan(&gioChan->iodChan, entry->devp,
								name, mode, optArgs, _GIO_iodevCallback, gioChan);

	if (gioChan->iodChan == NULL) 
	{
		GIO_deleteJ(gioChan);
		return (NULL);
	}

	return (gioChan);
}

