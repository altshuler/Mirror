/**
* @file dev.c
* @brief general device driver 
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 08.05.2012
*/
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <dev.h>
#include "_dev.h"
#include <iodev.h>
#include <freertos.h>
#include <task.h>
#include <sysport.h>
#include <syserr.h>
		
DEV_Attrs DEV_ATTRS = {
	0,				/* device id */
	NULL,	/* params */
	#ifdef KUKU
	0,
	#endif
	NULL
};


/*
 *	======== DEV_init ========
 */
void DEV_init(void)
{
    DEV_TableElem *objDevHead;
    DEV_TableElem *objDev;
    DEV_Device *dptr;
    IODEV_Fxns *fxns;
    int status;
    int i;

    /* Hook up the linked list ... */
    for (i = 0; i < _DEV_numStaticDevs; i++) {
        QUE_put(&DEV_table, &(_DEV_staticDevTable[i]));
    }

    /*
     *  For each device driver, call its DXX_init function *before* the
     *  statically created streams are "opened" (calling the device's
     *  open function for each static stream below).
     */
    for (i = (int) _DEV_numStaticDevs - 1; i >= 0; i--) {
        if (_DEV_initFxn[i] != NULL) {
           (_DEV_initFxn[i])();
        }
    }

    /*
     *  Call IODEV bind device function (iodBindDev) if driver is of type IODEV
     */
    objDevHead = (DEV_TableElem *)&DEV_table;
    for(objDev = (DEV_TableElem *) QUE_next((void *)objDevHead);
                objDev != objDevHead;
                objDev = (DEV_TableElem *) QUE_next((void *)objDev)) {

        dptr = &objDev->device;

		#ifdef KUKU
        if (dptr->type == DEV_IOMTYPE ) 
		#endif
		{
            fxns = (IODEV_Fxns *)dptr->fxns;
            status = fxns->iodBindDev(&dptr->devp, dptr->devid, dptr->params);

            if (status != IODEV_COMPLETED) 
			{
                //SYS_abort("ERROR - Device %s Config Failed", dptr->name);
            }
        }
    }
}


int DEV_ebadio(DEV_Handle dev)
{
	return E_BADIO;
}

char *DEV_match(char *name, DEV_Device **driver)
{
    DEV_TableElem *objDevHead = (DEV_TableElem*) &DEV_table;
    DEV_TableElem *objDev;
    DEV_Device *dptr;
    int len;

    /*
     * Trace the existence of device through OBJ_table[OBJ_DEV].
     * If successfull *dptr points to the device entry.
     */

    for (objDev = (DEV_TableElem *)QUE_next((void *)objDevHead); objDev != objDevHead;
        objDev = (DEV_TableElem *)QUE_next((void *)objDev)) 
	{
        dptr = &objDev->device;
        len = strlen(dptr->name);
        if ( (len == 0) || (strncmp(name,dptr->name,len) == 0) ) 
		{
            /* If driver exists in the devicetable, point the *driver
               to corresponding device entry */
            *driver = dptr;
            return(name + len);
        }
    }

    *driver = NULL;
    return (name);

}

void DEV_find(char *name, DEV_Device **driver)
{
    DEV_TableElem *objDevHead = (DEV_TableElem*) &DEV_table;
    DEV_TableElem *objDev;
    DEV_Device *dptr;

    /*
     * Do the exact match, return device entry if successfull.
     */

    for (objDev = (DEV_TableElem *)QUE_next((void *)objDevHead); objDev != objDevHead;
        objDev = (DEV_TableElem *)QUE_next((void *)objDev)) {
        dptr = &objDev->device;
        if ( strcmp(name,dptr->name) == 0 ) {
            /* If driver exists in the devicetable, point the *driver
               to corresponding device entry */
            *driver = dptr;
            return;
        }
    }

    *driver = NULL;
    return;
}

DEV_Frame *DEV_mkframe(unsigned int size, unsigned int align)
{
    DEV_Frame   *frame;

    if ((frame = pvPortMalloc(sizeof(DEV_Frame))) == NULL) 
	{
        return (NULL);
    }

    /* don't allocate frame buffer if size is zero */
    if (size > 0) 
	{
        if ((frame->addr = pvPortMalloc(size)) == NULL) 
		{
            vPortFree(frame);
            return (NULL);
        }
    }

    frame->size = size;

    return (frame);
}

void DEV_rmframe(DEV_Frame *frame)
{
	if (frame==NULL)
		return;
	
    if (frame->size > 0) 
	{
        /* free buffer */
        vPortFree(frame->addr);
    }

    /* free object */
    vPortFree(frame);
}

int DEV_createDevice(char *name, void *fxns, void (*initFxn)(), DEV_Attrs *attrs)
{
    DEV_TableElem *objDevHead = (DEV_TableElem*) &DEV_table;
    DEV_TableElem *objDev, *objEntry;
    DEV_Device *dptr, *entry;
    IODEV_Fxns *iomfxns;
    int status;
    uint32_t key;

    /*
     * Crate a device entry, if not successful return
     * E_ALLOC. 
     */

    objEntry = pvPortMalloc(sizeof(DEV_TableElem));

    if (objEntry == NULL) 
        return(E_ALLOC);
    else
		memset(objEntry, 0, sizeof(DEV_TableElem));

	vTaskSuspendAll();
    //TSK_disable();


    /*
     * Check if device already exists in the Device table, if yes return
     * E_INVAL
     */
    DEV_find(name, &entry);

    if (entry != NULL) {
		xTaskResumeAll();
        //TSK_enable();
        vPortFree(objEntry);
        //SYS_error("DEV", SYS_EINVAL);
        return(E_INVAL);
    }

    /*
     * Initialize new device entry(DEV_Device) in the OBJ table with
     * the parameters passed to API
     */
    entry = &objEntry->device;
    entry->name = name;
    entry->fxns = fxns;

    if (attrs == NULL) {
        attrs = &DEV_ATTRS;
    }
    entry->devid  = attrs->devid;
    entry->params = attrs->params;
	#ifdef KUKU
    entry->type   = attrs->type;
	#endif
    entry->devp   = attrs->devp;

    /*
     * Call the Device init function if its not NULL, with interrupts
     * disabled.
     */
    if (initFxn != NULL) {
        key = __disableInterrupts();
        (*initFxn)();
        __restoreInterrupts(key);
    }

    /*
     * Call IO driver function
     * iodBindDev with interrupts disabled.
     */
    #ifdef KUKU
    if (entry->type == DEV_IODEVTYPE)
	#endif
 	{
 		iomfxns = (IODEV_Fxns *) entry->fxns;

        key = __disableInterrupts();
        status = iomfxns->iodBindDev(&entry->devp, entry->devid,
                                     entry->params);
        __restoreInterrupts(key);

        if (status != IODEV_COMPLETED) 
		{
			xTaskResumeAll();
            //TSK_enable();

            /* Delete the just created device entry in device table */
            vPortFree(objEntry);

            //SYS_error("DEV",E_BADIO);

            return(status);
        }
    }

    /*
     * Device is ready for addition into OBJ_Table. Check new device
     * name length against existing device name lengths. If length of
     * new device is greater than one in OBJ_table, mark the location
     * and insert device ahead of device whose name length is shorter
     * else add it to the end.
     *
     * This will keep all the devices sorted in descending order, which is
     * required to pass additional parameters along with device name in 
     * DEV_open()
     */

    objDev = (DEV_TableElem *)QUE_next((void *)objDevHead);
    while (objDev != objDevHead) {
        dptr = &objDev->device;
        if (strlen(name) > strlen(dptr->name)) {
            break;
        }
        objDev = (DEV_TableElem *)QUE_next((void *)objDev);
    }

    /* Insert objEntry ahead of objDev */
    QUE_insert(objDev, objEntry);

	xTaskResumeAll();
    //TSK_enable();

    return(E_OK);
}

int DEV_deleteDevice(char *name)
{
    DEV_TableElem *objDev;
    DEV_Device *entry;
    IODEV_Fxns *iomfxns;
    int status = E_OK;
    uint32_t key;

    /* Check if device   exists in the Device table, if not return FALSE */
    DEV_find(name, &entry);
    if (entry == NULL) {
        //SYS_error("DEV", E_NODEV);
        return(E_NODEV);
    }

    /*
     * If device to be deleted is of type IODEV call iodUnBindDev with
     * interrupts disabled
     */
    #ifdef KUKU
    if (entry->type == DEV_IODEVTYPE) 
	#endif
	{
        iomfxns = (IODEV_Fxns *)entry->fxns;

        key = __disableInterrupts();
        status = iomfxns->iodUnBindDev(entry->devp);
        __restoreInterrupts(key);

        if (status != IODEV_COMPLETED) 
		{
            //SYS_error("DEV", E_BADIO);
        }
        else {
            status = E_OK;
        }

    }

    /* Free Device entry in the device table */
    objDev = (DEV_TableElem *)((char *)entry - sizeof(QUE_Elem));
    QUE_remove(objDev);
    vPortFree(objDev);


    return(status);
}

