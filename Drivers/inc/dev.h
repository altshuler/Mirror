/**
* @file dev.h
* @brief io system device definitions
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 07.05.2012
*/
#ifndef _DEV_H
#define _DEV_H

#include <stddef.h>
#include "que.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DE_INPUT   (0)
#define DE_OUTPUT  (1)
#define DE_MODES   (2)

typedef struct DEV_Obj *DEV_Handle;

/*
 * DEV_Frame represents hangers which hold the stream buffers
 * All the buffer exchange between IODEV/DEV happens using these frames.
 */
typedef struct DEV_Frame {      /* frame object */
	QUE_Elem link;      /* queue link */
	void *addr;           /* buffer address */
	size_t size;        /* buffer size */
	unsigned long misc;           /* miscellaneous item */
	unsigned long arg;            /* user argument */

	/* these fields are used by IODEV ... */
	unsigned int cmd;            /* command for io-driver */
	int status;         /* status of command */
} DEV_Frame;

typedef void    (*DEV_Tinit)(void);

typedef struct DEV_Callback {   /* DEV callback structure */
    int         (*fxn)();            /* function */
    void        *arg0;           /* argument 0 */
    void        *arg1;           /* argument 1 */
} DEV_Callback;

#define MAX_DEV_FUNCTIONS 8

/* 
 * DEV_Obj provides interface to DEV layer and its part of SIO object.
 */
typedef struct DEV_Obj {    /* must be first field in device object */
    QUE_Handle  todevice;   /* downstream frames go here */
    QUE_Handle  fromdevice; /* upstream frames go here */
    size_t      bufsize;    /* buffer size */
    unsigned int  nbufs;      /* number of buffers */
    //int         segid;      /* buffer segment id */
    int         mode;       /* DEV_INPUT/DEV_OUTPUT */
    int         devid;      /* device id */
    void        *params;     /* device parameters */
    void        *object;     /* pointer to device specific object */
    void    *fxns[MAX_DEV_FUNCTIONS];       /* driver function table */
    unsigned int timeout;    /* timeout for DEV_reclaim() */
    unsigned int align;      /* buffer alignment */
    DEV_Callback *callback; /* pointer to callback */
} DEV_Obj;

/* 
 * DEV_Device holds the device attributes as specified by the user
 * in the GCONF DEV template or by the DEV_createDevice API.
 */
typedef struct DEV_Device {     /* device driver specifier */
    char      *name;           /* device name */
    void        *fxns;          /* device function table */
    int         devid;          /* device id */
    void        *params;         /* device parameters */
    //unsigned int type;           /* type of the device */
    void        *devp;           /* pointer to device global data */
} DEV_Device;

typedef struct DEV_TableElem {
    QUE_Elem    qElem;
    DEV_Device  device;
} DEV_TableElem;

/* 
 * DEV_Attrs is used while creating the device dynamically
 */
typedef struct DEV_Attrs{
    int         devid;          /* device id */
    void        *params;         /* device parameters */
	#ifdef KUKU
    unsigned int type;           /* type of the device */
	#endif
    void        *devp;           /* pointer to device global data */
}DEV_Attrs;


extern void DEV_init(void);
extern int DEV_ebadio(DEV_Handle);
extern char *DEV_match(char *name, DEV_Device **driver);
extern void DEV_find(char *name, DEV_Device **driver);
extern DEV_Frame *DEV_mkframe(unsigned int size, unsigned int align);
extern void DEV_rmframe(DEV_Frame *frame);
extern int DEV_createDevice(char *name, void *fxns, void (*initFxn)(), DEV_Attrs *attrs);
extern int DEV_deleteDevice(char *name);

#define DEV_matchJ(name, driver)                DEV_match(name, driver)
#define DEV_findJ(name, driver)                 DEV_find(name, driver)
#define DEV_createDeviceJ(name, fxns, initFxn, attrs)   \
        DEV_createDevice(name, fxns, initFxn, attrs)
#define DEV_deleteDeviceJ(name)                 DEV_deleteDevice(name)

#ifdef __cplusplus
}
#endif


#endif



