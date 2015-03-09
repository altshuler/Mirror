/**
* @file gio_cfg.c
* @brief general io driver default configuration
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 08.05.2012
*/

#include <stddef.h>
#include <gio.h>
#include <sem.h>

/*
 *  A pointer named 'GIO' and a global GIO_CONFIG structure will be initialized
 *  by the configuration tool to point to an GIO_Config structure.  This 
 *  structure will be referenced at run-time so that IOM will not have any
 *  hard-coded reference to SEM_pend, SEM_post, etc. This will allow IOM to
 *  be used in TSK and non-TSK based applications.
 */
const GIO_Config GIO_CONFIG = {
    (GIO_TsemCreate)SEM_create,
    (GIO_TsemDelete)SEM_delete,
    (GIO_TsemPend)SEM_pend,
    (GIO_TsemPost)SEM_post
};

const GIO_Config * const GIO = &GIO_CONFIG;

