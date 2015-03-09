/**
* @file dev_data.c
* @brief device drivers data 
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 08.05.2012
*/
#include <stddef.h>
#include <que.h>
#include <dev.h>
#include "_dev.h"
	
QUE_Obj DEV_table = {(QUE_Elem *)&DEV_table, (QUE_Elem *)&DEV_table};

#ifdef KUKU
DEV_Attrs DEV_ATTRS = {
	NULL,				/* dev ID */
	NULL,				/* params */
	#ifdef KUKU
	DEV_IODEVTYPE,		/* Default is set to IODEV(fxns) */
	#endif
	NULL				/* devp */
};
#endif

