/**
* @file buff.h
* @brief buffers system.
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 09.01.2011
*
*/

#ifndef _BUFF_H
#define _BUFF_H

#include <stddef.h>
#include <stdint.h>
//#include "cpu_types.h"


#define E_NOBUF		(-1)	/**< Error: No available buffers in pool */
#define E_NOPOOL 	(-2)	/**< Error: No pool to return the buffer */


// Buffer callback function pointer
typedef int (*CB_BUFCALL_FN_t)(void *buf, void *arg);


struct sBuffHdr
{
	void *link;				/**< Pointer to next buffer in linked list */
	CB_BUFCALL_FN_t cbFunc; /**< Pointer to callback function */
	void *cbArg; 			/**< Pointer to callback arguments */
};

typedef struct sBuffHdr BUFF_HDR;

#define BUFF_DATA(ptr) (unsigned char *)(&((BUFF_HDR *)ptr)[1])



#endif

