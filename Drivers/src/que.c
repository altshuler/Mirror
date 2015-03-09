/**
* @file que.c
* @brief queue module
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 07.05.2012
*/
#include <stddef.h>
#include "que.h"
#include <freertos.h>
#include <sysport.h>

QUE_Attrs QUE_ATTRS={0};

/*
 *  ======== QUE_create ========
 */
QUE_Handle QUE_create(QUE_Attrs *attrs)
{
	QUE_Handle q;

	void *pvPortMalloc( size_t xWantedSize );
	q=(QUE_Handle)pvPortMalloc(sizeof(QUE_Elem));
	if (q)
		QUE_new(q);
	return q;
}




/*
 *  ======== QUE_get ========
 *  disable interrupts and returns the first element in the queue.
 */
void *QUE_get(QUE_Handle queue)
{
	uint32_t psr;
	void *p;

	psr=__disableInterrupts();
	p=QUE_dequeue(queue);
	__restoreInterrupts(psr);
	return p; 
}


/*
 *  ======== QUE_print ========
 */
void QUE_print(QUE_Handle queue)
{
}

/*
 *  ======== QUE_put ========
 *  Disable interrupts and put "elem" at end of "queue".  
 */
void QUE_put(QUE_Handle queue, void *elem)
{
	uint32_t psr;

	psr=__disableInterrupts();
	QUE_enqueue(queue, elem);
	__restoreInterrupts(psr);
}

