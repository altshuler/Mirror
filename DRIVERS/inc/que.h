/**
* @file que.h
* @brief queue module definitions
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 16.04.2012
*/
#ifndef _QUE_H
#define _QUE_H


#include <freertos.h>

/*
 *
 *
 * QUEUEs
 * ======
 * 
 *      Queues are doubly linked with dummy node to eliminate special
 *      cases for speed.
 *          _______        _______        _______      _______
 *  ,----->|_______|----->|_______|----->|_______|--->|_______|--//---,
 *  | ,----|_______|<-----|_______|<-----|_______|<---|_______|<-//-, |
 *  | |    prev           queue          elem         next          | |
 *  | |_____________________________________________________________| |
 *  |_________________________________________________________________|
 *
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct QUE_Elem {
    struct QUE_Elem *next;
    struct QUE_Elem *prev;
} QUE_Elem, *QUE_Handle;

typedef struct QUE_Elem QUE_Obj;

typedef struct QUE_Attrs {
    int dummy;
} QUE_Attrs;

extern QUE_Attrs QUE_ATTRS;

/*
 *  ======== QUE_create ========
 */
extern QUE_Handle QUE_create(QUE_Attrs *attrs);

/*
 *  ======== QUE_delete ========
 */
#define QUE_delete(queue)       vPortFree((queue))

/*
 *  ======== QUE_dequeue ========
 *
 *  get elem from front of "queue".
 *  This operation is *NOT* atomic.  External synchronization must
 *  be used to protect this queue from simultaneous access by interrupts
 *  or other tasks. 
 *
 *               _______        _______        _______      _______
 *  Before:     |_______|----->|_______|----->|_______|--->|_______|--->
 *              |_______|<-----|_______|<-----|_______|<---|_______|<---
 *               prev           queue          elem         next
 *
 *
 *               _______        _______        _______
 *  After:      |_______|----->|___*___|----->|_______|--->
 *              |_______|<-----|_______|<-----|___*___|<---
 *               prev           queue          next
 *               _______
 *      elem -->|___x___|       * = modified
 *              |___x___|       x = undefined
 *
 */
static inline void *QUE_dequeue(QUE_Handle queue)
{
    QUE_Elem *elem = queue->next;
    QUE_Elem *next = elem->next;

    queue->next = next;
    next->prev = queue;

    return (elem);
}

/*
 *  ======== QUE_empty ========
 */
#define QUE_empty(queue)        ((queue)->next == (queue))

/*
 *  ======== QUE_enqueue ========
 *
 *  put "elem" at end of "queue".
 *  This operation is *NOT* atomic.  External synchronization must
 *  be used to protect this queue from simultaneous access by interrupts
 *  or other tasks. 
 *
 *               _______        _______        _______
 *  Before:     |_______|----->|_______|----->|_______|--->
 *              |_______|<-----|_______|<-----|_______|<---
 *               prev           queue          next
 *               _______
 *      elem -->|___x___|       * = modified
 *              |___x___|       x = undefined
 *
 *               _______        _______        _______      _______
 *  After:      |___*___|----->|___*___|----->|_______|--->|_______|--->
 *              |_______|<-----|___*___|<-----|___*___|<---|_______|<---
 *               prev           elem          queue         next
 *
 */
static inline void QUE_enqueue(QUE_Handle queue, void *elem)
{
    QUE_Elem *prev = queue->prev;

    ((QUE_Elem *)elem)->next = queue;
    ((QUE_Elem *)elem)->prev = prev;
    prev->next = (QUE_Elem *)elem;
    queue->prev = (QUE_Elem *)elem;
}

/*
 *  ======== QUE_get ========
 *  disable interrupts and returns the first element in the queue.
 */
extern void *QUE_get(QUE_Handle queue);

/*
 *  ======== QUE_head ========
 */
#define QUE_head(queue)         ((void *)((queue)->next))

/*
 *  ======== QUE_init ========
 */
static inline void QUE_init(QUE_Handle queue)
{
}

/*
 *  ======== QUE_insert ========
 */
#define QUE_insert(qElem, elem)         QUE_enqueue((QUE_Handle)qElem, elem)

/*
 *  ======== QUE_new ========
 */
#define QUE_new(elem)           (elem)->next = (elem)->prev = (elem)

/*
 *  ======== QUE_next ========
 */
#define QUE_next(elem)          ((void *)((QUE_Elem *)(elem))->next)

/*
 *  ======== QUE_prev ========
 */
#define QUE_prev(elem)          ((void *)((QUE_Elem *)(elem))->prev)

/*
 *  ======== QUE_print ========
 */
extern void QUE_print(QUE_Handle queue);

/*
 *  ======== QUE_put ========
 *  Disable interrupts and put "elem" at end of "queue".  
 */
extern void QUE_put(QUE_Handle queue, void *elem);

/*
 *  ======== QUE_remove ========
 */
#define QUE_remove(elem) {\
    ((QUE_Elem *)elem)->prev->next = ((QUE_Elem *)elem)->next;  \
    ((QUE_Elem *)elem)->next->prev = ((QUE_Elem *)elem)->prev;  \
    }


#define QUE_getJ(queue)         QUE_get(queue)
#define QUE_putJ(queue, elem)   QUE_put(queue, elem)


#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /* _QUE_H */

