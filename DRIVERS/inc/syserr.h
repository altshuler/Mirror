/**
* @file syserr.h
* @brief System error codes
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 08.05.2012
*/
#ifndef _SYSERR_H
#define _SYSERR_H

#define E_OK		0               /* no error */
#define E_ALLOC		1               /* memory allocation error */
#define E_FREE		2               /* memory free error */
#define E_NODEV		3               /* device driver not found */
#define E_EBUSY		4               /* device driver busy */
#define E_INVAL		5               /* invalid parameter for device */
#define E_BADIO		6               /* IO failure */
#define E_MODE		7               /* bad mode for device driver */
#define E_DOMAIN	8               /* domain error */
#define E_TIMEOUT	9               /* timeout error */
#define E_EOF		10              /* end-of-file */
#define E_DEAD		11              /* previously deleted object */
#define E_BADOBJ	12              /* invalid object */
#define E_NOTIMPL	13              /* action not implemented */
#define E_NOTFOUND	14              /* resource not found */

#define E_USER		256             /* user errors start here */

#endif

