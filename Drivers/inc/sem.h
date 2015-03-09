/**
* @file sem.h
* @brief semaphore functions wrapper
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 08.05.2012
*/
#ifndef _SEM_H
#define _SEM_H
	
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void *SEM_create(int count, void *attrs);
void SEM_delete(void *semHandle);
int SEM_pend(void *semHandle, unsigned int timeout);
int SEM_post(void *semHandle);


#ifdef __cplusplus
}
#endif

#endif


