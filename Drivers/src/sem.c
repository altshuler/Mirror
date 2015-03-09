/**
* @file sem.c
* @brief semaphore functions wrapper
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 08.05.2012
*/
#include <stddef.h>
#include <freertos.h>
#include <semphr.h>
#include <sysport.h>
#include "sem.h"

void *SEM_create(int count, void *attrs)
{
	return xSemaphoreCreateCounting(count,0);
}

void SEM_delete(void *semHandle)
{
	vSemaphoreDelete(semHandle);
}

int SEM_pend(void *semHandle, unsigned int timeout)
{
	return (int)xSemaphoreTake(semHandle,timeout);
}

int SEM_post(void *semHandle)
{
	portBASE_TYPE xHigherPriorityTaskWoken= pdFALSE;

	if (inIsr())
		xSemaphoreGiveFromISR(semHandle,&xHigherPriorityTaskWoken);
	else
		xSemaphoreGive(semHandle);
	return (int)xHigherPriorityTaskWoken;
}


