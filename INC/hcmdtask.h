#ifndef _HCMDTASK_H
#define _HCMDTASK_H

#include <stdint.h>

/* RTOS Includes */
/**************************************************************************/
#include "freertos.h"
#include "task.h"
#include "queue.h"


#ifdef __cplusplus
extern "C" 
{
#endif


void hCmdTask(void *para);
void hostCmdTask(void *para);
int sendPacketToCmd(void *packet, uint16_t hdr, uint32_t timeout);



#ifdef TASK_STACK_CHECK

struct sHighWaterMark
{
	unsigned portBASE_TYPE uxHighWaterMark_hCmdTask;
	unsigned portBASE_TYPE uxHighWaterMark_hTxTask;
	unsigned portBASE_TYPE uxHighWaterMark_hServerTask;
	unsigned portBASE_TYPE uxHighWaterMark_hServiceTask;
	unsigned portBASE_TYPE uxHighWaterMark_hWeightTask;
};

#endif

extern struct sHighWaterMark HighWaterMark;

#ifdef __cplusplus
}
#endif

#endif

