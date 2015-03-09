/*******************************************************************************
 * @file Hcmdtask.c
 * @ Host command interpriter task 
 *
 * @author Andrei Mamtsev
 *
 * @version 0.0.1
 * @date 09.02.2013
 *
*******************************************************************************/
/**************************************************************************/
/* Standard Includes */
/**************************************************************************/
#include <stddef.h>
#include <stdint.h>
/**************************************************************************/

/**************************************************************************/
/* Library Includes */
/**************************************************************************/


/**************************************************************************/
/* Driver includes */
/**************************************************************************/



/**************************************************************************/
/* Src includes */
/**************************************************************************/
#include "msg_type.h"
#include "membuf.h"
#include "hostframe.h"
#include "packetbuf.h"
#include "pbuffer.h"
#include "hcmd.h"
#include "hcmdtask.h"
//#include "data.h"
#include "main.h"
#include "IntHost.h"



/**************************************************************************/
/*Declaration of global variables*/
/**************************************************************************/
xQueueHandle hCmdMbx=NULL;


#ifdef TASK_STACK_CHECK
struct sHighWaterMark HighWaterMark;
#endif
/**************************************************************************/
/*Extern Declarations*/
/**************************************************************************/

struct sHostInterface intHost;

/**
* @fn void hCmdTask(void *para)
*
*  Host command interpriter task
*
* @author Andrei Mamtsev
*
* @param void *para
*
* @return void
*
* @date 17.02.2013
*/
void hCmdTask(void *para)
{
	MSG_HDR cmd_in_msg;
	PACKETBUF_HDR *p;

	
	#ifdef TASK_STACK_CHECK
	unsigned portBASE_TYPE uxHighWaterMark;
	
	/* Inspect our own high water mark on entering the task. */
	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
	#endif

	#ifdef KUKU
	/* Initialize  interface queue*/
	hCmdMbx = xQueueCreate( HCMD_QUEUE_SIZE, sizeof(MSG_HDR) );
	if( hCmdMbx == NULL )
	{
		// Failed to create the queue.
		while(1)
		{
			vTaskSuspend(NULL);
			vTaskDelay(10);
		}
	}
	#endif
	
	
	//initCommandProcessor();	
	
	initHostInterface(&intHost, 0);
	
	for (;;)
	{
		if (xQueueReceive(hCmdMbx,&cmd_in_msg,portMAX_DELAY))
		{
			if (cmd_in_msg.hdr.bit.type==MSG_TYPE_PACKET)
			{
				p=(PACKETBUF_HDR *)cmd_in_msg.buf;
				if (p)
				{
					if (cmd_in_msg.hdr.bit.source==MSG_SRC_HOSTRX)
					{        					
						if(processHostCommand(p,0))
							retMemBuf(p); // Command processor signalled to return the command buffer now
					}
					else if (cmd_in_msg.hdr.bit.source==MSG_SRC_INTERP)
					{     
							if(processDriveForward(p,cmd_in_msg.data))
							retMemBuf(p); // Forward Driver responce to ETH
					}
					else
						retMemBuf(p);
				}
			}
			else if (cmd_in_msg.hdr.bit.type==MSG_TYPE_CMD)
			{
				// for now do nothing
			}
			#ifdef TASK_STACK_CHECK
			/* Calling the function will have used some stack space, we would therefore now expect
			uxTaskGetStackHighWaterMark() to return a value lower than when it was called on
			entering the task. */
			HighWaterMark.uxHighWaterMark_hCmdTask=uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
			#endif
		}
	}
}

/**
* @fn int sendPacketToCmd(void *packet, uint16_t hdr, uint32_t timeout)
*
*  Send packet to hcmd task
*
* @author Andrei Mamtsev
*
* @param void *packet - pointer to packet
* @param uint16_t hdr - header of  packet
* @param uint32_t timeout - timeout
*
* @return void
*
* @date 17.02.2013
*/

int sendPacketToCmd(void *packet, uint16_t hdr, uint32_t timeout)
{
	MSG_HDR msg;
	
	msg.hdr.all=hdr;
	msg.data=0;
	msg.buf=packet;
	return xQueueSend(hCmdMbx,&msg,timeout);
}


