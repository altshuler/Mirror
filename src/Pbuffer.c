#include <stdint.h>
#include <stddef.h>
#include "freertos.h"
#include "task.h"
#include "membuf.h"
#include "pbuffer.h"


PAYLOAD_BUFFER_HEADER *getPayloadBuffer(MEMBUF_POOL *pool)

{
	PAYLOAD_BUFFER_HEADER *p;

	p=(PAYLOAD_BUFFER_HEADER *)getMemBuf(pool);
	if (p)
	{
		p->link=NULL;
		p->length=0;
	}
	return p;
}


PAYLOAD_BUFFER_HEADER *getPayloadBufferWithWait(MEMBUF_POOL *pool, uint32_t timeout)
{
	PAYLOAD_BUFFER_HEADER *p=NULL;

	if (timeout==0)
		p=getPayloadBuffer(pool);
	else 
	{
		while ((p=getPayloadBuffer(pool))==NULL)
		{
			vTaskDelay(1);
			if (timeout!=portMAX_DELAY)
			{
				timeout--;
				if (timeout==0)
					break;
			}
		}
	}


	return p;
}

