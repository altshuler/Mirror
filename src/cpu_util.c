/**
* @file cpu_util.c
* @brief CPU utilization measurement services
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 03.03.2013
*/
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "freertos.h"
#include "task.h"
#include "sysport.h"
#include "irqhndl.h"
#include "oshooks.h"
#include "cpu_util.h"
#include "stm32f2xx.h"

#define CPU_UTIL_PRESCALER	125

struct sFullCpuUtilHystogram 
{
	uint16_t count;
	uint16_t first;
	union 
	{
		uint32_t		s[CPU_UTIL_HYSTOGRAM_ENTRIES];
		uint32_t		u[CPU_UTIL_HYSTOGRAM_ENTRIES];
	} val;
	portTickType tick[CPU_UTIL_HYSTOGRAM_ENTRIES];
};

struct sPartialCpuUtilHystogram
{
	uint16_t count;
	uint16_t first;
	union 
	{
		uint32_t		s[CPU_UTIL_HYSTOGRAM_ENTRIES];
		uint32_t		u[CPU_UTIL_HYSTOGRAM_ENTRIES];
	} val;
};


static uint32_t lastIdleCount=0;
uint32_t idleIdleCount; // of a full hystogram
float	 reciprocalIdleIdleCount;

struct sFullCpuUtilHystogram fullCpuUtilHystogram={0, 0,{{0}},{0}};

typedef int (*FN_CPU_UTIL_TIMER_CALLBACK)(void *arg);

FN_CPU_UTIL_TIMER_CALLBACK cpuUtilTimerCB=NULL;
void *cpuUtilTimerCbArg=NULL;

static int cpuUtilTimerIsr(void *para);
static int cpuUtilTimerCallback(void *arg);
static void setCpuUtilTimerCallback(FN_CPU_UTIL_TIMER_CALLBACK cb, void *cbArg);


/**
* @fn uint32_t calibrateCpuUtilizationMeter(void)
*
* @brief This function calibrates thee CPU utilization meter.
*
* @param payload - pointer to a payload. A NULL pointer means no payload is accessed.
* @param actualSize - size of payload. 0=don't check size related consistency
*
* @return 0=invalid payload header, 1=valid payload header
*
* @author Eli Schneider
*
* @date 10.02.2013
*/
uint32_t calibrateCpuUtilizationMeter(void)
{
	register uint32_t key;
	register uint32_t tmp;
	register uint32_t *p;
	xTaskHandle thisTaskHandle;
	unsigned portBASE_TYPE thisTaskPriority;


	thisTaskHandle=xTaskGetCurrentTaskHandle();
	thisTaskPriority=uxTaskPriorityGet(thisTaskHandle);
	vTaskPrioritySet( thisTaskHandle, configMAX_PRIORITIES-1 );
	vTaskDelay(1);				  // Synchronize with system counter
	key=__disableInterrupts();
	memset(&fullCpuUtilHystogram, 0, sizeof(struct sFullCpuUtilHystogram));
	setCpuUtilTimerCallback(cpuUtilTimerCallback,NULL);
	__restoreInterrupts(key);
	vTaskDelay(1);				  // Synchronize with system counter
	vTaskDelay((configTICK_RATE_HZ*CPU_UTIL_HYSTOGRAM_ENTRIES)/CPU_UTIL_FREQ+2);
	#ifdef CPU_UTIL_SUM_CAL
	tmp=0;
	p= fullCpuUtilHystogram.val.u;
	key=__disableInterrupts();
	while (p != &fullCpuUtilHystogram.val.u[CPU_UTIL_HYSTOGRAM_ENTRIES])
		tmp+= *p++;
	__restoreInterrupts(key);
	#else
	tmp=0;
	p= fullCpuUtilHystogram.val.u;
	key=__disableInterrupts();
	while (p != &fullCpuUtilHystogram.val.u[CPU_UTIL_HYSTOGRAM_ENTRIES])
	{
		if (tmp<*p)
			tmp= *p;
		p++;
	}
	tmp*=CPU_UTIL_HYSTOGRAM_ENTRIES;
	__restoreInterrupts(key);
	#endif
	
	idleIdleCount=tmp;
	if (tmp)
		reciprocalIdleIdleCount=(1.0/tmp);
	else
		reciprocalIdleIdleCount=0.0;

	// restore task priority
	vTaskPrioritySet( thisTaskHandle, thisTaskPriority);

	return tmp;
}

int32_t cpuUtilReadout(int percentage)
{
	register uint32_t key;
	register uint32_t sum;
	register uint32_t *p;
	//register uint32_t utilCount;
	float fCpuUtil;

	sum=0;
	p= fullCpuUtilHystogram.val.u;
	key=__disableInterrupts();
	while (p != &fullCpuUtilHystogram.val.u[CPU_UTIL_HYSTOGRAM_ENTRIES])
		sum+= *p++;
	__restoreInterrupts(key);
	if (sum<=idleIdleCount)
	{
		fCpuUtil=(idleIdleCount-sum);
		fCpuUtil*=reciprocalIdleIdleCount;
	}
	else
		fCpuUtil=0;
	
	if (percentage)
		return (int32_t)(fCpuUtil*100.0);
	else
		return (int32_t)(fCpuUtil*(1.0/32768.0));
}

int cpuUtilHystogram (struct sCpuUtilHystogram *d, int percentage)
{
	register uint32_t key;
	struct sPartialCpuUtilHystogram hystogramSample;
	uint16_t idx;
	float fCpuUtil;
	uint32_t tmp;

	if (d==NULL)
		return 0;
	key=__disableInterrupts();
	memcpy(&hystogramSample, &fullCpuUtilHystogram, sizeof(struct sPartialCpuUtilHystogram));
	__restoreInterrupts(key);

	//flatten hystogram
	if (hystogramSample.count==0)
	{
	}
	else if (hystogramSample.count<CPU_UTIL_HYSTOGRAM_ENTRIES)
	{
		if (hystogramSample.count<hystogramSample.first)
			memmove(&hystogramSample.val.u[0],&hystogramSample.val.u[hystogramSample.first-hystogramSample.count],sizeof(uint32_t)*(hystogramSample.first-hystogramSample.count));
		else 
		{
			while (hystogramSample.first<hystogramSample.count)
			{
				tmp=hystogramSample.val.u[CPU_UTIL_HYSTOGRAM_ENTRIES-1];
				memmove(&hystogramSample.val.u[1], &hystogramSample.val.u[0], sizeof(uint32_t)*hystogramSample.first);
				hystogramSample.val.u[0]=tmp;
				hystogramSample.first++;
			}
		}
	}
	else
	{
		while (hystogramSample.first)
		{
			tmp=hystogramSample.val.u[0];
			memmove(&hystogramSample.val.u[0], &hystogramSample.val.u[1], sizeof(uint32_t)*(CPU_UTIL_HYSTOGRAM_ENTRIES-1));
			hystogramSample.val.u[CPU_UTIL_HYSTOGRAM_ENTRIES-1]=tmp;
			hystogramSample.first--;
		}
	}

	if (percentage)
	{

		for (idx=0;idx<hystogramSample.count;idx++)
		{
			tmp=hystogramSample.val.u[idx]*CPU_UTIL_HYSTOGRAM_ENTRIES;
			if (tmp<=idleIdleCount)
			{
				fCpuUtil=(idleIdleCount-tmp);
				fCpuUtil*=reciprocalIdleIdleCount;
			}
			else
				fCpuUtil=0;
			d->val[idx]=(int16_t)(fCpuUtil*100.0);
		}
	}
	else
	{
		for (idx=0;idx<hystogramSample.count;idx++)
		{
			tmp=hystogramSample.val.u[idx]*CPU_UTIL_HYSTOGRAM_ENTRIES;
			if (tmp<=idleIdleCount)
			{
				fCpuUtil=(idleIdleCount-tmp);
				fCpuUtil*=reciprocalIdleIdleCount;
			}
			else
				fCpuUtil=0;
			d->val[idx]=(int16_t)(fCpuUtil*32768.0);
		}
	}
	
	return 1;
}

void addToCpuUtilHystogram(uint32_t dIdleCount, portTickType nTick)
{
	register uint32_t key;

	key=__disableInterrupts();
	fullCpuUtilHystogram.val.u[fullCpuUtilHystogram.first]=dIdleCount;
	fullCpuUtilHystogram.tick[fullCpuUtilHystogram.first]=nTick;
	if (fullCpuUtilHystogram.count<CPU_UTIL_HYSTOGRAM_ENTRIES)
		fullCpuUtilHystogram.count++;
	if ((fullCpuUtilHystogram.first+1)<CPU_UTIL_HYSTOGRAM_ENTRIES)
		fullCpuUtilHystogram.first++;
	else
		fullCpuUtilHystogram.first=0;
	__restoreInterrupts(key);
}


/**
* @fn void  initCpuUtilTimer(void)
*
* This function initializes the CPU utilization measurement timer.
*
* @author Eli Schneider
*
* @date 04.03.2013
*/
void  initCpuUtilTimer(void)
{
	TIM_TimeBaseInitTypeDef tbInit;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_ClocksTypeDef clkStatus;
	
	installInterruptHandler(TIM4_IRQn,cpuUtilTimerIsr,NULL);
	
	/* Configure NVIC */
	/* Enable the TIM4 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	

	/* Enable clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
 	/* Configure timer TIM4 for 16 Hz operation */
	TIM_TimeBaseStructInit(&tbInit);

	RCC_GetClocksFreq(&clkStatus);
	tbInit.TIM_Prescaler=(CPU_UTIL_PRESCALER-1);
	tbInit.TIM_Period=clkStatus.PCLK1_Frequency/(CPU_UTIL_FREQ*(CPU_UTIL_PRESCALER));
	
	TIM_TimeBaseInit(TIM4,&tbInit);
	TIM_SetCounter(TIM4, 0);

	/* TIM Interrupts enable */
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	
	/* TIM enable counter */
	TIM_Cmd(TIM4, ENABLE);

}



void setCpuUtilTimerCallback(FN_CPU_UTIL_TIMER_CALLBACK cb, void *cbArg)
{
	uint32_t key;
	
	key=__disableInterrupts();
	cpuUtilTimerCB=cb;
	cpuUtilTimerCbArg=cbArg;
	__restoreInterrupts(key);
}

int cpuUtilTimerCallback(void *arg)
{
	register uint32_t	idleCountSample;
	register uint32_t	deltaIdleCount;

	idleCountSample=idleCount;
	deltaIdleCount=idleCountSample-lastIdleCount;
	lastIdleCount=idleCountSample;
	addToCpuUtilHystogram(deltaIdleCount, xTaskGetTickCountFromISR());
	return 0;
}

/**
  * @brief  This function handles Cpu utilization measurement timer global interrupt request.
  */
int cpuUtilTimerIsr(void *para)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		if (cpuUtilTimerCB)
			return (*cpuUtilTimerCB)(cpuUtilTimerCbArg);
	}
	return 0;
}


