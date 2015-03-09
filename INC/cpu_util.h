/**
* @file cpu_util.h
* @brief CPU utilization measurement services
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 03.03.2013
*/
#ifndef _CPU_UTIL_H
#define _CPU_UTIL_H

#include <stdint.h>

#define CPU_UTIL_HYSTOGRAM_ENTRIES	(16)
#define CPU_UTIL_FREQ				(32)

struct sCpuUtilHystogram 
{
	int16_t		val[CPU_UTIL_HYSTOGRAM_ENTRIES];
};


#ifdef __cplusplus
extern "C" 
{
#endif

void  initCpuUtilTimer(void);
uint32_t calibrateCpuUtilizationMeter(void);
int32_t cpuUtilReadout(int percentage);
int cpuUtilHystogram (struct sCpuUtilHystogram *d, int percentage);
void addToCpuUtilHystogram(uint32_t dIdleCount, portTickType nTick);

#ifdef __cplusplus
}
#endif

#endif




