/**
* @file bootinfo.h
* @brief Bootloader-application shared information
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 30.10.2013
*/
#ifndef _BOOTINFO_H
#define _BOOTINFO_H
	
#include <stdint.h>

struct sBootInfoData
{
	uint8_t version[16];
};

struct sBootInfoArea
{
	struct sBootInfoData bootData;
	uint32_t bootRqAddr; 		/* pointer to boot loader request function */
	uint32_t bootRqAddrComp;	/* = ~bootRqAddr */
};

extern struct sBootInfoArea bootInfo;


#endif


