/**
* @file appinfo.h
* @brief Application download information
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 30.10.2013
*/
#ifndef _APPINFO_H
#define _APPINFO_H
	
#include <stdint.h>

struct sAppUpgradeData
{
	uint16_t length;	/* in lines */
	uint16_t nu;		/* always 0 */
	uint8_t name[16];
	uint8_t version[16];
};

struct sAppInfoArea
{
	struct sAppUpgradeData upgradeData;
};

extern struct sAppInfoArea appInfo;


#endif

