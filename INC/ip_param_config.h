/**
* @file ip_param_config.h
* @brief Application access 
*
* @author David Anidjar
*
* @version 0.0.1
* @date 06.03.2014
*/
#ifndef _IP_PARAM_CONFIG_H
#define _IP_PARAM_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct sIpIntSettings
{
	uint32_t DHCPEnable;
	uint32_t IPAddress;
	uint32_t NetMask;
	uint32_t DefaultGateway;
};

union uIpIntSettings
{
	uint32_t v[sizeof(struct sIpIntSettings)/sizeof(uint32_t)];
	struct sIpIntSettings s;
};

struct sIpSettings
{
	uint32_t DHCPEnable;
	uint32_t IPAddress;
	uint32_t NetMask;
	uint32_t DefaultGateway;
	char *hostname;
};


struct sAppIpSettings
{
	struct sIpSettings Ip;
	uint32_t ServerIPAddress;
	uint32_t Port;
};


int getIpSettings(struct sIpSettings *d);

#ifdef __cplusplus
}
#endif


#endif
