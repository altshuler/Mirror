/**
* @file ip_param_config.c
* @brief Application access 
*
* @author David Anidjar
*
* @version 0.0.1
* @date  06.03.2014
*/

//#include "production_data.h"
//#include "volume.h"
//#include "param_vol.h"
//#include "boot_params_data.h"
#include <stddef.h>
#include <stdint.h>
#include "bootparam.h"
#include "ip_param_config.h"
#include "boot_param_id.h"
#include "endianutils.h"
//struct Ip_Settings IpSettings;


const uint16_t vInitIpIntSetVecIdx[] = {ID_PARAM_IPADDR_SRC,ID_PARAM_STATIC_IPADDR,ID_PARAM_STATIC_NETMASK,ID_PARAM_STATIC_DEFAULT_GATEWAY};
const uint32_t vInitIpIntSetVecDef[] = {1,0,0,0};

int getIpSettings(struct sIpSettings *d)
{
	union uIpIntSettings ipIntSetVec;
	int status;
	int skip = 0;

	status=getParamVecInUnsignedInt(ipIntSetVec.v,sizeof(ipIntSetVec.v)/sizeof(ipIntSetVec.v[0]), (uint16_t *)vInitIpIntSetVecIdx, (uint32_t *)vInitIpIntSetVecDef,&skip);

	d->DHCPEnable=ipIntSetVec.s.DHCPEnable;
	d->IPAddress=longBE2LE(ipIntSetVec.s.IPAddress);
	d->NetMask=longBE2LE(ipIntSetVec.s.NetMask);
	d->DefaultGateway=longBE2LE(ipIntSetVec.s.DefaultGateway);
	d->hostname=NULL;;

	return status;
}









