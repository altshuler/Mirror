/**
* @file interface.h
* @brief interface identifiers
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 16.04.2012
*/
#ifndef _INTERFACE_H
#define _INTERFACE_H

#include <stdint.h>
#include <msg_type.h>

#define INT_CTL1	1
#define INT_CTL2	2
#define INT_READOUT	3

#define MSGHDR_CTLRX_PACKET(x) MAKE_MSG_HDRTYPE(0,MSG_SRC_CTL1RX+((x)*2),MSG_TYPE_PACKET)

#define MSGHDR_CTLRX1_PACKET MAKE_MSG_HDRTYPE(0,MSG_SRC_CTL1RX,MSG_TYPE_PACKET)
#define MSGHDR_CTLRX2_PACKET MAKE_MSG_HDRTYPE(0,MSG_SRC_CTL2RX,MSG_TYPE_PACKET)
#define MSGHDR_READOUT_PACKET MAKE_MSG_HDRTYPE(0,MSG_SRC_READOUTTX,MSG_TYPE_PACKET)

//#define MSGHDR_CMD_PACKET MAKE_MSG_HDRTYPE(0,MSG_SRC_CMD,MSG_TYPE_PACKET)

#ifdef __cplusplus
extern "C" 
{
#endif

int sendPacketToReadout(void *packet, uint16_t hdr, uint32_t timeout);
int sendPacketToCtl(uint16_t ctlId, void *packet, uint16_t hdr, uint32_t timeout);
int sendPacketToIntCmd(void *packet, uint16_t hdr, uint32_t timeout);


#ifdef __cplusplus
}
#endif



#endif


