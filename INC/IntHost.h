/**
* @file inthost.h
* @brief meters internal network services.
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 09.01.2011
*
*/
#ifndef __INTHOST_H
#define __INTHOST_H

#include "packetbuf.h"
//#include "cpu_types.h"
#include "hostcomm.h"
#include "membuf.h"


int initHostInterface(struct sHostInterface *iface, void *dev);
void uart0_tx_pdma_handler(void);
int sendPacketToHost(void *packet, uint16_t hdr, uint32_t timeout);
int sendHostPacket(void *packetBuf, struct sHostInterface *iface, uint32_t ch);
unsigned int getHostInterfaceState(struct sHostInterface *iface);
unsigned int setHostInterfaceState(struct sHostInterface *iface, unsigned int val);

#endif


