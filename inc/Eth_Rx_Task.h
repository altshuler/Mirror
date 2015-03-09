
#ifndef __ETH_RX_TASK_H
#define __ETH_RX_TASK_H






void Eth_Rx_Task(void * pvParameters);
void Eth_Tx_Task(void * pvParameters);
int sendPacketToHost(void *packet, uint16_t hdr, uint32_t timeout);
void checkEthLink(void *pvParameters);


#endif
