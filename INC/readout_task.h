
#ifndef __READOUT_TASK_H
#define __READOUT_TASK_H

struct sReadoutServerParam 
{
	int readoutId;								/**<Readout identifier 	*/			
	int devId;								/**< UART device identifier		*/								
	Uart_BaudRate		baud;				/**< Baud of Operation			*/
	Uart_NumStopBits	stopBits;			/**< Stopbits of Operation		*/
	Uart_CharLen		charLen;			/**< Character Length			*/
	Uart_Parity			parity;				/**< Parity of Operation		*/
};


struct sToReadout_packetizerStat
{
	uint32_t readout;
	uint32_t other;
};

struct sToReadout_packetizer
{
	struct sToReadout_packetizerStat stat;
};

struct sReadoutInterface
{
	void *outDev;					/**< Pointer to interface device driver output channel */
	void *inDev;					/**< Pointer to interface device driver input channel */
	int rxPacketTimeout;		/**< Packet reception timeout */
	unsigned int state;			/**< media interface state */
	//struct sFromCtl_packetizer rxPack;	/**< CTL Rx packetizer structure */
	struct sToReadout_packetizer txPack;	/**< Readout Tx packetizer structure */
};

#define READOUT_STATE_IDLE					0 /**< READOUT interface: Idle */
#define READOUT_STATE_TX					1 /**< READOUT interface: Transmitting, driver enabled */
#define READOUT_STATE_RX_WAIT_WINDOW		2 /**< READOUT interface: Response reception window */
#define READOUT_STATE_RX					3 /**< READOUT interface: Receiving */
#define READOUT_STATE_TX_DEFER				4 /**< READOUT interface: Transmission defer window */

#define MT_PACKET_FROM_READOUT	0x30

#define IS_MT_PACKET_FROM_READOUT(x) ((x)==(MT_PACKET_FROM_READOUT))


#ifdef __cplusplus
	extern "C" {
#endif



void readoutRxServerTask(void *para);
void readoutTxServerTask(void *para);
int32_t GetReadoutData(void);

	
#ifdef __cplusplus
}
#endif



#endif
