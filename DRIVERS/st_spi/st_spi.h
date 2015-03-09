/**
* @file st_spi.h
* @brief lspi driver local definitions
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 28.06.2012
*/

#ifndef _ST_SPI_H_
#define _ST_SPI_H_
 
#include <stddef.h>
#include <stdint.h>
#include <iodev.h>
#include <freertos.h>
#include <semphr.h>
#include <sem.h>
#include <que.h>
#include <spi.h>
#include "stm32f2xx.h"

#ifndef N_SPI
#define N_SPI	(3)
#endif

#ifdef __cplusplus
extern "C" {
#endif
/* -------- constants -------- */

#define Spi_ERROR_BASE				(-11)		/**< Base error code				*/
#define Spi_RECEIVE_OVERRUN_ERR		((-11) - 1)	/**< SPI receive overrun error code	*/
#define Spi_BIT_ERR					((-11) - 2)	/**< SPI Bit transfer error code		*/
#define Spi_DESYNC_ERR				((-11) - 3)	/**< SPI Slave desync error code	*/
#define Spi_PARITY_ERR				((-11) - 4)	/**< SPI parity error code			*/
#define Spi_TIMEOUT_ERR				((-11) - 5)	/**< SPI time out error code		*/
#define Spi_DATALENGTH_ERR			((-11) - 6)	/**< SPI Data length  error code		*/
#define Spi_CANCEL_IO_ERROR			((-11) - 7)	/**< SPI Cancel IO error code		*/

//#define Spi_HW_INTR_NUM				(0x0080u)	/**< SPI hardware interrupt number ,that will enable the interrupt function   */

#define Spi_MAX_CHIPSELECT			(3u)		/**< Max chip select numbers		*/

#define Spi_INTERRUPT_MASK			(0x000001FFu)	/**< SPI Interrupt register mask		*/

#define Spi_DATAFORMAT_VALUE		(4u)		/**< it shows spi supports maximum value of data format register.	*/

#define Spi_SPIDAT1_CSHOLD_8BIT_MASK	(0x10u)`	/**< SPI CSHOLD MASK 8 bit	*/

#define Spi_RX_DMA_CALLBACK_OCCURED	(0x2u)		/**< Flag to indicate that Reception DMA callback has occured	*/
#define Spi_TX_DMA_CALLBACK_OCCURED	(0x1u)		/**< Flag to indicate that Transmission DMA callback has occured	*/
    
#define Spi_DMA_8_BIT_CHAR_LEN		(8u)		/**< SPI 8 bit character length		*/
#define Spi_ACNT_FLAG				(2u)		/**< CNT flag to indicate in DMA Transfer                                    */
#define Spi_DEST_INDEX_FLAG			(1u)		/**< Destination flag to indicate in DMA Transfer                            */

#define Spi_SPIDAT1_CSNR_MASK		(0x00010000u)	/**< Chip select number MASK Macro for SPIDAT1			*/

#define Spi_OPT_TCINTEN_SHIFT		(0x00000014u)	/**< Interrupt enable bit in OPT register for dma			*/


/* -------- enums -------- */

/** 
 *  \brief Spi driver state
 * 
 *  SPi driver state enums used to track the driver state.
 */
typedef uint8_t Spi_DriverState;
	
#define Spi_DriverState_DELETED ((Spi_DriverState)(0))
#define Spi_DriverState_CREATED ((Spi_DriverState)(1))
#define Spi_DriverState_INITIALIZED ((Spi_DriverState)(2))
#define Spi_DriverState_OPENED ((Spi_DriverState)(3))
#define Spi_DriverState_CLOSED ((Spi_DriverState)(4))
#define Spi_DriverState_DEINITIALIZED ((Spi_DriverState)(5))
#define Spi_DriverState_POWERED_DOWN ((Spi_DriverState)(6))


/* -------- structs -------- */

/**  
 *  \brief channel structure
 *
 *  There is one ChanObj per direction.  This mini-driver must be
 *  opened for input and output separately.
 */
typedef struct Spi_ChanObj 
{
	int8_t					mode;			/**< INPUT or OUTPUT					*/
	Spi_DriverState			channelState;	/**< state of the SPI Either created or deleted                           */
	int8_t					loopbackEnabled;	/**< Enable/Disable loop back mode                                        */
	int8_t					charLength16Bits;	/**< Flag to indicate if char length greater than 8 bits                  */
	IODEV_TiomCallback		cbFxn;			/**< to notify client when I/O complete                                   */
	void					*cbArg;			/**<  argument for cbFxn()                                                */
	void					*instHandle;	/**< Spi Handle to access the spi params                                  */
	uint32_t				busFreq;		/**< SPI Bus Frequency                                                    */
	IODEV_Packet			*activeIOP;		/**< Current IO packet                                                    */
	Spi_DataParam           dataParam;		/**< Current IO packet                                                    */
	int						pendingState;	/**< Shows whether io is in pending state or not                          */
	int						cancelPendingIO;	/**< Shows whether IO has to cancel or not                                */
	int32_t					currError;		/**< current error flag                                                   */
	uint32_t				currFlags;		/**< Current Flags for read/write                                         */
	int						transcieveFlags;	/**< flag for transcieve operation                                        */
	uint8_t					*currBuffer;	/**< User buffer for read/write                                           */
	uint8_t					*transBuffer;	/**< Buffer used for transieve operation of spi                           */
	uint32_t				currBufferLen;	/**<*< User buffer length                                                 */
	QUE_Obj                 queuePendingList;	/**< pending Iop List head                                                */
	unsigned portBASE_TYPE	taskPriority;	/**<this will hold the priority of the task that created this channel     */
}Spi_ChanObj;

/**
 *  \brief HW info stfructure
 *
 *  Spi structure to hold the instance specific information.
 */

typedef struct Spi_HwInfo {
    SPI_TypeDef *baseAddress;
    uint32_t inputFrequency;
    uint16_t cpuEventNumber;
    uint16_t rxDmaEventNumber;
    uint16_t txDmaEventNumber;
    //uint32_t maxChipSelect;
}Spi_HwInfo;


/** 
 *  \brief instance structure
 *
 *  There is one instance structure per SPI instance. maximum number of
 *  structures is given by {@link #numInstances}
 */
typedef struct {
    uint8_t *inUse;
} Spi_Module_State;


/** 
 *  \brief instance structure
 *
 *  There is one instance structure per SPI instance. maximum number of
 *  structures is given by {@link #numInstances}
 */
typedef struct 
{
    uint32_t		instNum;				/**< Instance number of this instance                                     */
    Spi_DriverState	devState;				/**< state of the SPI Either created or deleted                           */
    Spi_OpMode		opMode;	  				/**< Mode of operation                                                    */
    Spi_HwInfo		deviceInfo;				/**< instance specific information                                        */
    Spi_ChanObj		chanObj[Spi_NUM_CHANS];	/**< channel objects for the SPI                                          */
    uint32_t		numOpens;				/**< Number of channels opened                                            */
    //int				enableCache;			/**< Submitted buffers are in cacheable memory                            */
	void			*hDmaRes;				/**< Instance specific information                                        */
	void			*hTxDmaHandler;			/**< Handle to Tx DMA interrupt handler object   */
	void			*hTxDmaHandlerArg;		/**< Handle to DMA interrupt handler argument object   */
	void			*hRxDmaHandler;			/**< Handle to Rx DMA interrupt handler object   */
	void			*hRxDmaHandlerArg;		/**< Handle to DMA interrupt handler argument object   */

    Spi_Stats		stats;					/**< SPI specific stats                                                   */
    Spi_HWConfigData    spiHWconfig;		/**< SPI Hardware configurations                                          */
    int				csHighPolarity;			/**< Chip Select Polarity. Default is set to Active Low = FALSE           */
    void			*hDma;					/**< Handle used for dma                                                 */
    int8_t			dmaChanAllocated;		/**< Flag to inidicate DMA channels allocation status                    */
    int8_t			isSlaveChannelOpened;	/**<This boolean track for having only one slave channel                  */
    uint16_t		cpuEventNumber;			/**< Hardware CPU event number                                            */
	
    volatile uint32_t	edmaCbCheck;		/**< Use to check occurance of DMA callback                              */
    uint32_t		polledModeTimeout;		/**< Timeout for the io operation                                      */
    Spi_ChanObj		*currentActiveChannel;	/**< This specifies the current active channel                            */
} Spi_Object;

extern void spiLocalControlDataConfig(Spi_ChanObj *chanHandle, uint32_t chipSelect, Spi_DataFormat dataFormat, uint32_t flags);
extern void spiLocalGetNextChannel(Spi_Object *instHandle, Spi_ChanObj**pChanHandle);
extern int spiLocalDmaTransfer(void *handle, uint32_t chipSelect, Spi_DataFormat dataFormat, uint32_t flags);
extern int spiLocalDmaChannel_Request(Spi_Object *instHandle);

#ifdef __cplusplus
}
#endif

#endif  

