/**
* @file st_i2c.h
* @brief system portable functions
*
* @author Eli Schneider
* @author David Anidjar
*
* @version 0.0.1
* @date 12.08.2014
*/

#ifndef _ST_I2C_H
#define _ST_I2C_H

#include <stdint.h>

//#define N_I2C	(2)

#ifdef __cplusplus
extern "C" {
#endif

/* -------- constants -------- */

#define I2C_ERROR_BASE				(-11)		/**< Base error code				*/
#define I2C_RECEIVE_OVERRUN_ERR		((-11) - 1)	/**< I2C receive overrun error code	*/
#define I2C_BIT_ERR					((-11) - 2)	/**< I2C Bit transfer error code		*/
#define I2C_DESYNC_ERR				((-11) - 3)	/**< I2C Slave desync error code	*/
#define I2C_PARITY_ERR				((-11) - 4)	/**< I2C parity error code			*/
#define I2C_TIMEOUT_ERR				((-11) - 5)	/**< I2C time out error code		*/
#define I2C_DATALENGTH_ERR			((-11) - 6)	/**< I2C Data length  error code		*/
#define I2C_CANCEL_IO_ERROR			((-11) - 7)	/**< I2C Cancel IO error code		*/

//#define Spi_HW_INTR_NUM				(0x0080u)	/**< SPI hardware interrupt number ,that will enable the interrupt function   */

#define I2C_MAX_CHIPSELECT			(3u)		/**< Max chip select numbers		*/

#define I2C_INTERRUPT_MASK			(0x000001FFu)	/**< I2C Interrupt register mask		*/

#define I2C_DATAFORMAT_VALUE		(4u)		/**< it shows spi supports maximum value of data format register.	*/

#define I2C_SPIDAT1_CSHOLD_8BIT_MASK	(0x10u)`	/**< SPI CSHOLD MASK 8 bit	*/

#define I2C_RX_DMA_CALLBACK_OCCURED	(0x2u)		/**< Flag to indicate that Reception DMA callback has occured	*/
#define I2C_TX_DMA_CALLBACK_OCCURED	(0x1u)		/**< Flag to indicate that Transmission DMA callback has occured	*/

#define I2C_DMA_8_BIT_CHAR_LEN		(8u)		/**< SPI 8 bit character length		*/
#define I2C_ACNT_FLAG				(2u)		/**< CNT flag to indicate in DMA Transfer                                    */
#define I2C_DEST_INDEX_FLAG			(1u)		/**< Destination flag to indicate in DMA Transfer                            */

#define I2C_SPIDAT1_CSNR_MASK		(0x00010000u)	/**< Chip select number MASK Macro for SPIDAT1			*/

#define I2C_OPT_TCINTEN_SHIFT		(0x00000014u)	/**< Interrupt enable bit in OPT register for dma			*/


/* -------- enums -------- */
/** 
 *  \brief I2C driver state
 * 
 *  I2C driver state enums used to track the driver state.
 */


typedef uint8_t I2C_DriverState;

#define I2C_DriverState_DELETED ((I2C_DriverState)(0))
#define I2C_DriverState_CREATED ((I2C_DriverState)(1))
#define I2C_DriverState_INITIALIZED ((I2C_DriverState)(2))
#define I2C_DriverState_OPENED ((I2C_DriverState)(3))
#define I2C_DriverState_CLOSED ((I2C_DriverState)(4))
#define I2C_DriverState_DEINITIALIZED ((I2C_DriverState)(5))
#define I2C_DriverState_POWERED_DOWN ((I2C_DriverState)(6))


/* -------- structs -------- */

/**  
 *  \brief channel structure
 *
 *  There is one ChanObj per direction.  This mini-driver must be
 *  opened for input and output separately.
 */
typedef struct I2C_ChanObj 
{
	int						mode;			/**< INPUT or OUTPUT					*/
	I2C_DriverState			channelState;	/**< state of the I2C Either created or deleted                           */
	IODEV_TiomCallback		cbFxn;			/**< to notify client when I/O complete                                   */
	void					*cbArg;			/**<  argument for cbFxn()                                                */
	void					*instHandle;	/**< I2C Handle to access the spi params                                  */
	uint32_t				busFreq;		/**< I2C Bus Frequency                                                    */
	IODEV_Packet			*activeIOP;		/**< Current IO packet                                                    */
	i2c_DataParam           dataParam;		/**< Current IO packet                                                    */
	int						pendingState;	/**< Shows whether io is in pending state or not                          */
	int						cancelPendingIO;	/**< Shows whether IO has to cancel or not                                */
	int32_t					currError;		/**< current error flag                                                   */
	uint32_t				currFlags;		/**< Current Flags for read/write                                         */
	int						transcieveFlags;	/**< flag for transcieve operation                                        */
	uint8_t					*currBuffer;	/**< User buffer for read/write                                           */
	uint8_t					*transBuffer;	/**< Buffer used for transieve operation of spi                           */
	uint32_t				currBufferLen;	/**<*< User buffer length                                                 */
	QUE_Obj                 queuePendingList;	/**< pending Iop List head                                                */
}I2C_ChanObj;


/**
 *  \brief HW info stfructure
 *
 *  Spi structure to hold the instance specific information.
 */

typedef struct I2C_HwInfo {
    I2C_TypeDef *baseAddress;
    uint32_t inputFrequency;
    uint16_t cpuEventNumber;
    uint16_t rxDmaEventNumber;
    uint16_t txDmaEventNumber;
}I2C_HwInfo;

/** 
 *  \brief instance structure
 *
 *  There is one instance structure per I2C instance. maximum number of
 *  structures is given by {@link #numInstances}
 */
typedef struct {
    uint8_t *inUse;
} I2C_Module_State;

/** 
 *  \brief instance structure
 *
 *  There is one instance structure per SPI instance. maximum number of
 *  structures is given by {@link #numInstances}
 */
typedef struct 
{
    uint32_t			instNum;				/**< Instance number of this instance                                     */
    i2c_OpMode			opMode;	  				/**< Mode of operation                                                    */
    uint16_t			cpuEventNumber;			/**< Hardware CPU event number                                            */
    I2C_HwInfo			deviceInfo;				/**< instance specific information                                        */
    I2C_ChanObj			chanObj[I2C_NUM_CHANS];	/**< channel objects for the I2C                                          */
    uint32_t			numOpens;				/**< Number of channels opened                                            */
    void				*hDmaRes;				/**< Instance specific information                                        */
	void				*hTxDmaHandler;			/**< Handle to Tx DMA interrupt handler object   */
	void				*hTxDmaHandlerArg;		/**< Handle to DMA interrupt handler argument object   */
	void				*hRxDmaHandler;			/**< Handle to Rx DMA interrupt handler object   */
	void				*hRxDmaHandlerArg;		/**< Handle to DMA interrupt handler argument object   */

    i2c_Stats			stats;					/**< I2C specific stats                                                   */
    i2c_HWConfigData    i2cHWconfig;			/**< I2C Hardware configurations                                          */
    void				*hDma;					/**< Handle used for dma                                                 */
    uint32_t			polledModeTimeout;		/**< Timeout for the io operation                                      */
    I2C_ChanObj			*currentActiveChannel;	/**< This specifies the current active channel                            */
    I2C_DriverState		devState;				/**< state of the I2C Either created or deleted                           */
    volatile uint8_t	edmaCbCheck;			/**< Use to check occurance of DMA callback                              */
    uint8_t				dmaChanAllocated;		/**< Flag to inidicate DMA channels allocation status                    */
    uint8_t				isSlaveChannelOpened;	/**<This boolean track for having only one slave channel                  */
} I2C_Object;

/**
 *  \brief HW info stfructure
 *
 *  i2c structure to hold the instance specific information.
 */

typedef struct i2c_HwInfo {
    I2C_TypeDef *baseAddress;
    uint32_t cpuEventNumber;
    uint32_t rxDmaEventNumber;
    uint32_t txDmaEventNumber;
    uint32_t inputFrequency;
    //uint32_t maxChipSelect;
}i2c_HwInfo;


/**
 *  \brief instance structure
 *
 *  There is one instance structure per SPI instance. maximum number of
 *  structures is given by {@link #numInstances}
 */
typedef struct {
	uint8_t *inUse;
} i2c_Module_State;


/**
 *  \brief instance structure
 *
 *  There is one instance structure per SPI instance. maximum number of
 *  structures is given by {@link #numInstances}
 */
typedef struct
{
    uint32_t		instNum;				/**< Instance number of this instance                                     */
    I2C_DriverState	devState;				/**< state of the SPI Either created or deleted                           */
    i2c_OpMode		opMode;	  				/**< Mode of operation                                                    */
    i2c_HwInfo		deviceInfo;				/**< instance specific information                                        */
    I2C_ChanObj		chanObj[I2C_NUM_CHANS];	/**< channel objects for the SPI                                          */
    uint32_t		numOpens;				/**< Number of channels opened                                            */
    uint16_t		cpuEventNumber;			/**< Hardware CPU event number                                            */
    void			*hDmaRes;				/**< Instance specific information                                        */
	void			*hTxDmaHandler;			/**< Handle to Tx DMA interrupt handler object   */
	void			*hTxDmaHandlerArg;		/**< Handle to DMA interrupt handler argument object   */
	void			*hRxDmaHandler;			/**< Handle to Rx DMA interrupt handler object   */
	void			*hRxDmaHandlerArg;		/**< Handle to DMA interrupt handler argument object   */

	i2c_Stats		stats;					/**< SPI specific stats                                                   */
	i2c_HWConfigData    i2cHWconfig;		/**< SPI Hardware configurations                                          */
    int				csHighPolarity;			/**< Chip Select Polarity. Default is set to Active Low = FALSE           */
    void			*hDma;					/**< Handle used for dma                                                 */
    int				dmaChanAllocated;		/**< Flag to inidicate DMA channels allocation status                    */

    volatile uint32_t	edmaCbCheck;		/**< Use to check occurance of DMA callback                              */
    uint32_t		polledModeTimeout;		/**< Timeout for the io operation                                      */
    I2C_ChanObj		*currentActiveChannel;	/**< This specifies the current active channel                            */
    int				isSlaveChannelOpened;	/**<This boolean track for having only one slave channel                  */
} i2c_Object;

extern void i2cLocalControlDataConfig(I2C_ChanObj *chanHandle);
extern void i2cLocalGetNextChannel(i2c_Object *instHandle, I2C_ChanObj**pChanHandle);
extern int i2cLocalDmaTransfer(void *handle);
extern int i2cLocalDmaChannel_Request(i2c_Object *instHandle);


#ifdef __cplusplus
}
#endif


#endif


