/**
* @file i2c.h
* @brief i2c driver definitions
*
* @author David Anidjar
*
* @version 0.0.1
* @date 23.06.2014
*/

#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>
#include <iodev.h>

#ifdef __cplusplus
	extern "C" {
#endif


/*  -------- enums -------- */



/**
 *  \brief Enumeration of the different modes of operation
 */
//typedef enum i2c_OpMode
//{
//	i2c_OpMode_POLLED = 0,      /**< Polled Mode                              */
//	i2c_OpMode_INTERRUPT,       /**< Interrupt Mode                           */
//	i2c_OpMode_INTERRUPTDMA     /**< DMA Mode                                 */
//} i2c_OpMode;
typedef uint8_t i2c_OpMode;

#define i2c_OpMode_POLLED ((i2c_OpMode)(0)) /**< Polled Mode			*/
#define i2c_OpMode_INTERRUPT ((i2c_OpMode)(1)) /**< Interrupt Mode	*/
#define i2c_OpMode_INTERRUPTDMA ((i2c_OpMode)(2)) /**< DMA Mode		*/
/**
 *  \brief Enumeration to set i2c master to different opmode
 */
//typedef enum i2c_PinOpMode
//{
//    Spi_PinOpMode_3PIN,         /**< Spi master 3 pin mode                    */
//    Spi_PinOpMode_SPISCS_4PIN,  /**< Spi master 4 pin mode uses SpiSCS        */
//    Spi_PinOpMode_SPIENA_4PIN,  /**< Spi master 4 pin mode uses SpiENA        */
//    Spi_PinOpMode_5PIN          /**< Spi master 5 pin mode                    */
//} i2c_PinOpMode;

//typedef uint8_t i2c_PinOpMode;
//
//#define i2c_PinOpMode_3PIN ((i2c_PinOpMode)(0)) 		/**< Spi master 3 pin mode				*/
//#define i2c_PinOpMode_SPISCS_4PIN ((i2c_PinOpMode)(1))	/**< Spi master 4 pin mode uses SpiSCS	*/
//#define i2c_PinOpMode_SPIENA_4PIN ((i2c_PinOpMode)(2))	/**< Spi master 4 pin mode uses SpiENA	*/
//#define i2c_PinOpMode_5PIN ((i2c_PinOpMode)(3))			/**< Spi master 5 pin mode				*/


/**
 *  \brief Enumeration to set Spi as master or slave
 *
 */
//typedef enum i2c_CommMode
//{
//	i2c_CommMode_MASTER,        /**< Spi master mode of operatoin             */
//	i2c_CommMode_SLAVE          /**< Spi slave mode of operatoin              */
//} i2c_CommMode;

typedef uint8_t i2c_CommMode;

#define i2c_CommMode_MASTER ((i2c_CommMode)(0)) 		/**< i2c master mode of operatoin             */
#define i2c_CommMode_SLAVE ((i2c_CommMode)(1))			/**< i2c slave mode of operatoin             */

#define I2C_WRITE					(0x00)
#define I2C_READ					(0x01)

/**
 *  \brief enumeration of IOCTLS Supported
 */
//typedef enum i2c_ioctlCmd
//{
//	i2c_IOCTL_CANCEL_PENDING_IO = 128,	/**< To cancel pending IO, cmdArg = NULL		*/
//	i2c_IOCTL_SET_CS_POLARITY,			/**< to set or reset CS Polarity, cmdArg = int *		*/
//	i2c_IOCTL_SET_POLLEDMODETIMEOUT,	/**< set polled mode timeout, cmdArg = uint32_t *	*/
//	i2c_IOCTL_SET_CLOCK_RATE			/**< set clock rate, cmdArg = uint32_t *			*/
//} i2c_ioctlCmd;

typedef enum i2c_ioctlCmd
{
    i2c_IOCTL_CANCEL_PENDING_IO = 128,	/**< To cancel pending IO, cmdArg = NULL		*/
    i2c_IOCTL_SET_CS_POLARITY,			/**< to set or reset CS Polarity, cmdArg = int *		*/
    i2c_IOCTL_SET_POLLEDMODETIMEOUT,	/**< set polled mode timeout, cmdArg = uint32_t *	*/
    i2c_IOCTL_SET_CLOCK_RATE			/**< set clock rate, cmdArg = uint32_t *			*/
} i2c_ioctlCmd;

/* -------- typedefs -------- */

typedef int (*i2c_isr)(void *);

/* -------- structs -------- */
/**
 *  \brief i2c statistics Collection Object
 *
 *  Statistics are collected on a per-controller basis for Spi. Hence, an
 *  object of this type is found embedded in the instance structure.
 */
typedef struct i2c_Stats
{
	uint32_t  rxBytes;                 /**< Number of bytes received            */
	uint32_t  txBytes;                 /**< Number of bytes transmitted         */
	uint32_t  pendingPacket;           /**< Number of pending packets           */
	uint32_t  rxOverrunError;          /**< Number of overrun errors            */
	uint32_t  timeoutError;            /**< Number of timeouts                  */
	uint32_t  bitError;                /**< Number of biterrors                 */
	//uint32_t  parityError;             /**< Number of parity Errors             */
	uint32_t  desyncError;             /**< Number of desync Errors             */
} i2c_Stats;

/**
 *  \brief i2c Channel Params Object
 */
typedef struct i2c_ChanParams
{
    void *hDma;                      /**< Dma handle                          */
} i2c_ChanParams;

/**
*	\brief Structure to initialize the data transfer format register.
*/
typedef struct i2c_ConfigDataFmt
{
	uint32_t	wDelay;				/**< Delay in between two transfers													*/
	uint8_t		charLength;			/**< defines the word length- b/w 2-16												*/
	uint8_t		lsbFirst;			/**< Sets the Shift of bits, TRUE - LSB First ; FALSE - MSB First							*/
	uint8_t		phaseIn;			/**< True - Phase In, data & clock in phase ; False - Phase out, data 1/2 cycle before clock	*/
	uint8_t		oddParity;			/**< True - Odd Parity ; False - Even Parity, valid only if ParityEnable  is true 				*/
	uint8_t		parityEnable;		/**< Parity check in the data format, True - Checks for parity, False - No Check				*/
	uint8_t		clkHigh;			/**< Sets the polarity of the clock,  True - POLARITY HIGH , False - POLARITY LOW			*/
	uint8_t		waitEnable;			/**< True - if in master mode,	wait for ENA signal from slave, False - do not wait			 */
} i2c_ConfigDataFmt;



/**
 *  \brief i2c Hardware Configuartion Structure
 */
typedef struct i2c_HWConfigData
{
	uint16_t		masterOrSlave:1;			/**< True - Slave Mode , False - Master Mode								*/
    uint16_t 		i2c_Mode:2;					/**< I2C mode	can be :I2C_Mode_I2C, I2C_Mode_SMBusDevice or I2C_Mode_SMBusHost*/
    uint16_t 		i2c_DutyCycle:1;			/**< I2C DutyCycle	can be :I2C_DutyCycle_16_9 or I2C_DutyCycle_2*/
    uint16_t 		i2c_Ack:1;					/**< I2C enable/disable i2c ack*/
    uint16_t		i2c_rsvrd:11;
    uint16_t 		i2c_OwnAddress;				/**< I2C slave mode need to have an address*/
    uint32_t 		i2c_ClockSpeed;				/**< I2C must be lower than 400Khz*/
    uint16_t 		i2c_AcknowledgedAddress; 	/**< I2C set 10 bits address or 7 bits address*/
    //uint32_t		waitDelay;					/**< True - enable format delay between 2 consecutive transfers ; False - No delay b/w 2 transfers 	*/
}i2c_HWConfigData;

/**
  * \brief i2c Transaction structure
  *
  * This structure holds the information needed to carry out a transaction on
  * an i2c bus to a slave device.
  */
typedef struct i2c_DataParam
{
    uint8_t			*outBuffer;		/**< Data buffer - During transceieve operation this buffer data is transmitted	*/
    uint8_t			*inBuffer;		/**< Data buffer - During transceive operation read data is stored in this buffer	*/
    uint32_t		outBufLen;		/**< Length of the buffer 												*/
    uint32_t		inBufLen;		/**< Length of the buffer 												*/
    uint16_t		flags;			/**< Flags  - To indicate the Read / Write modes of operation					*/
    uint16_t		deviceAddress;  /**< I2C Slave address to transmit/receive to/from */
    void			*param;			/**< Extra paramerter													*/
} i2c_DataParam;

/* -------- per-instance configs -------- */

/**
 *  \brief i2c Per instance configuration struct
 */
typedef struct i2c_Params
{
	i2c_OpMode			opMode;				/**< Driver operation mode						*/
	uint32_t			outputClkFreq;		/**< output clock frequency from Spi Module , transmits bitrate  in bits per second	*/
	//int					loopbackEnabled;	/**< To enable/disable loop back				*/
	uint32_t			polledModeTimeout;	/**< Polled mode timeout						*/
	i2c_HWConfigData	i2cHWCfgData;		/**< Hardware configuration						*/
	void				*hDma;				/**< Handle for dma							*/
	uint16_t			cpuEventNumber;		/**< cpu event number configured for i2c event		*/
	//uint32_t			inputFrequency;
} i2c_Params;

/**
 *  \brief Default i2c_Params struct
 *
 *  const i2c_Params i2c_PARAMS = {
 *
 *      i2c_OpMode_POLLED,              opMode
 *
 *      (Uint32)0x2dc6c0,               outputClkFreq
 *
 *      0,                               loopbackEnabled
 *
 *      SYS_FOREVER,                     timeout
 *
 *      {
 *          0,                           intrLevel
 *
 *          i2c_CommMode_MASTER,         masterOrSlave
 *
 *          1,                           clkInternal
 *
 *          0,                           enableHighZ
 *
 *          i2c_PinOpMode_SpiSCS_4PIN,   pinOpModes
 *
 *          {
 *              (Uint32)0x0,             c2TDelay
 *
 *              (Uint32)0x0,             t2CDelay
 *
 *              (Uint32)0x0,             t2EDelay
 *
 *              (Uint32)0x0,             c2EDelay
 *
 *          },                           delay
 *
 *          (Uint32)0x0,                 waitDelay
 *
 *          (Uint32)0xff,                csDefault
 *
 *          {
 *              {
 *              (Uint32)0x0,             wDelay
 *
 *              (Uint32)0x0,             charLength
 *
 *              0,                       lsbFirst
 *
 *              0,                       phaseIn
 *
 *              0,                       oddParity
 *
 *              0,                       parityEnable
 *
 *              0,                       clkHigh
 *
 *              0,                       waitEnable
 *
 *          },                           [0]

 *          {
 *              (Uint32)0x0,             wDelay
 *
 *              (Uint32)0x0,             charLength
 *
 *              0,                       lsbFirst
 *
 *              0,                       phaseIn
 *
 *              0,                       oddParity
 *
 *              0,                       parityEnable
 *
 *              0,                       clkHigh
 *
 *              0,                       waitEnable
 *
 *          },                           [1]

 *          {
 *              (Uint32)0x0,             wDelay
 *
 *              (Uint32)0x0,             charLength
 *
 *              0,                       lsbFirst
 *
 *              0,                       phaseIn
 *
 *              0,                       oddParity
 *
 *              0,                       parityEnable
 *
 *              0,                       clkHigh
 *
 *              0,                       waitEnable
 *
 *          },                           [2]

 *          {
 *              (Uint32)0x0,             wDelay
 *
 *              (Uint32)0x0,             charLength
 *
 *              0,                       lsbFirst
 *
 *              0,                       phaseIn
 *
 *              0,                       oddParity
 *
 *              0,                       parityEnable
 *
 *              0,                       clkHigh
 *
 *              0,                       waitEnable
 *
 *          },                           [3]

 *      },                               configDatafmt
 *	#ifdef I2C_EXTENDED_CHIP_SELECT
 *	NULL, 								extGpiochipSelect
 *	#endif
 *
 *  },                                   spiHWCfgData
 *
 *  ((Ptr)((void*)0x0)),                 edmaHandle
 *
 *  (Uint16)0x0,                         HWINumber
 *
 *};
 *
 */
extern const i2c_Params i2c_PARAMS;


/**
 *  \brief i2c IOM_Fxns table
 *
 *  Driver function table to be used by applications.
 */
extern const IODEV_Fxns i2c_IODEVFXNS;

/**
 *  \brief BB_i2c IOM_Fxns table
 *
 *  Driver function table to be used by applications.
 */
extern const IODEV_Fxns BB_i2c_IODEVFXNS;

/* -------- defines -------- */

/**
 *  \brief   Local Buffer data allocation for transcieve operation.
 */
#define I2C_BUFFER_DATA_SIZE (32u)
/**
 *  \brief   Max channels supported
 *
 *          Max i2c driver opens - which means number of s/w channels which
 *          can be opened for a single instance of Spi
 */
#define I2C_NUM_CHANS (1)
/**
 * \brief   Macro used to enable/Disable CSHOLD
 */
#define I2C_CSHOLD ((0x1u))
/**
 * \brief   Macro for CSHOLD multi transceive
 *
 *  CSHOLD will be selected [pulled low]after the trasceive operation and
 *  this is valid if Spi_CSHOLD flag is enabled. Also if Chip select value is
 *  changed in the next transceive operation then also this flag becomes
 *  invalid for the previous rtansceive operation i.e CS lines comes high.
 */
#define I2C_CSHOLD_FOR_MULTI_TRANSCEIVE ((0x2u))

/** Macro used to set gpio pin as active low                              */
#define I2C_LOW   (0x0u)

/** Macro used to set gpio pin as active high                             */
#define I2C_HIGH  (0x1u)

/**
 *  \brief    Initializes Spi instances which are statically allocated
 *
 *  This function needs to be be called at part of BIOS initialization by
 *  setting initFxn for that particular UDEV instance or by calling this
 *  function as part of user specific initFxn.
 */
void sti2cInit(void);

/**
 *  \brief    Initializes Bit Bang i2c instances which are statically allocated
 *
 *  This function needs to be be called at part of BIOS initialization by
 *  setting initFxn for that particular UDEV instance or by calling this
 *  function as part of user specific initFxn.
 */
void stBB_i2cInit(void);


#ifdef __cplusplus
}
#endif


#endif /* I2C_H_ */
