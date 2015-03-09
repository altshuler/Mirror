/**
* @file spi.h
* @brief spi driver definitions
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 28.06.2012
*/
#ifndef _SPI_H
#define _SPI_H
	
#include <stdint.h>
#include <iodev.h>

#ifdef __cplusplus
	extern "C" {
#endif

/*  -------- enums -------- */

/**
 *  \brief Enumeration to select the different data transfer format.
 */
typedef uint8_t Spi_DataFormat;

#define Spi_DataFormat_0 ((Spi_DataFormat)(0)) /**< select format 0 */
#define Spi_DataFormat_1 ((Spi_DataFormat)(1)) /**< select format 1 */
#define Spi_DataFormat_2 ((Spi_DataFormat)(2)) /**< select format 2 */
#define Spi_DataFormat_3 ((Spi_DataFormat)(3)) /**< select format 3 */

/**  
 *  \brief Enumeration of the different modes of operation
 */ 
typedef uint8_t Spi_OpMode;

#define Spi_OpMode_POLLED ((Spi_OpMode)(0)) /**< Polled Mode			*/
#define Spi_OpMode_INTERRUPT ((Spi_OpMode)(1)) /**< Interrupt Mode	*/
#define Spi_OpMode_INTERRUPTDMA ((Spi_OpMode)(2)) /**< DMA Mode		*/


/**
 *  \brief Enumeration to set Spi master to different opmode
 */
typedef uint8_t Spi_PinOpMode;

#define Spi_PinOpMode_3PIN ((Spi_PinOpMode)(0)) 		/**< Spi master 3 pin mode				*/
#define Spi_PinOpMode_SPISCS_4PIN ((Spi_PinOpMode)(1))	/**< Spi master 4 pin mode uses SpiSCS	*/
#define Spi_PinOpMode_SPIENA_4PIN ((Spi_PinOpMode)(2))	/**< Spi master 4 pin mode uses SpiENA	*/
#define Spi_PinOpMode_5PIN ((Spi_PinOpMode)(3))			/**< Spi master 5 pin mode				*/


/**
 *  \brief Enumeration to set Spi as master or slave
 *
 */
typedef uint8_t Spi_CommMode;

#define Spi_CommMode_MASTER ((Spi_CommMode)(0)) 		/**< Spi master mode of operatoin             */
#define Spi_CommMode_SLAVE ((Spi_CommMode)(1))			/**< Spi slave mode of operatoin             */


/**
 *  \brief enumeration of IOCTLS Supported
 */
typedef enum Spi_ioctlCmd 
{
    Spi_IOCTL_CANCEL_PENDING_IO = 128,	/**< To cancel pending IO, cmdArg = NULL		*/
    Spi_IOCTL_SET_CS_POLARITY,			/**< to set or reset CS Polarity, cmdArg = int *		*/
    Spi_IOCTL_SET_POLLEDMODETIMEOUT,	/**< set polled mode timeout, cmdArg = uint32_t *	*/
    Spi_IOCTL_SET_CLOCK_RATE			/**< set clock rate, cmdArg = uint32_t *			*/
} Spi_ioctlCmd;

/* -------- typedefs -------- */

typedef int (*Spi_isr)(void *);

/* -------- structs -------- */
/**
 *  \brief Spi statistics Collection Object
 *
 *  Statistics are collected on a per-controller basis for Spi. Hence, an
 *  object of this type is found embedded in the instance structure.
 */
typedef struct Spi_Stats
{
	uint32_t  rxBytes;                 /**< Number of bytes received            */
	uint32_t  txBytes;                 /**< Number of bytes transmitted         */
	uint32_t  pendingPacket;           /**< Number of pending packets           */
	uint32_t  rxOverrunError;          /**< Number of overrun errors            */
	uint32_t  timeoutError;            /**< Number of timeouts                  */
	uint32_t  bitError;                /**< Number of biterrors                 */
	uint32_t  parityError;             /**< Number of parity Errors             */
	uint32_t  desyncError;             /**< Number of desync Errors             */
} Spi_Stats;

/**
 *  \brief Spi Channel Params Object
 */    
typedef struct Spi_ChanParams 
{
    void *hDma;                      /**< Dma handle                          */
} Spi_ChanParams;
    
/**
*	\brief Structure to initialize the data transfer format register.
*/
typedef struct Spi_ConfigDataFmt 
{
	#ifdef KUKU
	uint32_t	wDelay;				/**< Delay in between two transfers													*/
	uint8_t		charLength;			/**< defines the word length- b/w 2-16												*/			   
	uint8_t		lsbFirst;			/**< Sets the Shift of bits, TRUE - LSB First ; FALSE - MSB First							*/
	uint8_t		phaseIn;			/**< True - Phase In, data & clock in phase ; False - Phase out, data 1/2 cycle before clock	*/
	uint8_t		oddParity;			/**< True - Odd Parity ; False - Even Parity, valid only if ParityEnable  is true 				*/
	uint8_t		parityEnable;	/**< Parity check in the data format, True - Checks for parity, False - No Check				*/
	uint8_t		clkHigh;		/**< Sets the polarity of the clock,  True - POLARITY HIGH , False - POLARITY LOW			*/
	uint8_t		waitEnable;		/**< True - if in master mode,	wait for ENA signal from slave, False - do not wait			 */
	#else
	uint32_t	wDelay;				/**< Delay in between two transfers													*/
	uint16_t	charLength:5;			/**< defines the word length- b/w 2-16												*/			   
	uint16_t	lsbFirst:1;			/**< Sets the Shift of bits, TRUE - LSB First ; FALSE - MSB First							*/
	uint16_t	phaseIn:1;			/**< True - Phase In, data & clock in phase ; False - Phase out, data 1/2 cycle before clock	*/
	uint16_t	oddParity:1;			/**< True - Odd Parity ; False - Even Parity, valid only if ParityEnable  is true 				*/
	uint16_t	parityEnable:1;	/**< Parity check in the data format, True - Checks for parity, False - No Check				*/
	uint16_t	clkHigh:1;		/**< Sets the polarity of the clock,  True - POLARITY HIGH , False - POLARITY LOW			*/
	uint16_t	waitEnable:1;		/**< True - if in master mode,	wait for ENA signal from slave, False - do not wait			 */
	#endif
} Spi_ConfigDataFmt;


#ifdef EXTENDED_CHIP_SELECT	
struct sExtGpiochipselect
{
	
	uint8_t		enableMask;			/**< Flag to determine whether gpio and spi internal cs should be used as chip select */
	uint16_t	extGpioPinNo[8];	/**< GPIO Pin numbers used as chip select */	
	uint16_t	extGpioPinClock[8];	/**< GPIO Pin clock numbers  */	
	void		*extGpioPinPort[8];	/**< GPIO Pin ports  */
	//uint16_t	extSpiPinMask[8];	/**< SPI Pin masks used as chip select */ 
};
#endif

/**
 *  \brief Spi Hardware Configuartion Structure
 */
typedef struct Spi_HWConfigData 
{
#ifdef KUKU
    Spi_CommMode	masterOrSlave;			/**< True - Slave Mode , False - Master Mode								*/
    uint8_t				clkInternal;			/**< True - Selects Internal clock source ; False - Selects external  clock source	*/
    uint8_t				enableHighZ;			/**< whether ENA signal should be tristated when inactive - True ; or it should bear a value - False	*/
    Spi_PinOpMode	pinOpModes;				/**< Spi Operation Modes												*/
    uint32_t		waitDelay;				/**< True - enable format delay between 2 consecutive transfers ; False - No delay b/w 2 transfers 	*/
    Spi_ConfigDataFmt   configDatafmt[4];	/**< Data Format Configuration values                                     */
    uint8_t		csDefault;				/**< Default chip select pattern											*/
    uint8_t		gpioChipselectFlag;		/* Flag to determine whether gpio should be used as chip select           */
    uint16_t		gpioPinNo;				/* GPIO Pin number used as chip select                                    */
	uint16_t		gpioPinClock;	/**< GPIO Pin clock numbers  */	
	void			*gpioPinPort;	/**< GPIO Pin ports  */
    //Spi_DelayParam	delay;					/**< Spi delay registers value											*/
#else
	uint8_t		masterOrSlave:1;
	uint8_t		clkInternal:1;
	uint8_t		enableHighZ:1;
	uint8_t		pinOpModes:2;
	uint8_t		gpioChipselectFlag:1;
	uint8_t		rsvd:2;
    uint8_t		csDefault;				/**< Default chip select pattern											*/
    uint32_t		waitDelay;				/**< True - enable format delay between 2 consecutive transfers ; False - No delay b/w 2 transfers 	*/
    uint16_t		gpioPinNo;				/* GPIO Pin number used as chip select                                    */
	uint16_t		gpioPinClock;	/**< GPIO Pin clock numbers  */	
	void			*gpioPinPort;	/**< GPIO Pin ports  */
    Spi_ConfigDataFmt   configDatafmt[4];	/**< Data Format Configuration values                                     */
	
#endif
	#ifdef EXTENDED_CHIP_SELECT	
	struct sExtGpiochipselect *extGpiochipSelect; /* NULL=>no extended chip select */
	/* Table of extended chip select (SPI and GPIO) mapping */		
	#endif
}Spi_HWConfigData;

/**
  * \brief Spi Transaction structure
  *
  * This structure holds the information needed to carry out a transaction on
  * an Spi bus to a slave device.
  */
typedef struct Spi_DataParam 
{
    uint8_t			*outBuffer;		/**< Data buffer - During transceieve operation this buffer data is transmitted	*/
    uint8_t			*inBuffer;		/**< Data buffer - During transceive operation read data is stored in this buffer	*/
    uint32_t		bufLen;			/**< Length of the buffer 												*/
    uint32_t		chipSelect;		/**< chipselect to be selected											*/
    Spi_DataFormat	dataFormat;		/**< which data format to select											*/
    uint32_t		flags;			/**< Flags  - To indicate the Read / Write modes of operation					*/
    void			*param;			/**< Extra paramerter													*/
} Spi_DataParam;

/* -------- per-instance configs -------- */

/**
 *  \brief Spi Per instance configuration struct
 */
typedef struct Spi_Params 
{
	Spi_OpMode			opMode;				/**< Driver operation mode						*/
	int8_t				loopbackEnabled;	/**< To enable/disable loop back				*/
	uint16_t			cpuEventNumber;		/**< cpu event number configured for spi event		*/
	uint32_t			outputClkFreq;		/**< output clock frequency from Spi Module , transmits bitrate  in bits per second	*/
	uint32_t			polledModeTimeout;	/**< Polled mode timeout						*/
	Spi_HWConfigData	spiHWCfgData;		/**< Hardware configuration						*/
	void				*hDma;				/**< Handle for dma							*/
	//uint32_t			inputFrequency;
	//int					enableCache;		/**< enable/disable cache operations in driver		*/
} Spi_Params;

/**
 *  \brief Default Spi_Params struct
 *
 *  const Spi_Params Spi_PARAMS = {
 *
 *      Spi_OpMode_POLLED,              opMode
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
 *          Spi_CommMode_MASTER,         masterOrSlave
 *
 *          1,                           clkInternal
 * 
 *          0,                           enableHighZ
 * 
 *          Spi_PinOpMode_SpiSCS_4PIN,   pinOpModes
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
 *          0xff,                csDefault
 *
 *          {
 *              {
 *              0x0,             wDelay
 * 
 *              0x0,             charLength
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
 *	#ifdef EXTENDED_CHIP_SELECT	
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
extern const Spi_Params Spi_PARAMS;


/**
 *  \brief Spi IOM_Fxns table
 *
 *  Driver function table to be used by applications. 
 */
extern const IODEV_Fxns Spi_IODEVFXNS;

/**
 *  \brief BB_Spi IOM_Fxns table
 *
 *  Driver function table to be used by applications. 
 */
extern const IODEV_Fxns BB_Spi_IODEVFXNS;

/* -------- defines -------- */

/**
 *  \brief   Local Buffer data allocation for transcieve operation. 
 */
#ifndef Spi_BUFFER_DATA_SIZE
#define Spi_BUFFER_DATA_SIZE (32u)
#endif

/**
 *  \brief   Max channels supported
 * 
 *          Max Spi driver opens - which means number of s/w channels which 
 *          can be opened for a single instance of Spi
 */
#define Spi_NUM_CHANS (1)
/**
 * \brief   Macro used to enable/Disable CSHOLD  
 */
#define Spi_CSHOLD ((0x1u))
/**
 * \brief   Macro for CSHOLD multi transceive
 *
 *  CSHOLD will be selected [pulled low]after the trasceive operation and
 *  this is valid if Spi_CSHOLD flag is enabled. Also if Chip select value is
 *  changed in the next transceive operation then also this flag becomes
 *  invalid for the previous rtansceive operation i.e CS lines comes high. 
 */
#define Spi_CSHOLD_FOR_MULTI_TRANSCEIVE ((0x2u))

/** Macro used to set gpio pin as active low                              */
#define Spi_LOW   (0x0u)

/** Macro used to set gpio pin as active high                             */
#define Spi_HIGH  (0x1u)

/**
 *  \brief    Initializes Spi instances which are statically allocated
 *
 *  This function needs to be be called at part of BIOS initialization by
 *  setting initFxn for that particular UDEV instance or by calling this
 *  function as part of user specific initFxn.
 */
void stSpiInit(void);

/**
 *  \brief    Initializes Bit Bang Spi instances which are statically allocated
 *
 *  This function needs to be be called at part of BIOS initialization by
 *  setting initFxn for that particular UDEV instance or by calling this
 *  function as part of user specific initFxn.
 */
void stBB_SpiInit(void);

#ifdef __cplusplus
}
#endif

#endif

