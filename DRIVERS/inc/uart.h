/**
* @file uart.h
* @brief uart driver definitions
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 16.04.2012
*/
#ifndef _UART_H
#define _UART_H

#include <stdint.h>
#include <iodev.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \brief  Uart Baud Rate
 */

#define Uart_BaudRate_2_4K   ((uint32_t)2400)     /**< Baudrate 2400    bps   */
#define Uart_BaudRate_4_8K   ((uint32_t)4800)     /**< Baudrate 4800    bps   */
#define Uart_BaudRate_9_6K   ((uint32_t)9600)     /**< Baudrate 9600    bps   */
#define Uart_BaudRate_19_2K  ((uint32_t)19200)    /**< Baudrate 19200    bps   */
#define Uart_BaudRate_38_4K  ((uint32_t)38400)    /**< Baudrate 38400    bps   */
#define Uart_BaudRate_57_6K  ((uint32_t)57600)    /**< Baudrate 57600    bps   */
#define Uart_BaudRate_115_2K ((uint32_t)115200)   /**< Baudrate 115200    bps   */
#define Uart_BaudRate_230_4K ((uint32_t)230400)   /**< Baudrate 230400    bps   */
#define Uart_BaudRate_256K ((uint32_t)256000)     /**< Baudrate 256000    bps   */
#define Uart_BaudRate_460_8K ((uint32_t)460800)   /**< Baudrate 460800    bps   */
#define Uart_BaudRate_921_6K ((uint32_t)921600)   /**< Baudrate 921600    bps   */

typedef uint32_t Uart_BaudRate; 

/**
 *  \brief  Uart Character Length
 *
 */
typedef uint8_t Uart_CharLen;

#define Uart_CharLen_5 ((Uart_CharLen)(5)) 	 /**< Character Length 5  bits	*/
#define Uart_CharLen_6 ((Uart_CharLen)(6)) 	 /**< Character Length 6  bits	*/
#define Uart_CharLen_7 ((Uart_CharLen)(7)) 	 /**< Character Length 7  bits	*/
#define Uart_CharLen_8 ((Uart_CharLen)(8)) 	 /**< Character Length 8  bits	*/
#define Uart_CharLen_9 ((Uart_CharLen)(9)) 	 /**< Character Length 9  bits	*/


/**
 *  \brief  Uart Flow Control
 *
 */
typedef uint8_t Uart_FcType;

#define Uart_FcType_NONE ((Uart_FcType)(0))	/**< No Flow Control 	   */
#define Uart_FcType_SW ((Uart_FcType)(1))	/**< Software Flow Control  */
#define Uart_FcType_HW ((Uart_FcType)(2))	/**< Hardware Flow Control  */


/**
 *  \brief  Uart Stop Bits
 *
 */
typedef uint8_t Uart_NumStopBits;

#define Uart_NumStopBits_1 ((Uart_NumStopBits)(0))	/**< Stop Bits 1	bits	*/
#define Uart_NumStopBits_0_5 ((Uart_NumStopBits)(0x1))	/**< Stop Bits 0.5	bits	*/
#define Uart_NumStopBits_1_5 ((Uart_NumStopBits)(0x2))	/**< Stop Bits 1.5	bits	*/
#define Uart_NumStopBits_2 ((Uart_NumStopBits)(0x4))	/**< Stop Bits 2	bits	*/


/**  
 *  \brief  Uart OpMode 
 *  
 *  \note   Enumeration of the different modes of operation available for the 
 *          Uart device.
 */ 
typedef uint8_t Uart_OpMode;

#define Uart_OpMode_POLLED			((Uart_OpMode)(0))	/**< Polled Mode 		   */
#define Uart_OpMode_INTERRUPT		((Uart_OpMode)(1))	/**< Interrupt Mode 		   */
#define Uart_OpMode_INTERRUPT_DMA	((Uart_OpMode)(2))	/**< Interrupt and DMA Mode 		   */



/**
 *  \brief  Uart Parity
 *
 */ 
typedef uint8_t Uart_Parity;

#define Uart_Parity_NONE			((Uart_Parity)(0x0u))	/**< Parity Bits NONE	bits	*/
#define Uart_Parity_ODD				((Uart_Parity)(0x2))	/**< Parity Bits ODD	bits	*/
#define Uart_Parity_EVEN			((Uart_Parity)(0x3))	/**< Parity Bits EVEN	bits	*/


/**
 *  \brief  Uart Ioctl commands
 */
typedef enum Uart_IOCTL {
	Uart_IOCTL_SET_BAUD = 128,
	/**< Set baud rate, cmdArg = Sci_BaudRate * 							 */
	Uart_IOCTL_SET_STOPBITS,
	/**< Set number of stop bits, cmdArg = Sci_NumStopBits *				 */
	Uart_IOCTL_SET_DATABITS,
	/**< Set number of Data bits, cmdArg = Sci_CharLen *					 */
	Uart_IOCTL_SET_PARITY,
	/**< Set parity type, cmdArg = Sci_Parity * 							 */
	Uart_IOCTL_SET_FLOWCONTROL,
	/**< Set flowcontrol, cmdArg = Sci_FlowControl *						 */
	Uart_IOCTL_SET_RX_TRIGGER_LEVEL,
	/**< Changing Trigger level, cmdArg = Uart_RxTrigLvl *					 */
	Uart_IOCTL_SET_TX_TRIGGER_LEVEL,
	/**< Changing Trigger level, cmdArg = Uart_TxTrigLvl *					 */
	Uart_IOCTL_RESET_RX_FIFO,
	/**< Resets the Uart HW RX FIFO, cmdArg = NONE							  */
	Uart_IOCTL_RESET_TX_FIFO,
	/**< Resets the Uart HW TX FIFO, cmdArg = NONE							  */
	Uart_IOCTL_CANCEL_CURRENT_IO,
	/**< Cancel the current IO in TX or RX channel, cmdArg = NONE			  */
	Uart_IOCTL_GET_STATS,
	/**< Getting the Uart stats for DDC, cmdArg = Sci_Stats *				 */
	Uart_IOCTL_CLEAR_STATS,
	/**< Clearing the Stats of DDC, cmdArg = NONE							  */
	Uart_IOCTL_FLUSH_ALL_REQUEST,
	/**< Flush all IO requests , cmdArg = NONE								  */
	Uart_IOCTL_SET_POLLEDMODETIMEOUT,
	/**< Set Polled Mode timeout, cmdArg = timeout value in ticks			  */
	Uart_IOCTL_SET_ERROR_CALLBACK,
	/**< Set error callback function, cmdArg =	pointer to callback structure */
	Uart_IOCTL_SET_RX_CALLBACK
	/**< Set rx callback function, cmdArg =	pointer to callback structure */
} Uart_IOCTL;

typedef struct Uart_Callback {
    int (*f)();             	/**< callback function          */
    void *arg;             		/**< callback function argument       */
} Uart_Callback;

/**
 *  \brief Uart Instance configuration parameters
 *
 *  A pointer to such a structure is used when driver is instantiated
 */
typedef struct Uart_Params
{
	uint8_t				cacheEnable;		/**< Driver will use cache APIs   */
	uint8_t				fifoEnable;			/**< Fifo mode of Operation       */
	Uart_OpMode			opMode;				/**< Driver operational mode      */
	uint8_t				loopbackEnabled;	/**< LoopBack Mode flag           */
	Uart_BaudRate		baudRate;			/**< Baudrate of Operation        */
	Uart_NumStopBits	stopBits;			/**< Stopbits of Operation        */
	Uart_CharLen		charLen;			/**< Character Length             */
	Uart_Parity			parity;				/**< Parity of Operation          */
	Uart_FcType			fc;					/**< Flow Control                 */
	
	uint16_t			rxThreshold;		/**< Rx FIFO trigger level        */
	uint16_t			txThreshold;		/**< Tx FIFO trigger level        */
	uint32_t			polledModeTimeout;	/**< Timeout used in polled mode  */
	uint16_t			softTxFifoThreshold;
} Uart_Params;
    
/** 
 *	\brief Number of channels per uart instance
 */
#define Uart_NUM_CHANS (2)

/**
 *  \brief Uart FIFO Size
 */
#define Uart_FIFO_SIZE (16)

/**
 *  \brief Default Uart_Params struct
 *
 *  Following values are defaults and application can assign and change
 *  interested parameters.
 *
 *	const struct Uart_Params UART_PARAMS = 
 *	 {
 *		 0, 
 *		 0, 
 *		 Uart_OpMode_INTERRUPT_DMA, 
 *		 0,
 *		 Uart_BaudRate_9_6K,
 *		 Uart_NumStopBits_1,
 *		 Uart_CharLen_8,
 *		 Uart_Parity_NONE,
 *		 1, 
 *		 1, 
 *		 Uart_FcType_NONE,						
 *		 0xffffffff
 *	 };
 */
extern const Uart_Params UART_PARAMS;

/**
 *  \brief Usart IODEV_Fxns table
 *
 *  Driver function table to be used by applications. 
 */
extern const IODEV_Fxns Usart_IODEVFXNS;

/**
 *  \brief  Uart channel config parameters
 *
 *  Uart Channel Params passed to Uart_open function
 */
typedef struct Usart_ChanParams {
    void *hDma;
}Usart_ChanParams;

/**
 *  \brief Uart Statistics Collection Object
 *
 *  Statistics are collected on a per-device basis for Uart.
 */
typedef struct Usart_Stats 
{
    uint32_t rxBytes;             /**< Number bytes received          */
    uint32_t txBytes;             /**< Number bytes transmitted       */
    uint32_t overrun;             /**< Number of overrun errors       */
    uint32_t rxTimeout;           /**< Number of Rx timeouts          */
    uint32_t rxFramingError;      /**< Number of Rx Framing errors    */
    uint32_t rxBreakError;        /**< Number of Rx Break Errors      */
    uint32_t rxParityError;       /**< Number of Rx Parity Errors     */
} Usart_Stats;

/**
 *  \brief    Initializes Uart instances which are statically allocated
 *
 *  This function needs to ve be called at part of OS initialization by
 *  setting initFxn for that particular UDEV instance or by calling this
 *  function as part of user specific initFxn.
 */
void stUsartInit(void);


#ifdef __cplusplus
}
#endif


#endif


