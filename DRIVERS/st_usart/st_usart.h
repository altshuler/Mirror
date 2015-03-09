/**
* @file st_usart.h
* @brief usart IO driver
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 08.05.2012
*/
#ifndef _ST_USART_H
#define _ST_USART_H

#include <stddef.h>
#include <stdint.h>
#include <freertos.h>
#include <semphr.h>
#include <sem.h>
#include <uart.h>
#include "stm32f2xx.h"

#ifndef N_USART
#define N_USART	(6)
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifndef Usart_MAX_BUFFER_SIZE
#define Usart_MAX_BUFFER_SIZE	(16) /**< UART driver TX internal buffer size	*/
#endif
#ifndef Usart_TPOLL_MSECS
#define Usart_TPOLL_MSECS		(1)  /**< UART driver polling interval in ticks	*/
#endif


/**
 *  \brief   Usart driver state enums used to track the driver state
 */
typedef uint8_t Usart_DriverState;

#define Usart_DriverState_DELETED ((Usart_DriverState)(0))
#define Usart_DriverState_CREATED ((Usart_DriverState)(1))
#define Usart_DriverState_INITIALIZED ((Usart_DriverState)(2))
#define Usart_DriverState_OPENED ((Usart_DriverState)(3))
#define Usart_DriverState_CLOSED ((Usart_DriverState)(4))
#define Usart_DriverState_DEINITIALIZED ((Usart_DriverState)(5))
#define Usart_DriverState_POWERED_DOWN ((Usart_DriverState)(6))


/**
 *  \brief  Mode is INPUT or OUTPUT
 */

typedef uint8_t Usart_IoMode;
#define Usart_INPUT ((Usart_IoMode)(1))
#define Usart_OUTPUT ((Usart_IoMode)(2))


/**
 *  \brief  Internal data structure maintaining Uart device instance options
 */
typedef struct Usart_DevParams 
{
    Uart_BaudRate       baudRate;
    uint16_t   			stopBits:3;
   	uint16_t 			charLen:4;
    uint16_t         	parity:2;
    uint16_t			fc:2;
    uint16_t			fifoEnable:1;
    uint16_t			loopbackEnabled:1;
    uint16_t			softTxFifoThreshold;    
} Usart_DevParams;

/**
 *  \brief  Usart structure to hold the instance specific information.
 */
typedef struct Usart_HwInfo 
{
	USART_TypeDef	*baseAddress;
	uint32_t		inputFrequency;
	uint16_t		cpuEventNumber;
} Usart_HwInfo;

/**
 *  \brief  Structure of the internal buffer
 */
typedef struct Usart_InternalBuffer 
{
    uint32_t buffer[Usart_MAX_BUFFER_SIZE];
    /**<The actual internal buffer that holds the data to be
    transferred.The size is configured through Usart_MAX_BUFFER_SIZE;
    Also note that this buffer implementation is not circular one */

    uint8_t  *bufFillPtr;
    /**< This pointer is used to fill up the temp internal
    buffer from task context from all IOPs; Please note that
    the internal buffer will be filled till it is able to
    accomodate the data. Once it is full or the current application
    request makes to exceed the buffer limit, that request will be queued
    */

    uint8_t  *bufReadPtr;
    /**<This pointer is used to read and pump the data out to the
    SCI transmitter in ISR context*/

    uint8_t bufferIndex;
    /**<This index maintains the count of bytes filled into the
    internal buffer from application request from Task context*/

    uint8_t xferActual;
    /**This maintains the actual no. of bytes transferred from
    internal buffer. When this equals the bufferIndex, it means that
    we are done with current buffer and reset the members to
    init values*/
}Usart_InternalBuffer;

/**
 *  \brief  Structure of the channel object
 */
typedef struct Usart_ChanObj 
{
    Usart_DriverState		status;
    /**< Flag to tell channel object resource is used or free                 */

    Usart_IoMode			mode;
    /**< mode of the channel i.e INPUT or OUTPUT mode                         */

    uint16_t				ioCompleted;
    /**<                           */
	
    IODEV_TiomCallback		cbFxn;
    /**< Callback to the application                                          */

    void					*cbArg;
    /**< Argument to be passed in the callback                                */

    QUE_Obj					queuePendingList;
    /**< pending Iop List head                                                */

    IODEV_Packet			*activeIOP;
    /**< Current Active Packet under progress in the channel                  */

    uint8_t					*activeBuf;
    /**< Buffer address of the current packet                                 */

    uint32_t				bytesRemaining;
    /**< Bytes remaining for the completion of the current packet             */

    uint32_t				chunkSize;
    /**< No. of bytes to be transferred per operation (i.e FIFO Size or 1)    */

    void					*devHandle;
    /**< Handle to the Device                                                 */

    uint32_t				errors;
    /**< Counts how many errors encountered per IOP request                   */

    void                     *hDma;
    /**< Handle to DMA object                                                 */

	void *hDmaHandler;
    /**< Handle to DMA interrupt handler object   */

	void *hDmaHandlerArg;
    /**< Handle to DMA interrupt handler argument object   */


    Usart_InternalBuffer	iBuffer;
    /**< Internal Buffer                                                      */

	uint16_t			irqNumber;
	/**< Hardware interrupt Number                                            */
}Usart_ChanObj;

/**
 *  \brief module-wide state
 *
 *  inUse in module state points to an array whose size is soc specific
 *  Pointer used to match IDriver codebase.
 */
typedef struct Usart_Module_State
{
    uint8_t *inUse;
    /**< Mantain inUse state of each sci device                              */
} Usart_Module_State;

/**
 *  \brief per-instance state
 */
typedef struct Usart_Object
{
	uint32_t			instNum;
	/**< Instance number or device ID of the SCI                             */

	Uart_OpMode			opMode;
	/**< Mode of operation                                                    */

	Usart_DriverState	devState;
	/**< Driver State (deleted or created)                                    */

	uint8_t			cacheEnable;
	/**< Submitted buffers are in cacheable memory                            */

	Usart_DevParams		devParams;
	/**< Uart device Params                                                   */

	Usart_ChanObj		xmtChanObj;
	/**< transmiiter channel                                                  */

	Usart_ChanObj		rcvChanObj;
	/**< receiver channel                                                     */

	Usart_HwInfo		deviceInfo;
	/**< Instance specific information                                        */

	void 				*hDmaRes;
	/**< Instance specific information                                        */

	uint16_t			irqNumber;
	/**< Hardware interrupt Number                                            */

	Usart_Stats			stats;
	/**< UART specific stats                                                  */


	uint32_t			polledModeTimeout;
	/**< Timeout used in polled mode - could be changed by an IOCTL           */

	xSemaphoreHandle	syncSem;
	/**< sync semaphore object(used in the polled mode transfer for sync      *
	 * between multiple tasks IO submit requests)                             */

	Uart_Callback		rxCallback;

} Usart_Object;

/* -------- shared functions -------- */

typedef int (*Usart_Isr)(void *arg);


extern int Usart_localCompleteCurrentIO (Usart_ChanObj *chanHandle);

int usart_localStartDmaTransfer(Usart_ChanObj *chanHandle, uint32_t cmd);


#ifdef __cplusplus
}
#endif


#endif


