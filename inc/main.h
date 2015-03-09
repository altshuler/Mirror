/**
  ******************************************************************************
  * @file    main.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    07-October-2011 
  * @brief   This file contains all the functions prototypes for the main.c 
  *          file.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx.h"
#include "stm32f2x7_eth_bsp.h"
#include "FreeRTOS.h"			// E.A. Line added
#include "Queue.h"				// E.A. Line added
#include "eth_Rx_Task.h"		// E.A. Line added
#include "hcmdtask.h"			// E.A. Line added
#include "hcmd.h"				// E.A. Line added
#include "Msg_type.h"			// E.A. Line added
#include "Drive_task.h"			// E.A. Line added
#include "board.h"				// E.A. Line added
#include "cbit_task.h"			// E.A. Line added
#include "irqhndl.h"			// E.A. Line added
#include "root_task.h"			// E.A. Line added
#include "readout_task.h"		// E.A. Line added
#include "ip_param_config.h"
#include "Motion_task.h"


/* Exported types ------------------------------------------------------------*/
/*	E.A.  SPI buffer  added*/
//#define SPI_Buf_Size	200

/*--------------- Tasks Priority -------------*/     
#define LED_TASK_PRIO		( tskIDLE_PRIORITY + 1 )
#define CBIT_TASK_PRIO  	( tskIDLE_PRIORITY + 2 )		// D.A. Line added
#define HCMD_TASK_PRIO		( tskIDLE_PRIORITY + 3 )		// E.A. Line added
#define DHCP_TASK_PRIO		( tskIDLE_PRIORITY + 6 ) 
#define ETH_TX_TASK_PRIO    ( tskIDLE_PRIORITY + 7 )		// E.A. Line added
#define ETH_RX_TASK_PRIO    ( tskIDLE_PRIORITY + 8 )		// E.A. Line added

//#define SPI_TX_TASK_PRIO     ( tskIDLE_PRIORITY + 6 )		// E.A. Line added
#define DRIVE_TX_TASK_PRIO  ( tskIDLE_PRIORITY + 11 )		// E.A. Line added
#define DRIVE_RX_TASK_PRIO  ( tskIDLE_PRIORITY + 12 )		// E.A. Line added
#define DRIVE_INT_TASK_PRIO ( tskIDLE_PRIORITY + 10 )		// E.A. Line added

#define MOTION_TASK_PRIO    ( tskIDLE_PRIORITY + 6 )		// E.A. Line added
#define READ_TX_TASK_PRIO   ( tskIDLE_PRIORITY + 9 )		// E.A. Line added
#define READ_RX_TASK_PRIO   ( tskIDLE_PRIORITY + 2 )		// E.A. Line added
#define ENC_TASK_PRIO		( tskIDLE_PRIORITY + 8 )		// E.A. Line added

#define ROOT_TASK_PRIO    	( tskIDLE_PRIORITY + 15 )		// E.A. Line added


extern struct netif xnetif;		//E.A.  line added


//typedef struct Spi_TX_Buffer {
//  u8  TxBuffer [SPI_Buf_Size];		/* SPI Buffer for transmitting */
//  u16 Datalen;						/* Command length */
//} Spi_TX_Buffer;

extern xQueueHandle 		hCmdMbx;
extern xQueueHandle 		intHostTXQueue;
extern xQueueHandle     	MotionQueue;
extern xQueueHandle			DriveIntQueue;
extern xQueueHandle 		readoutOutQ;
extern xQueueHandle 		readoutInQ;

extern const char *ctlRxServerQueueName[N_CTL]; 
extern const char *ctlTxServerQueueName[N_CTL];

/* Exported constants --------------------------------------------------------*/

//#define USE_LCD        /* enable LCD  */  
//#define USE_DHCP       /* enable DHCP, if disabled static address is used*/
   
/* Uncomment SERIAL_DEBUG to enables retarget of printf to  serial port (COM1 on STM32 evalboard) 
   for debug purpose */   
//#define SERIAL_DEBUG    

/* MAC ADDRESS*/
#define MAC_ADDR0   02
#define MAC_ADDR1   00
#define MAC_ADDR2   00
#define MAC_ADDR3   00
#define MAC_ADDR4   00
#define MAC_ADDR5   00
 
/*Static IP ADDRESS*/
#define IP_ADDR0   192
#define IP_ADDR1   168
#define IP_ADDR2   1
#define IP_ADDR3   1
   
/*NETMASK*/
#define NETMASK_ADDR0   255
#define NETMASK_ADDR1   255
#define NETMASK_ADDR2   255
#define NETMASK_ADDR3   0

/*Gateway Address*/
#define GW_ADDR0   192
#define GW_ADDR1   168
#define GW_ADDR2   1
#define GW_ADDR3   1  

/* MII and RMII mode selection, for STM322xG-EVAL Board(MB786) RevB ***********/
//#define RMII_MODE  // User have to provide the 50 MHz clock by soldering a 50 MHz
                     // oscillator (ref SM7745HEV-50.0M or equivalent) on the U3
                     // footprint located under CN3 and also removing jumper on JP5. 
                     // This oscillator is not provided with the board. 
                     // For more details, please refer to STM3220G-EVAL evaluation
                     // board User manual (UM1057).

                                     
#define MII_MODE

/* Uncomment the define below to clock the PHY from external 25MHz crystal (only for MII mode) */
#ifdef 	MII_MODE
 #define PHY_CLOCK_MCO
#endif

/* STM322xG-EVAL jumpers setting
    +==========================================================================================+
    +  Jumper |       MII mode configuration            |      RMII mode configuration         +
    +==========================================================================================+
    +  JP5    | 2-3 provide 25MHz clock by MCO(PA8)     |  Not fitted                          +
    +         | 1-2 provide 25MHz clock by ext. Crystal |                                      +
    + -----------------------------------------------------------------------------------------+
    +  JP6    |          2-3                            |  1-2                                 +
    + -----------------------------------------------------------------------------------------+
    +  JP8    |          Open                           |  Close                               +
    +==========================================================================================+
  */




/* Exported define ------------------------------------------------------------*/

#define HCMD_QUEUE_SIZE			8
#define CTL_TX_QUEUE_SIZE 		16
#define CTL_RX_QUEUE_SIZE 		8
#define DRIVE_INT_QUEUE_SIZE 	8
#define MOTION_QUEUE_SIZE 		8
#define HOST_TX_QUEUE_SIZE 		5//8
#define READOUT_TX_QUEUE_SIZE	8
#define READOUT_RX_QUEUE_SIZE	8


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */  
void Time_Update(void);
void Delay(uint32_t nCount);
void drive1_tx_task(void *para);
void drive2_tx_task(void *para);
void DriveInterpTask(void *para);
void enc_tx_task(void *para);
void motion_task(void *para);
void GPIO_Config(void);
void ToggleLed4(void * pvParameters);



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

