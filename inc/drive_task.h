/**
* @file drive_task.h
* @brief drive task definitions
*
* @author Evgeny Altshuler
*
* @version 0.0.1
* @date 10.03.2014
*/
#ifndef _DRIVETASK_H
#define _DRIVETASK_H

#include <stdint.h>
#include <freertos.h>
//#include <task.h>
//#include <semphr.h>
#include <queue.h>
#include "uart.h"

#define N_CTL			2


struct sCtlServerParam 
{
	int ctlId;								/**< Controller identifier 0..(N_CTL-1)	*/			
	int devId;								/**< UART device identifier		*/								
	Uart_BaudRate		baud;				/**< Baud of Operation			*/
	Uart_NumStopBits	stopBits;			/**< Stopbits of Operation		*/
	Uart_CharLen		charLen;			/**< Character Length			*/
	Uart_Parity			parity;				/**< Parity of Operation		*/
};

extern xQueueHandle ctlOutQ[N_CTL];
extern xQueueHandle ctlInQ[N_CTL];

extern const char *ctlRxServerQueueName[N_CTL]; 
extern const char *ctlTxServerQueueName[N_CTL];






#define DRIVER_1_2_ID		0x01
#define DRIVER_1_ID			0x02
#define DRIVER_2_ID			0x03

#define AXIS_1					0x31	// 1
#define ON						0x31	// 1
#define OFF						0x30	// 0
#define GET						0x3F	// ?
#define SPACE					0x20	// Space
#define LF						0x0A	//LF

#define MOTOR_COMMAND			0
#define POSITION_COMMAND		1
#define MOVE_COMMAND			2
#define REF_MODE_COMMAND		3
#define VELOCITY_COMMAND		4
#define MOVE_REL_COMMAND		5
#define ON_TARGET_COMMAND		6


#define FTOA_OK			0
#define FTOA_TOO_LARGE	1
#define FTOA_TOO_SMALL  2


/* Host Commands*/
#define DRV_STATE_NO_CMD			1
#define DRV_STATE_FWD				2

#define DRV_STATE_MOTOR_ON			3
#define DRV_STATE_MOTOR_OFF			4
#define DRV_STATE_MOTOR_GET			5

#define DRV_STATE_POS_SET			6
#define DRV_STATE_POS_GET			7

#define DRV_STATE_MOVE_SET			8
#define DRV_STATE_MOVE_GET			9

#define DRV_STATE_REF_MODE_ON		10
#define DRV_STATE_REF_MODE_OFF		11
#define DRV_STATE_REF_MODE_GET		12

#define DRV_STATE_VELOCITY_SET		13
#define DRV_STATE_VELOCITY_GET		14

#define DRV_STATE_MOVE_REL_SET		15

#define DRV_STATE_ONT_GET			16


#define CURRENT_OFFSET		2.0
#define MOTION_ACTIVE		1
#define MOTION_NOT_ACTIVE	0
#define DRIVE_ERROR_BITS    0xFF //TBD



//#define   _IQ15toF(A)      (float) (A / (float)32768.0)
#define   _IQ15toF(A)      (float) (A *(float)((double)1.0/(double)32768.0))


#define BRAKE_1_OPEN		GPIO_SetBits(BREAK_M1N_GPIO_PORT,BREAK_M1N_PIN)		// Open Brake 1
#define BRAKE_1_CLOSE		GPIO_ResetBits(BREAK_M1N_GPIO_PORT,BREAK_M1N_PIN)		// Close Brake 1
#define BRAKE_2_OPEN		GPIO_SetBits(BREAK_M2N_GPIO_PORT,BREAK_M2N_PIN)		// Open Brake 2
#define BRAKE_2_CLOSE		GPIO_ResetBits(BREAK_M2N_GPIO_PORT,BREAK_M2N_PIN)		// Close Brake 2

#define DRIVER_INIT_STATUS \
{\
		DRIVER_1_ID,		/* Master Drive  */\
		0,					/* Mode Command */\
		0,					/* Driver 1 Status */\
		0,					/* Driver 2 Status */\
		DRV_STATE_NO_CMD,	/* Driver 1 State */\
		DRV_STATE_NO_CMD,	/* Driver 2 State */\
		DRV_STATE_NO_CMD,	/* Driver 1 Next State */\
		DRV_STATE_NO_CMD,	/* Driver 2 Next State */\
		0,					/* Driver 1 Packet Sent   */\
		0,					/* Driver 2 Packet Sent  */\
		0,					/* Driver 1 Timeout Exp Counter  */\
		0,					/* Driver 2 Timeout Exp Counter */\
		0,					/* Position 1 Command */\
		0,					/* Position 2 Command  */\
		0,					/* Target Position  Command  */\
		0.0,				/* Target Position X Command  */\
		0.0,				/* Target Position Y Command  */\	
		0.0,				/* Host Position X Command  */\
		0.0,				/* Host Position Y Command  */\			
		0.0,				/* Velocity Command */\
		0,					/* Velocity Update Flag*/\
		0,					/* Motion X Status Flag*/\
		0,					/* Motion Y Status Flag*/\		
		0,					/* Current Velocity */\
}

union DriverData{
	uint32_t    data1;
	float 		data2;
};



struct sDriverCmd
{
uint16_t			cmd;
uint8_t				axis;
uint8_t				parameters;
union DriverData	DrvData;
};


struct sDriverStatus
{
	uint8_t  MasterDrive;
	uint8_t  ModeCommand;
	uint16_t Status1;
	uint16_t Status2;
	uint16_t State1;
	uint16_t State2;
	uint16_t NextState1;
	uint16_t NextState2;
	uint8_t  Drive1PacketSent;
	uint8_t  Drive2PacketSent;
	uint8_t  Drive1TimeoutCnt;
	uint8_t  Drive2TimeoutCnt;
	uint32_t Position1Cmd;
	uint32_t Position2Cmd;
	uint32_t TargetPosCmd;
	float    TargetPosXCmd;
	float    TargetPosYCmd;
	float    HostPosXCmd;
	float    HostPosYCmd;
	float    VelocityXCmd;
	float	 VelocityYCmd;
	uint8_t  MotionXStatus;
	uint8_t  MotionYStatus;
	int32_t  CurrentVelocity;
};

typedef union {
long L;
float F;
} LF_t;


#ifdef __cplusplus
	extern "C" {
#endif
void ctlRxServerTask(void *para);

void ctlTxServerTask(void *para);
unsigned char  BuildDrivePckt(char *buf, uint16_t data, uint16_t chan);
void TIM_Config(void);
int SendCmdToDrive(uint16_t channel,  uint16_t data);
int  DriveTimeout(TIM_TypeDef* TIMx, uint32_t timeout);
int sendPacketToDriveInt(void *packet, uint16_t hdr, uint32_t timeout, uint16_t dst_chan);
int sendPacketToDrive(void *packet, uint32_t timeout, uint16_t dst_chan);
char *ftoa(float f, int *status);

#ifdef __cplusplus
}
#endif


#endif

