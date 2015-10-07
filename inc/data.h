#ifndef	DATA_H
#define	DATA_H

#include <stdint.h>
#include "stm32f2xx.h"
#include "Eeprom.h"
#include "AbsEncoderSSI.h"

/*************************************************************************\
*******************    Command Access Protection     **********************
\*************************************************************************/

/*Ibit States*/
#define IBIT_START 		0
#define IBIT_ONGOING 	1
#define IBIT_FINISHED   2

/*System States*/
#define SYS_STATE_INIT 		1
#define SYS_STATE_STDBY 	2
#define SYS_STATE_OPERATE 	3
#define SYS_STATE_MAINT 	4
#define SYS_STATE_RESET 	5
#define SYS_STATE_SHTDWN 	6

/*System Sub States*/
#define SYS_STPR_1_SELECT 		0
#define SYS_STPR_2_SELECT 		1
#define SYS_STPR_1_2_SELECT 	2
#define SYS_STPR_JOYSTICK		3


#define STATUS_OK	0
#define STATUS_FAIL	1

/*State Transition Results*/
#define STATE_RES_OK   0
#define UNKNOWN_CODE   1                             
#define ILLEGAL_TRANS  2            
#define SUB_ST_ERROR   3                        
#define SCAN_MODE_ERR  4                      
#define UNKNOWN_REASON 5

/*Action Complete Flag*/
#define ACTION_COMPLETE 0
#define ACTION_NOT_COMPLETE 1

#define EMERGENCY_1_ON	1
#define EMERGENCY_1_OFF	(~EMERGENCY_1_ON)
#define EMERGENCY_2_ON	(1<<1)
#define EMERGENCY_2_OFF	(~EMERGENCY_2_ON)

#define MIN_READOUT_RATE 250
#define MAX_READOUT_RATE 1000	//4000

#define MIN_ABS_POS 	-10.0 //mm
#define MAX_ABS_POS  	10.0  //mm
#define MIN_ANGLE 		-100000.0 //uRad
#define MAX_ANGLE  		100000.0  //uRad
#define MAX_VELOCITY 	564.0 //uRad/sec
#define MIN_VELOCITY	0.0

#define PIN_POS_OPEN		1
#define PIN_POS_CLOSE		0
#define PIN_CMD_NA			0
#define PIN_CMD_CLOSE		1
#define PIN_CMD_OPEN		2
#define MAX_PIN_CURRENT		0x65
#define PIN_CLOSED_POSITION	0x144	//0x1F4	//0x600
#define PIN_OPENED_POSITION	0x80	//0x6E	//0x100

#define  PEDESTAL_MOVE  	1<<18 //Pedestal Move  :Current pedestal state     0 - stop       1 - moving
#define  PEDESTAL_STOP  	0<<18 

#define  PEDESTAL_POS_DIR  1<<19 //Pedestal Move dir  :Current pedestal dir     0 - to negative edge (to left)       1 - to positive edge (to right)
#define  PEDESTAL_NEG_DIR  0<<19 

#define  PEDESTAL_NEAR_EDGE  	1<<20 //Pedestal in Sector Edge   : 0 - pedestal far from edge  , 1 - near edge (+/- 0.05 deg - TBR)
#define  PEDESTAL_FAR_EDGE  	0<<20 

#define  EDGE_PROXIMITY 	100  //100 counts= 0.0977 deg

#define PI      		3.14159265358979f	//3.1415926535897932384626433832795029
#define XRADIUS			0.0001768718	 //0.0001754564	//0.000177
#define YRADIUS			0.0001776837	//0.0001763852	//0.000177
#define MICRO			1000000.0
#define BCKL_X_OFFSET		50.0*XRADIUS
#define BCKL_Y_OFFSET		-50.0*YRADIUS



#define MIRROR_DEFAULT_PARAMS \
{\
		SYS_STATE_INIT,		/* System State */\
		SYS_STPR_1_SELECT,	/* Stepper Select */\
		0.0,				/* AxisYPosition  */\
		0.0,				/* AxisXPosition */\
		25.0,				/* Controller Temperature */\
		24.0,				/* Protected Voltage */\
		24.0,				/* Motor Voltage */\
		12.0,				/* 12V Voltage */\
		5.0,				/* 5V Voltage */\
		STATUS_OK,			/* PDU Status */\
		STATUS_OK,			/* Network Status */\
		STATUS_OK,			/* Driver 1  Status */\	
		STATUS_OK,			/* Driver 2  Status */\
		30.0,				/* Driver 1  Temperature */\
		30.0,				/* Driver 2  Temperature */\
		STATUS_OK,			/* Brake 1 Status */\
		STATUS_OK,			/* Brake 2 Status */\
		1,					/* Brake 1 State */\
		1,					/* Brake 2 State */\
		10,					/* Brake 1 Counter */\
		10,					/* Brake 2 Counter */\
		0.0,				/*Conroller Current */\	
		STATUS_OK,			/*Absolute Encoder X Failure */\
		STATUS_OK,			/*Flash test Result */\	
		STATUS_OK,			/*RAM test Result */\	
		STATUS_OK,			/*CPU Reset Event */\	
		STATUS_OK,			/*ALL OK FLAG */\	
		STATUS_OK,			/*Absolute Encoder Y Failure */\
		0x3f,				/*Limit switches states  */\
		STATUS_OK,			/*Safety Sw States */\
		1000,				/*RS422 Current Message Rate */\
		ACTION_COMPLETE,	/*Action completeness -Axis X */\
		ACTION_COMPLETE,	/*Action completeness -Axis Y   */\
		0,					/*Reserved 4 */\
		0,					/*Reserved 5 */\
		0,					/*Reserved 6 */\
		0					/*Timestamp  */\
}

#define MIRROR_VERSIONS \
{\
		0,			/* SW Version major number  */\
		0,			/* SW Version middle number */\
		8,			/* SW Version minor number   */\
		15,			/* SW Date year  */\
		8,			/* SW Date month */\
		25,			/* SW Date day */\
		"MIRROR SW ver:   0.0.8 FW ver: 3.0",			/* SW Version description -34 characters */\
		3,			/* FW Version major number  */\
		0,			/* FW Version minor number */\
		15,			/* FW Date year */\
		8,			/* FW Date month */\
		10,			/* FW Date day */\	
		0,			/* Reserved 1 */\
		0,			/* Reserved 2 */\
		0 			/* Reserved 3 */\
}


struct sIbitStatus
{
	uint32_t  Status;
	uint32_t  Percentage;
	uint32_t  Result;
};




struct sVersion
{
	uint8_t  SwVerMajor;
	uint8_t  SwVerMid;
	uint8_t  SwVerMinor;
	uint8_t  SwYear;
	uint8_t  SwMonth;
	uint8_t  SwDay;
	uint8_t	 SwVerDescriptor[34];
	uint8_t  FwVerMajor;
	uint8_t  FwVerMinor;
	uint8_t  FwYear;
	uint8_t  FwMonth;
	uint8_t  FwDay;
	uint8_t  Serial1;
	uint8_t  Serial2;
	uint8_t  Serial3;
};



struct sPacketHeader
{
	uint32_t Sync_Checksum;
	uint32_t MessageSize;
	uint32_t Opcode;
	uint32_t SourceID;
	uint32_t DestinationID;
	uint32_t Reserved1;
	uint32_t Reserved2;
	uint32_t MsgCounter;
	uint32_t Reserved3;
	uint32_t Reserved4;
};


struct sPedestalParams
{
	uint32_t State;
	uint32_t SubState;
	float 	 AxisYPosition;
	float	 AxisXPosition;
	float	 ControllerTemp;
	float	 Vprotected;
	float	 Vmotor;
	float	 V12volt;
	float	 V5volt;
	uint8_t  PDUStatus;
	uint8_t  NetStatus;
	uint8_t  Drive1Status;
	uint8_t  Drive2Status;
	float	 Drive1Temp;
	float	 Drive2Temp;
	uint8_t  Brake1Status;
	uint8_t  Brake2Status;
	uint8_t  Brake1State;
	uint8_t  Brake2State;
	uint32_t  Brake1Cnt;
	uint32_t  Brake2Cnt;
	float	 Current;
	uint8_t  AbsEncXStatus;
	uint8_t  FlashTest;
	uint8_t  RamTest;
	uint8_t  CpuReset;
	uint8_t  AllOkFlag;
	uint8_t  AbsEncYStatus;
	uint8_t  LimitSwState;
	uint8_t  SafetyState;
	uint32_t RS422MesRate;
	uint8_t  ActionXComplete;
	uint8_t  ActionYComplete;
	uint8_t  Reserved4;
	uint8_t  Reserved5;
	uint32_t Reserved6;
	uint32_t Timestamp;
	
};

unsigned char  SetDriveCmd(char *cmd,char *buf, char *ch);
unsigned char SetDriveRsp(char *cmd, char *buf, char ch);
unsigned char  SetStateAck(char * cmd,char *buf);
unsigned char BuildHeader(char *buf,struct sPacketHeader *header);
unsigned char  PedestalStatus(char *buf);
//unsigned char  LifetimeCounterResp(char *buf);
//unsigned char  LifetimeCounterSet(char *cmd,char *buf);
unsigned char  VersionResp(char *buf);
unsigned char  IBITResults(char *buf);
unsigned char  KeepAliveAck(char *buf);
unsigned char  IBITStatus(char *buf);
unsigned int PacketChecksum(char *buf, unsigned char len);
uint32_t GetState (void);
uint32_t GetSubState (void);
uint32_t SetState (uint32_t State);
uint32_t SetSubState (uint32_t SubState);
void  SetGraphCmd(char *cmd);
void Brake_1_Control(FunctionalState state);
void Brake_2_Control(FunctionalState state);
uint8_t GetAllFlags(void);

#ifndef RANISHOW
float CntToDeg(uint32_t enc, uint32_t offset);
#else
float CntToDeg(uint64_t enc, uint32_t offset);
#endif

uint8_t Travel_Pin_Control(void);
unsigned char  SetNetworkDetails(char *cmd,char *buf);
unsigned char  NetworkDetailsResp(char *buf, uint32_t Status);
unsigned char FactoryIpResp(char *buf, uint32_t addr);
unsigned char  GetHomePosition(char *buf, float xpos, float ypos);
void GoHomePosition(float xpos, float ypos);
float lpf_ema_float(float sample, float Emaverage, float alpha);



#endif
