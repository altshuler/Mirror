/*******************************************************************************
 * @data.c
 * @  data  
 *
 * @author Andrei Mamtsev
 *
 * @version 0.0.1
 * @date 09.02.2013
 *
**************************************************************************/
/**************************************************************************/
/* Standard Includes */
/**************************************************************************/
#include <string.h>

/**************************************************************************/
/* RTOS Includes */
/**************************************************************************/
#include "freertos.h"

/**************************************************************************/
/* Library Includes */
/**************************************************************************/

/**************************************************************************/
/* Driver includes */
/**************************************************************************/

/**************************************************************************/
/* Src includes */
/**************************************************************************/
#include "data.h"
#include "msg_type.h"
#include "hcmd.h"
#include "hostcomm.h"
#include "main.h"
#include "sysport.h"
#include "endianutils.h"

/**************************************************************************/
/*Declaration of global variables*/
/**************************************************************************/
struct sPacketHeader 	PacketHeader={SYNC,0,0,3,0,0,0,0,0,0};
struct sPedestalParams	SysParams=PEDESTAL_DEFAULT_PARAMS;
struct sVersion 		Versions=PEDESTAL_VERSIONS;
struct sIbitStatus		IbitStatus={2,0,1};

float LifetimeCnt=1.0;
uint16_t Counter=0;
uint8_t Travel_Pin_Pos;
uint8_t Travel_Pin_Cmd=PIN_CMD_NA;

extern xQueueHandle 	DriveIntQueue;
extern struct sDriverStatus DriveStatus;

extern uint16_t Timer_4_Period;
extern uint16_t Timer_9_Period;
extern uint16_t Channel1Pulse;
extern uint16_t Channel2Pulse;
extern uint16_t Channel3Pulse;
extern uint16_t Channel4Pulse;
extern float CurrentOffset;
extern portTickType TxDelayValue;
extern uSSI AbsEncoderXData;
extern uSSI AbsEncoderYData;
//extern uSSI AbsEncoderXCnt;
//extern uSSI AbsEncoderYCnt;
extern uint32_t AbsEncXOffset;
extern uint32_t AbsEncYOffset;
extern __IO uint16_t ADC1ConvertedValue[2];
extern __IO uint16_t ADC3ConvertedValue[5];
float PrevPosition=0;
uint8_t TravelPinTimeoutCnt=0;
uint8_t DirFlag=0;
float Enc=0;
float Emaverage=0;
extern portTickType RS_422_Rate;







unsigned char  SetDriveCmd(char *cmd, char *buf, char *ch)
{
unsigned char  len=0;
	

	*ch=*(cmd+20); 
	len=(*(cmd+4)-40);
		


	if (buf!=NULL)
	{
		memcpy(buf,cmd+40,len); 
	}
	
		return len;
}


unsigned char SetDriveRsp(char *cmd, char *buf, char ch)
{
	unsigned char  len=0;
	uint16_t Checksum;
	
	
	if (buf!=NULL)
	{
		len=11;		// driver resp length
		memcpy(buf+40, cmd, len);
		
		Checksum=PacketChecksum(buf+40,len);
		
		PacketHeader.Sync_Checksum =SYNC|(((uint32_t)Checksum)<<16);
		PacketHeader.MessageSize=(uint32_t)(len+sizeof(struct sPacketHeader));
		PacketHeader.Opcode=DIR_CMD_RSP;
		PacketHeader.DestinationID=1;
		PacketHeader.Reserved1=(uint32_t)ch;
		PacketHeader.MsgCounter++;
		len+=BuildHeader(buf,&PacketHeader);
	}
	return len;
}




unsigned char  SetStateAck(char *cmd,char *buf)
{
unsigned char  len=0;
uint32_t StateTranRes=0;
uint32_t ReqState;
uint32_t ReqSubState;
uint32_t ReadoutRate;
uint16_t Checksum;
MSG_HDR msg;	
MSG_HDR msg1;
uint32_t key;
float Pos1;
float Pos2;
float Vel;
uint8_t	MotionStatus;

	ReqState=*((uint32_t *)(cmd+40));
	ReqSubState=*((uint32_t *)(cmd+44));
	ReadoutRate=*((uint32_t *)(cmd+64));
	Pos1=*((float *)(cmd+48))/*DegToCnt(*((float *)(cmd+48)))*/;
	Pos2=*((float *)(cmd+52))/*DegToCnt(*((float *)(cmd+52)))*/;
	Vel=*((float *)(cmd+56));
	
#ifdef KUKU			
	if(Pos1>MAX_ANGLE)
		Pos1=MAX_ANGLE;
	else if (Pos1<MIN_ANGLE)
		Pos1=MIN_ANGLE;

	
	if(Pos2>MAX_ANGLE)
		Pos2=MAX_ANGLE;
	else if (Pos2<MIN_ANGLE)
		Pos2=MIN_ANGLE;

	if(Vel>MAX_VELOCITY)
		Vel=MAX_VELOCITY;
	else if (Vel<MIN_VELOCITY)
		Vel=MIN_VELOCITY;
	
	

	if(ReadoutRate<MIN_READOUT_RATE)
		ReadoutRate=MIN_READOUT_RATE;
	else if(ReadoutRate>MAX_READOUT_RATE)
		ReadoutRate=MAX_READOUT_RATE;
#endif	

	key=__disableInterrupts();
	DriveStatus.TargetPosXCmd=Pos1*RADIUS;
	DriveStatus.TargetPosYCmd=Pos2*RADIUS;
	DriveStatus.Position1Cmd=  DegToCnt(Pos1, AbsEncXOffset);
	DriveStatus.Position2Cmd=  DegToCnt(Pos2, AbsEncXOffset);
	DriveStatus.VelocityCmd= Vel;
	DriveStatus.VelUpdFlag= *((uint8_t *)(cmd+60));
	/*CurrentOffset*/ SysParams.RS422MesRate = ReadoutRate;
	__restoreInterrupts(key);
	
	RS_422_Rate=(portTickType)(1000/ReadoutRate);
		
	switch (ReqState)
		{
			case SYS_STATE_INIT:
				if(SysParams.State==SYS_STATE_INIT)
					StateTranRes=STATE_RES_OK;
				else	
					StateTranRes=ILLEGAL_TRANS;
				break;
				
			case SYS_STATE_STDBY:
				if((SysParams.State==SYS_STATE_INIT)||(SysParams.State==SYS_STATE_STDBY))
				{
					StateTranRes=STATE_RES_OK;
					key=__disableInterrupts();
					SysParams.State=ReqState;
					__restoreInterrupts(key);
				}
				else if(SysParams.State==SYS_STATE_OPERATE)
				{
					StateTranRes=STATE_RES_OK;
					Brake_1_Control(DISABLE);
					Brake_2_Control(DISABLE);
					
					key=__disableInterrupts();
					SysParams.State=ReqState;
					__restoreInterrupts(key);
					
					msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_1,MSG_TYPE_CMD);
					msg.data=DRV_STATE_MOTOR_OFF;
					msg.buf=NULL;
					xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);
				
				
					msg1.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_2,MSG_TYPE_CMD);
					msg1.data=DRV_STATE_MOTOR_OFF;
					msg1.buf=NULL;
					xQueueSend(DriveIntQueue,&msg1,portMAX_DELAY);
				}
				else
					StateTranRes=ILLEGAL_TRANS;
				break;

			case SYS_STATE_OPERATE:
				
				if(SysParams.State==SYS_STATE_OPERATE)
				{
					//if(SysParams.AllOkFlag==STATUS_OK)
					//{	
						StateTranRes=STATE_RES_OK;
						
						key=__disableInterrupts();
						SysParams.State=ReqState;
						SysParams.SubState=ReqSubState;
						MotionStatus=DriveStatus.MotionStatus;
						DriveStatus.MotionStatus=MOTION_ACTIVE;
						DriveStatus.TargetPosCmd=DriveStatus.Position1Cmd;
						__restoreInterrupts(key);
						
						if(SysParams.SubState==SYS_SUB_STATE_HOME)
							msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_1,MSG_TYPE_CMD);
						else
							msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_2,MSG_TYPE_CMD);
						
							msg.buf=NULL;

						/*	
						if(DriveStatus.VelUpdFlag==0)
						{
							msg.data=DRV_STATE_VELOCITY_SET;
							xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);
							vTaskDelay(1);
						}
						*/
					
						
							msg.data=DRV_STATE_MOVE_SET;
							xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);
					//}
					//else
					//	StateTranRes=ILLEGAL_TRANS;
				}
				else if(SysParams.State==SYS_STATE_STDBY)
				{				
					//if(SysParams.AllOkFlag==STATUS_OK)
					//{
						StateTranRes=STATE_RES_OK;
						//Brake_1_Control(ENABLE);
						//Brake_2_Control(ENABLE);
						
						key=__disableInterrupts();
						SysParams.State=ReqState;
						SysParams.SubState=ReqSubState;
						__restoreInterrupts(key);
					
						msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_1,MSG_TYPE_CMD);
						msg.data=DRV_STATE_MOTOR_ON;
						msg.buf=NULL;
						xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);
						


						msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_2,MSG_TYPE_CMD);
						msg.data=DRV_STATE_MOTOR_ON;
						msg.buf=NULL;
						xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);


					//}
					//else
					//	StateTranRes=ILLEGAL_TRANS;
				}
				else
					StateTranRes=ILLEGAL_TRANS;
				break;

			case SYS_STATE_MAINT:
				if(SysParams.State==SYS_STATE_STDBY)
				{
					StateTranRes=STATE_RES_OK;
					key=__disableInterrupts();
					SysParams.State=ReqState;
					__restoreInterrupts(key);
				}
				else
					StateTranRes=ILLEGAL_TRANS;
				break;
				
			case SYS_STATE_RESET:
				if((SysParams.State==SYS_STATE_STDBY)||(SysParams.State==SYS_STATE_MAINT)||(SysParams.State==SYS_STATE_OPERATE))
				{
					StateTranRes=STATE_RES_OK;
					
					msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_1,MSG_TYPE_CMD);
					msg.data=DRV_STATE_MOTOR_OFF;
					msg.buf=NULL;
					xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);

					vTaskDelay(1);
				
					msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_2,MSG_TYPE_CMD);	
					xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);
						
					key=__disableInterrupts();
					SysParams.State=ReqState;
					__restoreInterrupts(key);
					vTaskDelay(120);
					
					sendPacketToHost(NULL, MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD,MSG_TYPE_CMD), portMAX_DELAY);
					vTaskDelay(120);
					
					NVIC_SystemReset();
					while(1);
				}
				else
					StateTranRes=ILLEGAL_TRANS;
				break;

			case SYS_STATE_SHTDWN:
				if((SysParams.State==SYS_STATE_STDBY)||(SysParams.State==SYS_STATE_MAINT)||(SysParams.State==SYS_STATE_OPERATE))
				{
					StateTranRes=STATE_RES_OK;
					key=__disableInterrupts();
					SysParams.State=ReqState;
					__restoreInterrupts(key);
					
					msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_1,MSG_TYPE_CMD);
					msg.data=DRV_STATE_MOTOR_OFF;
					msg.buf=NULL;
					xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);
				
					vTaskDelay(1);
					
					msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_2,MSG_TYPE_CMD);	
					xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);
					
					vTaskDelay(200);
					Brake_1_Control(DISABLE);
					Brake_2_Control(DISABLE);
					
					sendPacketToHost(NULL, MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD,MSG_TYPE_CMD), portMAX_DELAY);
				}
				else
					StateTranRes=ILLEGAL_TRANS;
				break;

			default:
					StateTranRes=UNKNOWN_CODE;
				break;
		}


	if(StateTranRes==STATE_RES_OK)
	{
		key=__disableInterrupts();
		SysParams.State=ReqState;
		SysParams.SubState=ReqSubState;
		
		#ifdef KUKU		
		if((ReqState==SYS_STATE_OPERATE)&&(ReqSubState==SYS_SUB_STATE_SCAN))
			SysParams.AngVelocity=150;
		else
			SysParams.AngVelocity=0;
		#endif
		
		__restoreInterrupts(key);
	}

	if (buf!=NULL)
	{
		memcpy(buf+40,&StateTranRes,sizeof(uint32_t)); 
		memcpy(buf+44,&ReqState,sizeof(uint32_t));
		memcpy(buf+48,&SysParams.State,sizeof(uint32_t));
		memcpy(buf+52,&ReqSubState,sizeof(uint32_t));
		memcpy(buf+56,&SysParams.SubState,sizeof(uint32_t));
		
		len+=20;
		
		Checksum=PacketChecksum(buf+40,len);
		
		PacketHeader.Sync_Checksum =SYNC|(((uint32_t)Checksum)<<16);
		PacketHeader.MessageSize=60;
		PacketHeader.Opcode=STATE_RSP;
		PacketHeader.DestinationID=*((uint32_t *)(cmd+16));
		PacketHeader.MsgCounter++;
		len+=BuildHeader(buf,&PacketHeader);
	}
	
		return len;
}

unsigned char  PedestalStatus(char *buf)
{
	unsigned char  len=0;
	uint16_t Checksum;
	uint32_t key;
	float AbsEncoderXDeg;
	float AbsEncoderYDeg;
	int32_t AbsCounter;
	uint8_t AllOkFlag;
	float VelSample;
	
	
	Counter++;
	if(Counter>36000)	//Instead RTC -Temporary solution
	{
		LifetimeCnt+=1.0;
		Counter=0;

		if(LifetimeCnt>10000.0)
			LifetimeCnt=0.0;
	}



	
	#ifndef RANISHOW
		if(AbsEncoderXData.ssiData.Error==1)
		{
			key=__disableInterrupts();
			SysParams.AbsEncStatus=STATUS_FAIL;
			__restoreInterrupts(key);
		}
		else
		{
			key=__disableInterrupts();
			AbsEncoderXDeg=CntToDeg(AbsEncoderXData.raw32Data, AbsEncXOffset);
			__restoreInterrupts(key);
		}

	#else
		if(AbsEncoderXData.ssiData.Error==0)
		{
			key=__disableInterrupts();
			SysParams.AbsEncStatus=STATUS_FAIL;
			__restoreInterrupts(key);
		}
		else
		{
			key=__disableInterrupts();
			AbsEncoderXDeg=CntToDeg(AbsEncoderXData.raw32Data, AbsEncXOffset);
			__restoreInterrupts(key);
			if(AbsEncoderXDeg>180.0)
				AbsEncoderXDeg-=360.0;
		}

		if(AbsEncoderYData.ssiData.Error==0)
		{
			key=__disableInterrupts();
			SysParams.AbsEncStatus=STATUS_FAIL;
			__restoreInterrupts(key);
		}
		else
		{
			key=__disableInterrupts();
			AbsEncoderYDeg=CntToDeg(AbsEncoderYData.raw32Data, AbsEncYOffset);
			__restoreInterrupts(key);
			if(AbsEncoderYDeg>180.0)
				AbsEncoderYDeg-=360.0;
		}	
	
	#endif
	
		#ifdef KUKU

		if(DirFlag)
		{
			Enc+=0.3;
			if(Enc>90.0)
				DirFlag=0;
		}
		else
		{
			Enc-=0.3;
			if(Enc<-90.0)
				DirFlag=1;
		}
		#endif
			
			AllOkFlag=GetAllFlags();
			key=__disableInterrupts();
			SysParams.AllOkFlag=AllOkFlag;
			SysParams.Azimut=/*Enc*/AbsEncoderYDeg*PI/0.00018; // Position X in uRadians
			VelSample=(SysParams.Azimut-PrevPosition)*1000.0/((float)TxDelayValue+(float)RX_DELAY);
			PrevPosition=SysParams.Azimut;
		    SysParams.Timestamp+=(uint32_t)TxDelayValue+(uint32_t)RX_DELAY;
			__restoreInterrupts(key);

			Emaverage  = lpf_ema_float(VelSample, Emaverage, 0.5);

			key=__disableInterrupts();
			SysParams.AngVelocity = AbsEncoderXDeg*PI/0.00018/*Emaverage*/;// Position Y in uRadians
			__restoreInterrupts(key);


	
	
	if (buf!=NULL)
	{
		len=sizeof(struct sPedestalParams);
		memcpy(buf+40, &SysParams, len);
		
		Checksum=PacketChecksum(buf+40,len);
		
		PacketHeader.Sync_Checksum =SYNC|(((uint32_t)Checksum)<<16);
		PacketHeader.MessageSize=(uint32_t)(len+sizeof(struct sPacketHeader));
		PacketHeader.Opcode=STATUS_RSP;
		PacketHeader.DestinationID=1;
		PacketHeader.MsgCounter++;
		len+=BuildHeader(buf,&PacketHeader);
	}
	return len;
}


unsigned char  LifetimeCounterResp(char *buf)
{
	unsigned char  len=0;
	uint16_t Checksum;
	

	if (buf!=NULL)
	{
		memcpy(buf+40,&LifetimeCnt,sizeof(float)); 
		
		len=sizeof(float);
		
		Checksum=PacketChecksum(buf+40,len);

		PacketHeader.Sync_Checksum =SYNC|(((uint32_t)Checksum)<<16);
		PacketHeader.MessageSize=(uint32_t)(len+sizeof(struct sPacketHeader));
		PacketHeader.Opcode=CNT_RSP;
		PacketHeader.DestinationID=1;
		PacketHeader.MsgCounter++;
		
		len+=BuildHeader(buf,&PacketHeader);
	}
		return len;
}



unsigned char  LifetimeCounterSet(char *cmd,char *buf)
{
	unsigned char  len=0;
	uint32_t DestID;
	

	DestID=*((uint32_t *)(cmd+40));
	

	if (DestID==PacketHeader.SourceID)
	{
		LifetimeCnt=*((float *)(cmd+44));
		len=LifetimeCounterResp(buf);
	}
	
		return len;
}


unsigned char  SetNetworkDetails(char *cmd,char *buf)
{
	struct sAppIpSettings IpSettings;
	uint32_t Status;
	unsigned char  len=0;
	
	
	IpSettings=*((struct sAppIpSettings *)(cmd+40));

	if(ERROR == setApplicationIPSettings(&IpSettings))
			Status=STATUS_FAIL;
	else
			Status=STATUS_OK;

	vTaskDelay(5);

	len=NetworkDetailsResp(buf,Status);

	return len;
}



unsigned char  NetworkDetailsResp(char *buf, uint32_t Status)
{
	unsigned char  len=0;
	uint16_t Checksum;
	//uint32_t Status;
	struct sAppIpSettings IpSettings;
	char *temp;
	//uint8_t	*TmpBuffer = NULL;
	
		
	if (buf!=NULL)
	{
		
		if(Status==STATUS_FAIL)
		{
			temp=buf+40;
			len+=sizeof(uint32_t);
			memcpy(temp, &Status, sizeof(uint32_t));
	
			temp+=sizeof(uint32_t);
			len+=sizeof(struct sAppIpSettings);
			memset(temp, 0, sizeof(struct sAppIpSettings));
		}
		else
		{
			
			if(ERROR == getApplicationIPSettings(&IpSettings))
				Status=STATUS_FAIL;
			else
				Status=STATUS_OK;
			
			temp=buf+40;
			len+=sizeof(uint32_t);
			memcpy(temp, &Status, sizeof(uint32_t));

			temp+=sizeof(uint32_t);
			len+=sizeof(struct sAppIpSettings);

			memcpy(temp,  &IpSettings, sizeof(struct sAppIpSettings));
		
		}
		Checksum=PacketChecksum(buf+40,len);

		PacketHeader.Sync_Checksum =SYNC|(((uint32_t)Checksum)<<16);
		PacketHeader.MessageSize=(uint32_t)(len+sizeof(struct sPacketHeader));
		PacketHeader.Opcode=NET_RSP;
		PacketHeader.DestinationID=1;
		PacketHeader.MsgCounter++;
		
		len+=BuildHeader(buf,&PacketHeader);
		
	}
		return len;
}


unsigned char FactoryIpResp(char *buf, uint32_t addr)
{
	unsigned char  len=0;
	uint16_t Checksum;
	char *temp;
	//uint8_t *TmpBuffer = NULL;
	uint32_t IpAddr;	
			
	if (buf!=NULL)
	{
		IpAddr=longBE2LE(addr);	
		temp=buf+40;
		len+=sizeof(uint32_t);
		memcpy(temp, &IpAddr, sizeof(uint32_t));
		
		Checksum=PacketChecksum(buf+40,len);

		PacketHeader.Sync_Checksum =SYNC|(((uint32_t)Checksum)<<16);
		PacketHeader.MessageSize=(uint32_t)(len+sizeof(struct sPacketHeader));
		PacketHeader.Opcode=BOOT_REQ;
		PacketHeader.DestinationID=1;
		PacketHeader.MsgCounter++;
		
		len+=BuildHeader(buf,&PacketHeader);
		
	}
			return len;


}



unsigned char  VersionResp(char *buf)
{
	unsigned char  len=0;
	uint16_t Checksum;
	

	if (buf!=NULL)
	{
		len=sizeof(struct sVersion);
		memcpy(buf+40, &Versions, len);
		
		Checksum=PacketChecksum(buf+40,len);
		
		PacketHeader.Sync_Checksum =SYNC|(((uint32_t)Checksum)<<16);
		PacketHeader.MessageSize=(uint32_t)(len+sizeof(struct sPacketHeader));
		PacketHeader.Opcode=VER_RSP;
		PacketHeader.DestinationID=1;
		PacketHeader.MsgCounter++;
		len+=BuildHeader(buf,&PacketHeader);
	}
	return len;
}



unsigned char  IBITResults(char *buf)
{
	unsigned char  len=0;
	uint16_t Checksum;
	char *temp;

	
	
	if (buf!=NULL)
	{
		temp=buf+40;
		len+=sizeof(float);
		memcpy(temp, &SysParams.Vprotected, sizeof(float));
		
		temp+=sizeof(float);
		len+=sizeof(float);
		memcpy(temp, &SysParams.Vmotor, sizeof(float));

		temp+=sizeof(float);
		len+=sizeof(float);
		memcpy(temp, &SysParams.V12volt, sizeof(float));

		temp+=sizeof(float);
		len+=sizeof(float);
		memcpy(temp, &SysParams.V5volt, sizeof(float));

		temp+=sizeof(float);
		len+=sizeof(float);
		*temp=SysParams.PDUStatus;

		temp+=sizeof(uint8_t);
		len+=sizeof(uint8_t);
		*temp=1;							//Motors Status

		temp+=sizeof(uint8_t);
		len+=sizeof(uint8_t);
		*temp=1;							//Motor1  ON

		temp+=sizeof(uint8_t);
		len+=sizeof(uint8_t);
		*temp=1;							//Motor2  ON
		
		temp+=sizeof(uint8_t);
		len+=sizeof(uint8_t);
		*temp=0;							//Motor1  Failure
		
		temp+=sizeof(uint8_t);
		len+=sizeof(uint8_t);
		*temp=0;							//Motor2  Failure

		temp+=sizeof(uint8_t);
		len+=sizeof(uint8_t);
		*temp=SysParams.Drive1Status|SysParams.Drive2Status;	//Drivers Status

		temp+=sizeof(uint8_t);
		len+=sizeof(uint8_t);
		memcpy(temp, &SysParams.Drive1Status , sizeof(uint8_t));	//Driver 1 Status

		temp+=sizeof(uint8_t);
		len+=sizeof(uint8_t);
		memcpy(temp, &SysParams.Drive2Status , sizeof(uint8_t));	//Driver 2 Status

		temp+=sizeof(uint8_t);
		len+=sizeof(uint8_t);
		*temp=3;								//Driver 1 Mode

		temp+=sizeof(uint8_t);
		len+=sizeof(uint8_t);
		*temp=3;							//Driver 2 Mode

		temp+=sizeof(uint8_t);
		len+=sizeof(uint8_t);
		*temp=0;							//Driver 1  CPU status

		temp+=sizeof(uint8_t);
		len+=sizeof(uint8_t);
		*temp=0;							//Driver 2  CPU status

		temp+=sizeof(uint8_t);
		len+=sizeof(uint8_t);
		*temp=2;							//Motion sensors status

		temp+=sizeof(uint8_t);
		len+=sizeof(uint8_t);
		*temp=2;							//Drive 1  Encoder

		temp+=sizeof(uint8_t);
		len+=sizeof(uint8_t);
		*temp=1;							//Drive 2  Encoder

		temp+=sizeof(uint8_t);
		len+=sizeof(uint8_t);
		*temp=2;							//Absolute Encoder Status

		temp+=sizeof(uint8_t);
		len+=sizeof(uint8_t);
		*temp=0;							//Acceleration and Speed

		temp+=sizeof(uint8_t);
		len+=sizeof(uint8_t);
		*temp=SysParams.Brake1Status|SysParams.Brake2Status;		//Brakes  Test Status	

		temp+=sizeof(uint8_t);
		len+=sizeof(uint8_t);
		memcpy(temp, &SysParams.Brake1Status, sizeof(uint8_t));	//Brake 1  Status
		
		temp+=sizeof(uint8_t);
		len+=sizeof(uint8_t);
		*temp=SysParams.Brake2Status;		//Brake 2  Status

		temp+=sizeof(uint8_t);
		len+=sizeof(uint8_t);
		*temp=0;							//ALL OK FLAG

		temp+=sizeof(uint8_t);
		len+=sizeof(uint8_t);
		*temp=1;							//res 1

		temp+=sizeof(uint8_t);
		len+=sizeof(uint8_t);
		*temp=1;							//res 2

		temp+=sizeof(uint8_t);
		len+=sizeof(uint8_t);
		*temp=1;							//res 3

		temp+=sizeof(uint8_t);
		len+=sizeof(uint8_t);
		*temp=1;							//res 4

		temp+=sizeof(uint8_t);
		len+=sizeof(uint8_t);
		*temp=1;							//res 5

		temp+=sizeof(uint8_t);
		len+=sizeof(uint8_t);
		*temp=1;							//res 6
		
		Checksum=PacketChecksum(buf+40,len);
		
		PacketHeader.Sync_Checksum =SYNC|(((uint32_t)Checksum)<<16);
		PacketHeader.MessageSize=(uint32_t)(len+sizeof(struct sPacketHeader));
		PacketHeader.Opcode=IBIT_RES;
		PacketHeader.DestinationID=1;
		PacketHeader.MsgCounter++;
		len+=BuildHeader(buf,&PacketHeader);
	}
	return len;
}




unsigned char  KeepAliveAck(char *buf)
{
	unsigned char  len=0;
	uint16_t Checksum;
	

	if (buf!=NULL)
	{
		
		Checksum=0;
		
		PacketHeader.Sync_Checksum =SYNC|(((uint32_t)Checksum)<<16);
		PacketHeader.MessageSize=(uint32_t)(sizeof(struct sPacketHeader));
		PacketHeader.Opcode=KEEP_ALIVE_RSP;
		PacketHeader.DestinationID=1;
		PacketHeader.MsgCounter++;
		len+=BuildHeader(buf,&PacketHeader);
	}
	return len;
}




unsigned char  IBITStatus(char *buf)
{
	unsigned char  len=0;
	uint16_t Checksum;
	
	
	if (buf!=NULL)
	{

		if(IbitStatus.Status==IBIT_FINISHED)
		{
			IbitStatus.Percentage++;
			IbitStatus.Status=IBIT_START;
		}
		
		else if(IbitStatus.Status==IBIT_START)
		{
			IbitStatus.Percentage++;
			IbitStatus.Status=IBIT_ONGOING;
		}
		else if((IbitStatus.Status==IBIT_ONGOING)&&(IbitStatus.Percentage<100))
		{
			IbitStatus.Percentage++;
		}
		else
		{
			IbitStatus.Status=IBIT_FINISHED;
			IbitStatus.Percentage=0;
		}

		
			
		len=sizeof(struct sIbitStatus);
		memcpy(buf+40, &IbitStatus, len);
		
		Checksum=PacketChecksum(buf+40,len);
		
		PacketHeader.Sync_Checksum =SYNC|(((uint32_t)Checksum)<<16);
		PacketHeader.MessageSize=(uint32_t)(len+sizeof(struct sPacketHeader));
		PacketHeader.Opcode=IBIT_STATUS;
		PacketHeader.DestinationID=1;
		PacketHeader.MsgCounter++;
		len+=BuildHeader(buf,&PacketHeader);
	}
	return len;
}




unsigned char BuildHeader(char *buf,struct sPacketHeader *header)
{
unsigned char len=0;

	if (buf!=NULL)
	{
		len=sizeof(struct sPacketHeader);
		memcpy(buf, header, len);
	}
	
	return len;
}


unsigned int PacketChecksum(char *buf, unsigned char len)
{
unsigned int chksum=0;
uint8_t i;

	for(i=0;i<len;i++)
		chksum+=(unsigned int)(*(buf+i));

	return chksum;
	
}


uint32_t GetState (void)
{
uint32_t State;
uint32_t key;

	key=__disableInterrupts();
	State=SysParams.State;
	__restoreInterrupts(key);
	
	return  State;
}


uint32_t GetSubState (void)
{
uint32_t SubState;
uint32_t key;

	key=__disableInterrupts();
	SubState=SysParams.SubState;
	__restoreInterrupts(key);
	
	return  SubState;
}



uint32_t SetState (uint32_t State)
{
uint32_t cr_state=0;
uint32_t key;

	key=__disableInterrupts();
	SysParams.State=State;
	cr_state=SysParams.State;
	__restoreInterrupts(key);

	
	return cr_state;
}



uint32_t SetSubState (uint32_t SubState)
{
uint32_t cr_substate=0;
uint32_t key;

	key=__disableInterrupts();
	SysParams.SubState=SubState;
	cr_substate=SysParams.SubState;
	__restoreInterrupts(key);
	
	return cr_substate;
}


void  SetGraphCmd(char *cmd)
{
	uint32_t EthDelayFlag;	

	EthDelayFlag=*((uint32_t *)(cmd+40));
		


	if (EthDelayFlag)
		TxDelayValue=TX_DELAY_ATP;
	else
		TxDelayValue=TX_DELAY;
}


void Brake_1_Control(FunctionalState state)
{
uint32_t key;

	if(state==ENABLE)	//Release Brake 1
	{
		  TIM9->CCR1 = Timer_9_Period;
		  vTaskDelay(100);
  		  TIM9->CCR1 = Channel1Pulse;
		  key=__disableInterrupts();
 		  SysParams.Brake1State=0x0; 	//Brake Enabled
 		  __restoreInterrupts(key);	   
	}
	else				//Close Brake 1
	{
  		  TIM9->CCR1 = 0;
		  key=__disableInterrupts();
 		  SysParams.Brake1State=0x1; 	//Brake Disabled
 		  __restoreInterrupts(key);	
	}	  
	
		 
}

void Brake_2_Control(FunctionalState state)
{
uint32_t key;

	if(state==ENABLE)	//Release Brake 2
	{
		  TIM9->CCR2 = Timer_9_Period;
		  vTaskDelay(100);
  		  TIM9->CCR2 = Channel2Pulse;
		  key=__disableInterrupts();
 		  SysParams.Brake2State=0x0; 	//Brake Enabled
 		  __restoreInterrupts(key);	   
	}
	else				//Close Brake 2
	{
  		  TIM9->CCR2 = 0;
		  key=__disableInterrupts();
 		  SysParams.Brake2State=0x1; 	//Brake Disabled
 		  __restoreInterrupts(key);	
	}	  
}



uint8_t GetAllFlags(void)
{
uint8_t	Status=STATUS_OK;
uint8_t	Pdu_Status=STATUS_OK;
uint32_t key;
	
	if((15.7>SysParams.Vmotor)||(SysParams.Vmotor>36.3))
		Pdu_Status=STATUS_FAIL;
	else if((15.7>SysParams.Vprotected)||(SysParams.Vprotected>36.3))
		Pdu_Status=STATUS_FAIL;
	else if((11.5>SysParams.V12volt)||(SysParams.V12volt>12.5))
		Pdu_Status=STATUS_FAIL;
	else if((4.7>SysParams.V5volt)||(SysParams.V5volt>5.3))
		Pdu_Status=STATUS_FAIL;

	key=__disableInterrupts();
	SysParams.PDUStatus=Pdu_Status;	
	__restoreInterrupts(key); 

	if(SysParams.PDUStatus==STATUS_FAIL)
		Status=STATUS_FAIL;
	if(SysParams.NetStatus==STATUS_FAIL)
		Status=STATUS_FAIL;
	if(SysParams.Drive1Status==STATUS_FAIL)
		Status=STATUS_FAIL;
	if(SysParams.Drive2Status==STATUS_FAIL)
		Status=STATUS_FAIL;
	//if(SysParams.Drive1Temp==STATUS_FAIL)
	//	Status=STATUS_FAIL;
	//if(SysParams.Drive2Temp==STATUS_FAIL)
	//	Status=STATUS_FAIL;
	if(SysParams.Brake1Status==STATUS_FAIL)
		Status=STATUS_FAIL;
	if(SysParams.Brake2Status==STATUS_FAIL)
		Status=STATUS_FAIL;
	//if(SysParams.Current==STATUS_FAIL)
	//	Status=STATUS_FAIL;
	if(SysParams.AbsEncStatus==STATUS_FAIL)
		Status=STATUS_FAIL;
	if(SysParams.FlashTest==STATUS_FAIL)
		Status=STATUS_FAIL;
	if(SysParams.RamTest==STATUS_FAIL)
		Status=STATUS_FAIL;
	if(SysParams.SafetyState!=STATUS_OK)
		Status=STATUS_FAIL;
	if(SysParams.LimitSwState!=0x3f)
		Status=STATUS_FAIL;
	
return Status;

}

#ifndef RANISHOW

float CntToDeg(uint32_t enc, uint32_t offset)
{
int32_t AbsEnc;
float   Deg;

	AbsEnc=enc>>3;
	AbsEnc-=(int32_t)offset;
	
	if(AbsEnc>ABS_ENC_MAX_VAL)
		AbsEnc=AbsEnc-ABS_ENC_MAX_VAL+1;
	else if(AbsEnc<0)
		AbsEnc=AbsEnc+ABS_ENC_MAX_VAL+1;
	
	Deg=((float)AbsEnc)/1024.0-90.0;

return Deg;
}

#else

float CntToDeg(uint64_t enc, uint32_t offset)
{
int32_t AbsEnc;
float   Deg;

	AbsEnc=(uint32_t)((enc>>25)&0x3FFFFFF);
	AbsEnc-=(int32_t)offset;
	
	if(AbsEnc>ABS_ENC_MAX_VAL)
		AbsEnc=AbsEnc-ABS_ENC_MAX_VAL+1;
	else if(AbsEnc<0)
		AbsEnc=AbsEnc+ABS_ENC_MAX_VAL+1;

	Deg=(float)((((double)(AbsEnc))/67108863.0)*360.0);
	//Deg-=90.0;

return Deg;
}

#endif
	



uint8_t Travel_Pin_Control(void)
{
	uint8_t error=0;
	uint32_t key;
	MSG_HDR msg;
	MSG_HDR msg1;
	
	TravelPinTimeoutCnt++;

	if(TravelPinTimeoutCnt>100) //20 sec timeout
		error=1;


	if(ADC3ConvertedValue[4]>MAX_PIN_CURRENT)//OverCurrent
		error=1;

	
	if(error)
	{
		Travel_Pin_Cmd=PIN_CMD_NA;
		TravelPinTimeoutCnt=0;
		GPIO_ResetBits(PIN_ON_GPIO_PORT, PIN_ON_PIN); //Disable Travel Pin Bridge
		TIM4->CCR3 = 0x0;			
		TIM4->CCR4 = 0x0;
		return error;
	}
	else if(Travel_Pin_Cmd==PIN_CMD_CLOSE)	
	{
		if(ADC1ConvertedValue[1]>PIN_CLOSED_POSITION)
		{
			GPIO_ResetBits(PIN_ON_GPIO_PORT, PIN_ON_PIN); //Disable Travel Pin Bridge
			TIM4->CCR3 = 0x0;			
			TIM4->CCR4 = 0x0;
			Travel_Pin_Cmd=PIN_CMD_NA;
			Travel_Pin_Pos=PIN_POS_CLOSE;
			TravelPinTimeoutCnt=0;

			Brake_1_Control(DISABLE);
			Brake_2_Control(DISABLE);
					
			key=__disableInterrupts();
			SysParams.State=SYS_STATE_STDBY;
			__restoreInterrupts(key);
					
			msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_1,MSG_TYPE_CMD);
			msg.data=DRV_STATE_MOTOR_OFF;
			msg.buf=NULL;
			xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);
		
		
			msg1.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_2,MSG_TYPE_CMD);
			msg1.data=DRV_STATE_MOTOR_OFF;
			msg1.buf=NULL;
			xQueueSend(DriveIntQueue,&msg1,portMAX_DELAY);
		}
			
	}
	else if(Travel_Pin_Cmd==PIN_CMD_OPEN)	
	{
		if(ADC1ConvertedValue[1]<PIN_OPENED_POSITION)
		{
			GPIO_ResetBits(PIN_ON_GPIO_PORT, PIN_ON_PIN); //Disable Travel Pin Bridge
			TIM4->CCR3 = 0x0;			
			TIM4->CCR4 = 0x0;
			Travel_Pin_Cmd=PIN_CMD_NA;
			Travel_Pin_Pos=PIN_POS_OPEN;
			TravelPinTimeoutCnt=0;
		}  		
	}	
return error;
}



void  SetTravelPinCmd(char *cmd)
{
	uint32_t TravelPinCmd;	
	uint32_t key;
	float 	Azimuth;
	uint8_t MotionStatus;
	
	TravelPinCmd=*((uint32_t *)(cmd+40));
		
	key=__disableInterrupts();
 	Azimuth=SysParams.Azimut; 
	MotionStatus=DriveStatus.MotionStatus;
 	__restoreInterrupts(key);

	if (Azimuth>180.0)
		Azimuth=Azimuth-360.0;
	
	if(MotionStatus == MOTION_NOT_ACTIVE)
	{
		if(((Azimuth>89.5)&&(Azimuth<90.5))||((Azimuth<-89.5)&&(Azimuth>-90.5))||((Azimuth>-0.5)&&(Azimuth<0.5)))
		{
			if (TravelPinCmd)
			{
				if(!(ADC1ConvertedValue[1]<PIN_OPENED_POSITION))
				{
					Travel_Pin_Cmd=PIN_CMD_OPEN;
					GPIO_SetBits(PIN_ON_GPIO_PORT, PIN_ON_PIN); //Enable Travel Pin Bridge
					TIM4->CCR3 = 0x0;			//Open Travel Pin
					TIM4->CCR4 = Channel4Pulse;
				}
			}
			else
			{
				if(!(ADC1ConvertedValue[1]>PIN_CLOSED_POSITION))
				{
					Travel_Pin_Cmd=PIN_CMD_CLOSE;
					GPIO_SetBits(PIN_ON_GPIO_PORT, PIN_ON_PIN); //Enable Travel Pin Bridge
					TIM4->CCR3 = Channel3Pulse;//Close Travel Pin
					TIM4->CCR4 = 0x0;
				}
				
				
				
			}
		}
	}
}



unsigned char  GetTravelPinReostat(char *buf)
{
	unsigned char  len=0;
	uint16_t Checksum;


	if (buf!=NULL)
	{
		memcpy(buf+40,&ADC1ConvertedValue[1],sizeof(uint16_t)); 
		
		len=sizeof(uint16_t);
		
		Checksum=PacketChecksum(buf+40,len);

		PacketHeader.Sync_Checksum =SYNC|(((uint32_t)Checksum)<<16);
		PacketHeader.MessageSize=(uint32_t)(len+sizeof(struct sPacketHeader));
		PacketHeader.Opcode=REOSTAT_RSP;
		PacketHeader.DestinationID=1;
		PacketHeader.MsgCounter++;
		
		len+=BuildHeader(buf,&PacketHeader);
	}
		return len;
}



#ifdef KUKU
inline _iq dsp_ema_iq(_iq sample, _iq Emaverage, _iq alpha)
{
    _iq Ema;
    Ema=_IQ29mpyIQX(sample,GLOBAL_Q,alpha,GLOBAL_Q) + _IQ29mpyIQX(Emaverage,GLOBAL_Q,_IQ(1.0)-alpha,GLOBAL_Q);

	return _IQ29toIQ(Ema);
}
#else
float lpf_ema_float(float sample, float Emaverage, float alpha)
{
    float Ema;
	
    Ema=sample*alpha + Emaverage -  Emaverage*alpha; 

	return Ema;
}


#endif


