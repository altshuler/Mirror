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
struct sPedestalParams	SysParams=MIRROR_DEFAULT_PARAMS;
struct sVersion 		Versions=MIRROR_VERSIONS;
struct sIbitStatus		IbitStatus={2,0,1};

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
float PrevXVel=0;
float PrevYVel=0;
float PrevPos1=0;
float PrevPos2=0;







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
	float XVel;
	float YVel;
	float AbsEncoderXDeg;
	float AbsEncoderYDeg;
	float TargXPos;
	float TargYPos;



	ReqState=*((uint32_t *)(cmd+40));
	ReqSubState=*((uint32_t *)(cmd+44));
	ReadoutRate=*((uint32_t *)(cmd+64));
	Pos1=*((float *)(cmd+48))/*DegToCnt(*((float *)(cmd+48)))*/;
	Pos2=*((float *)(cmd+52))/*DegToCnt(*((float *)(cmd+52)))*/;
	XVel=*((float *)(cmd+56));
	YVel=*((float *)(cmd+60));
	
		
	if(Pos1>MAX_ANGLE)
		Pos1=MAX_ANGLE;
	else if (Pos1<MIN_ANGLE)
		Pos1=MIN_ANGLE;

	
	if(Pos2>MAX_ANGLE)
		Pos2=MAX_ANGLE;
	else if (Pos2<MIN_ANGLE)
		Pos2=MIN_ANGLE;
	

	if(XVel>MAX_VELOCITY)
		XVel=MAX_VELOCITY;
	else if (XVel<MIN_VELOCITY)
		XVel=MIN_VELOCITY;
	
	if(YVel>MAX_VELOCITY)
		YVel=MAX_VELOCITY;
	else if (YVel<MIN_VELOCITY)
		YVel=MIN_VELOCITY;

	if(ReadoutRate<MIN_READOUT_RATE)
		ReadoutRate=MIN_READOUT_RATE;
	else if(ReadoutRate>MAX_READOUT_RATE)
		ReadoutRate=MAX_READOUT_RATE;


	if(AbsEncoderXData.ssiData.Error==0)
	{
		//key=__disableInterrupts();
		//SysParams.AbsEncXStatus=STATUS_FAIL;
		//__restoreInterrupts(key);
	}
	else
	{
		key=__disableInterrupts();
		AbsEncoderXDeg=CntToDeg(AbsEncoderXData.raw32Data, AbsEncXOffset);
		__restoreInterrupts(key);
		if(AbsEncoderXDeg>180.0)
			AbsEncoderXDeg-=360.0;
	}


	 //EncDegreesData(AbsEncoderXData, &AbsEncoderXDeg,  AbsEncXOffset);



	if(AbsEncoderYData.ssiData.Error==0)
	{
		//key=__disableInterrupts();
		//SysParams.AbsEncYStatus=STATUS_FAIL;
		//__restoreInterrupts(key);
	}
	else
	{
		key=__disableInterrupts();
		AbsEncoderYDeg=CntToDeg(AbsEncoderYData.raw32Data, AbsEncYOffset);
		__restoreInterrupts(key);
		if(AbsEncoderYDeg>180.0)
			AbsEncoderYDeg-=360.0;
	}	

	AbsEncoderXDeg=AbsEncoderXDeg*PI/0.00018;	//X position in uRadians 	 TODO:Remove PI defines
	AbsEncoderYDeg=AbsEncoderYDeg*PI/0.00018;	//Y position in uRadians

	TargXPos=(Pos1-AbsEncoderXDeg)*XRADIUS;
	TargYPos=(Pos2-AbsEncoderYDeg)*YRADIUS;

	if(TargXPos<0.0)
		TargXPos-=BCKL_X_OFFSET;
	if(TargYPos>0.0)
		TargYPos-=BCKL_Y_OFFSET;

		
	key=__disableInterrupts();
	DriveStatus.HostPosXCmd=Pos1;
	DriveStatus.HostPosYCmd=Pos2;	
	DriveStatus.TargetPosXCmd=TargXPos;
	DriveStatus.TargetPosYCmd=TargYPos;
	DriveStatus.VelocityXCmd= XVel*XRADIUS;
	DriveStatus.VelocityYCmd= YVel*YRADIUS;
	SysParams.RS422MesRate = ReadoutRate;
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
					
					key=__disableInterrupts();
					SysParams.State=ReqState;
					DriveStatus.MotionXStatus=MOTION_NOT_ACTIVE;
					DriveStatus.MotionYStatus=MOTION_NOT_ACTIVE;	
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
						DriveStatus.TargetPosCmd=DriveStatus.Position1Cmd;
						__restoreInterrupts(key);
						
						if(SysParams.SubState==SYS_STPR_1_SELECT)
						{
							if(((SysParams.LimitSwState&0x1)&&(Pos1>AbsEncoderXDeg))||((SysParams.LimitSwState&0x2)&&(Pos1<AbsEncoderXDeg))||SysParams.LimitSwState==0x0)
							{						
								msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_1,MSG_TYPE_CMD);
								msg.buf=NULL;
								msg.data=DRV_STATE_VELOCITY_SET;
								xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);

								if(XVel>MIN_VELOCITY)
								{
									
									msg.data=DRV_STATE_MOVE_REL_SET;
									xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);


									
									key=__disableInterrupts();
									SysParams.ActionXComplete = ACTION_NOT_COMPLETE;
									DriveStatus.MotionXStatus=MOTION_ACTIVE;
									__restoreInterrupts(key);
								}
							}
						}	
						else if(SysParams.SubState==SYS_STPR_2_SELECT)
						{
							if(((SysParams.LimitSwState&0x4)&&(Pos2<AbsEncoderYDeg))||((SysParams.LimitSwState&0x8)&&(Pos2>AbsEncoderYDeg))||SysParams.LimitSwState==0x0)
							{
								msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_2,MSG_TYPE_CMD);
								msg.buf=NULL;
								msg.data=DRV_STATE_VELOCITY_SET;
								xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);

								
								if(YVel>MIN_VELOCITY)
								{
									
									msg.data=DRV_STATE_MOVE_REL_SET;
									xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);

									
									key=__disableInterrupts();
									SysParams.ActionYComplete = ACTION_NOT_COMPLETE;
									DriveStatus.MotionYStatus=MOTION_ACTIVE;
									__restoreInterrupts(key);
								}
							}	
						}
						else if(SysParams.SubState==SYS_STPR_JOYSTICK)
						{
							if(((SysParams.LimitSwState&0x1)&&(Pos1>AbsEncoderXDeg))||((SysParams.LimitSwState&0x2)&&(Pos1<AbsEncoderXDeg))||SysParams.LimitSwState==0x0)
							{						
								msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_1,MSG_TYPE_CMD);
								msg.buf=NULL;

								if(XVel!=PrevXVel)
								{
									if(DriveStatus.TargetPosXCmd>0.0)
										DriveStatus.TargetPosXCmd=10.0;
									else if(DriveStatus.TargetPosXCmd<0.0)
										DriveStatus.TargetPosXCmd=-10.0;
									
									msg.data=DRV_STATE_MOVE_SET;
									xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);
								}
	

								if(XVel!=PrevXVel)
								{
									if(XVel==0.0)
										msg.data=DRV_STATE_HLT_SET;
									else
										msg.data=DRV_STATE_VELOCITY_SET;
									
										xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);
									
									if(XVel>0)
									{
										key=__disableInterrupts();
										SysParams.ActionXComplete = ACTION_NOT_COMPLETE;
										__restoreInterrupts(key);									
									}
									else
									{
										key=__disableInterrupts();
										SysParams.ActionXComplete = ACTION_COMPLETE;
										__restoreInterrupts(key);									
									}																
								}
								
								
								PrevXVel=XVel;
								PrevPos1=Pos1;

							}

							if(((SysParams.LimitSwState&0x4)&&(Pos2<AbsEncoderYDeg))||((SysParams.LimitSwState&0x8)&&(Pos2>AbsEncoderYDeg))||SysParams.LimitSwState==0x0)
							{
								msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_2,MSG_TYPE_CMD);
								msg.buf=NULL;
								
								if(YVel!=PrevYVel)
								{
									if(DriveStatus.TargetPosYCmd>0.0)
										DriveStatus.TargetPosYCmd=10.0;
									else if(DriveStatus.TargetPosYCmd<0.0)
										DriveStatus.TargetPosYCmd=-10.0;
									
									msg.data=DRV_STATE_MOVE_SET;
									xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);
								}
								

								if(YVel!=PrevYVel)
								{
									if(YVel==0.0)
										msg.data=DRV_STATE_HLT_SET;
									else
										msg.data=DRV_STATE_VELOCITY_SET;
									
									xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);
									
									if(YVel>0)
									{
										key=__disableInterrupts();
										SysParams.ActionYComplete = ACTION_NOT_COMPLETE;
										__restoreInterrupts(key);									
									}
									else
									{
										key=__disableInterrupts();
										SysParams.ActionYComplete = ACTION_COMPLETE;
										__restoreInterrupts(key);									
									}											
								}
								

								PrevYVel=YVel;
								PrevPos2=Pos2;

							}								

						}
						else 
						{
							if(SysParams.LimitSwState==0x0)
							{	
								msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_1,MSG_TYPE_CMD);
								msg.buf=NULL;
								msg.data=DRV_STATE_VELOCITY_SET;
								xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);

								if(DriveStatus.VelocityXCmd>MIN_VELOCITY)
								{
									
									
									msg.data=DRV_STATE_MOVE_REL_SET;
									xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);

									
									key=__disableInterrupts();
									SysParams.ActionXComplete = ACTION_NOT_COMPLETE;
									DriveStatus.MotionXStatus=MOTION_ACTIVE;
									__restoreInterrupts(key);
								}	

								msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_2,MSG_TYPE_CMD);
								msg.buf=NULL;
								msg.data=DRV_STATE_VELOCITY_SET;
								xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);
							
							
								if(DriveStatus.VelocityYCmd>MIN_VELOCITY)
								{
									
									msg.data=DRV_STATE_MOVE_REL_SET;
									xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);
								
									
									key=__disableInterrupts();
									SysParams.ActionYComplete = ACTION_NOT_COMPLETE;
									DriveStatus.MotionYStatus=MOTION_ACTIVE;
									__restoreInterrupts(key);
								}
							}
						}						

						

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
					DriveStatus.MotionXStatus=MOTION_NOT_ACTIVE;
					DriveStatus.MotionXStatus=MOTION_NOT_ACTIVE;	
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
					DriveStatus.MotionXStatus=MOTION_NOT_ACTIVE;
					DriveStatus.MotionXStatus=MOTION_NOT_ACTIVE;					
					__restoreInterrupts(key);
					
					msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_1,MSG_TYPE_CMD);
					msg.data=DRV_STATE_MOTOR_OFF;
					msg.buf=NULL;
					xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);
				
					vTaskDelay(1);
					
					msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_2,MSG_TYPE_CMD);	
					xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);
					
					vTaskDelay(200);
	
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
		Counter=0;

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
			//key=__disableInterrupts();
			//SysParams.AbsEncStatus=STATUS_FAIL;
			//__restoreInterrupts(key);
			AbsEncoderXDeg=0.0;
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
			//key=__disableInterrupts();
			//SysParams.AbsEncStatus=STATUS_FAIL;
			//__restoreInterrupts(key);
			AbsEncoderYDeg=0.0;
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
			SysParams.AxisYPosition=/*Enc*/AbsEncoderYDeg*PI/0.00018; // Position X in uRadians
			VelSample=(SysParams.AxisYPosition-PrevPosition)*1000.0/((float)TxDelayValue+(float)RX_DELAY);
			PrevPosition=SysParams.AxisYPosition;
		    SysParams.Timestamp+=(uint32_t)TxDelayValue+(uint32_t)RX_DELAY;
			__restoreInterrupts(key);

			Emaverage  = lpf_ema_float(VelSample, Emaverage, 0.5);

			key=__disableInterrupts();
			SysParams.AxisXPosition = AbsEncoderXDeg*PI/0.00018/*Emaverage*/;// Position Y in uRadians
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


#ifdef KUKU
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
#endif

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
	if(SysParams.AbsEncXStatus==STATUS_FAIL)
		Status=STATUS_FAIL;
	if(SysParams.AbsEncYStatus==STATUS_FAIL)
		Status=STATUS_FAIL;	
	if(SysParams.FlashTest==STATUS_FAIL)
		Status=STATUS_FAIL;
	if(SysParams.RamTest==STATUS_FAIL)
		Status=STATUS_FAIL;
	if(SysParams.SafetyState!=STATUS_OK)
		Status=STATUS_FAIL;
	//if(SysParams.LimitSwState!=0x3f)
	//	Status=STATUS_FAIL;
	
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



unsigned char  GetHomePosition(char *buf, float xpos, float ypos)
{
	unsigned char  len=0;
	uint16_t Checksum;
	char *temp;


	if (buf!=NULL)
	{
		temp=buf+40;
		len+=sizeof(float);
		memcpy(temp, &xpos, sizeof(float));

		temp+=sizeof(float);
		len+=sizeof(float);
		memcpy(temp, &ypos, sizeof(float));
		

		
		Checksum=PacketChecksum(buf+40,len);

		PacketHeader.Sync_Checksum =SYNC|(((uint32_t)Checksum)<<16);
		PacketHeader.MessageSize=(uint32_t)(len+sizeof(struct sPacketHeader));
		PacketHeader.Opcode=GET_HOME_RSP;
		PacketHeader.DestinationID=1;
		PacketHeader.MsgCounter++;
		
		len+=BuildHeader(buf,&PacketHeader);
	}
		return len;
}



void GoHomePosition(float xpos, float ypos)
{
	MSG_HDR msg;	
	uint32_t key;
	float AbsEncoderXDeg;
	float AbsEncoderYDeg;
	float TargXPos;
	float TargYPos;
	
	if(SysParams.State == SYS_STATE_OPERATE)
	{
		if((DriveStatus.MotionXStatus==MOTION_NOT_ACTIVE)&&(DriveStatus.MotionYStatus==MOTION_NOT_ACTIVE))
		{
			if(AbsEncoderXData.ssiData.Error==0)
			{
				//key=__disableInterrupts();
				//SysParams.AbsEncStatus=STATUS_FAIL;
				//__restoreInterrupts(key);
			}
			else
			{
				key=__disableInterrupts();
				AbsEncoderXDeg=CntToDeg(AbsEncoderXData.raw32Data, AbsEncXOffset);
				__restoreInterrupts(key);
				if(AbsEncoderXDeg>180.0)
					AbsEncoderXDeg-=360.0;
			}
			
			
			 //EncDegreesData(AbsEncoderXData, &AbsEncoderXDeg,  AbsEncXOffset);
			
			
			
			if(AbsEncoderYData.ssiData.Error==0)
			{
				//key=__disableInterrupts();
				//SysParams.AbsEncStatus=STATUS_FAIL;
				//__restoreInterrupts(key);
			}
			else
			{
				key=__disableInterrupts();
				AbsEncoderYDeg=CntToDeg(AbsEncoderYData.raw32Data, AbsEncYOffset);
				__restoreInterrupts(key);
				if(AbsEncoderYDeg>180.0)
					AbsEncoderYDeg-=360.0;
			}	
			
			AbsEncoderXDeg=AbsEncoderXDeg*PI/0.00018;	//X position in uRadians	 TODO:Remove PI defines
			AbsEncoderYDeg=AbsEncoderYDeg*PI/0.00018;	//Y position in uRadians


			
			TargXPos=(xpos-AbsEncoderXDeg)*XRADIUS;
			TargYPos=(ypos-AbsEncoderYDeg)*YRADIUS;
			
			if(TargXPos<0.0)
				TargXPos-=BCKL_X_OFFSET;
			if(TargYPos>0.0)
				TargYPos-=BCKL_Y_OFFSET;

		
			key=__disableInterrupts();
			DriveStatus.HostPosXCmd=xpos;
			DriveStatus.HostPosYCmd=ypos;	
			DriveStatus.TargetPosXCmd=TargXPos;
			DriveStatus.TargetPosYCmd=TargYPos;
			DriveStatus.VelocityXCmd= 400.0*XRADIUS;	//Default Home Velocity - 400 uRad/sec
			DriveStatus.VelocityYCmd= 400.0*YRADIUS;	//Default Home Velocity - 400 uRad/sec
			__restoreInterrupts(key);


			msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_1,MSG_TYPE_CMD);
			msg.buf=NULL;
			msg.data=DRV_STATE_VELOCITY_SET;
			xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);
			
			msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_2,MSG_TYPE_CMD);
			xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);


									
			msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_1,MSG_TYPE_CMD);
			msg.buf=NULL;
			msg.data=DRV_STATE_MOVE_REL_SET;
			xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);

			msg.hdr.all=MAKE_MSG_HDRTYPE(0,MSG_SRC_HCMD_2,MSG_TYPE_CMD);
			xQueueSend(DriveIntQueue,&msg,portMAX_DELAY);

							
			key=__disableInterrupts();
			SysParams.ActionXComplete = ACTION_NOT_COMPLETE;
			SysParams.ActionYComplete = ACTION_NOT_COMPLETE;
			DriveStatus.MotionXStatus=MOTION_ACTIVE;
			DriveStatus.MotionYStatus=MOTION_ACTIVE;
			__restoreInterrupts(key);
		}	
	}
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


