/**
* @file Eeprom.c
* @brief system portable functions
*
* @author Evgeny  Altshuler
*
* @version 0.0.1
* @date 06.11.2014
*/

#include <string.h>
#include "stm32f2xx.h"
#include <FreeRtos.h>
#include <task.h>
#include "i2c.h"
#include "gio.h"
#include <dev.h>
#include "board_i2c.h"
#include "Eeprom.h"
#include "board.h"



i2c_ChanParams      	i2cChanParams		= {NULL};
extern i2c_Params       i2cParams0;
GIO_AppCallback 		gioEEpromAppCallback;


xSemaphoreHandle EEpromSem=NULL;
extern GIO_Handle  i2cHandle[];


ErrorStatus EEWriteData(uint8_t *p, uint16_t  addr, uint32_t  datalen)
{
	uint8_t TxBuffer[E2PROM_PAGE_LENGTH+2];
	size_t len = sizeof(i2c_DataParam);
	i2c_DataParam dataParam;

	if (EEpromSem==NULL)
		return ERROR;
	
	if( xSemaphoreTake( EEpromSem, portMAX_DELAY) == pdTRUE )
	{
		TxBuffer[0]=(uint8_t)(addr>>8);
		TxBuffer[1]=(uint8_t)(addr&0xFF);
		memcpy(&TxBuffer[2],p,datalen);
		datalen+=2;
		
		E2PROM_WRITE_ALLOW;
		dataParam.deviceAddress = 0xA4;
		dataParam.flags = I2C_WRITE;
		dataParam.inBufLen = 0;
		dataParam.inBuffer = NULL;
		dataParam.outBufLen = datalen;
		dataParam.outBuffer = TxBuffer;
		GIO_submit(i2cHandle[0], IODEV_WRITE, &dataParam, &len, NULL);
		xSemaphoreGive(EEpromSem);
		E2PROM_WRITE_PROTECT;
		
		return SUCCESS;
	}
	else
		return ERROR;
}


ErrorStatus EEReadData(uint8_t *p, uint16_t  addr, uint32_t  datalen)
{
	size_t len = sizeof(i2c_DataParam);
	uint8_t TxBufferRegAddr[2];
	i2c_DataParam dataParam;
	int status=IODEV_COMPLETED;
	

	if (EEpromSem==NULL)
	{
		return ERROR;
	}

	if( xSemaphoreTake( EEpromSem, portMAX_DELAY) == pdTRUE )
	{
		E2PROM_WRITE_ALLOW;
		TxBufferRegAddr[0]=(uint8_t)(addr>>8);
		TxBufferRegAddr[1]=(uint8_t)(addr&0xFF);
		dataParam.deviceAddress = 0xA4;
		dataParam.flags 		= I2C_WRITE;
		dataParam.inBufLen 		= 0;
		dataParam.inBuffer 		= NULL;
		dataParam.outBuffer 	= TxBufferRegAddr;
		dataParam.outBufLen 	= 2;
		//GIO_submit(i2cHandle[0], IODEV_WRITE, &dataParam, &len, &gioRTCWriteAppCallback);
		status=GIO_submit(i2cHandle[0], IODEV_WRITE, &dataParam, &len, NULL);
		//vTaskDelay(1);
		E2PROM_WRITE_PROTECT;
		
		if (status!=IODEV_COMPLETED)
		{
			xSemaphoreGive(EEpromSem);
			return ERROR;
		}
		dataParam.deviceAddress = 0xA5;
		dataParam.flags 		= I2C_READ;
		dataParam.inBufLen 		= datalen;
		dataParam.inBuffer 		= p;
		dataParam.outBufLen 	= 0;
		dataParam.outBuffer 	= NULL;
		//GIO_submit(i2cHandle[0], IODEV_READ, &dataParam, &len, &gioRTCReadAppCallback);
		status=GIO_submit(i2cHandle[0], IODEV_READ, &dataParam, &len, NULL);
		if (status!=IODEV_COMPLETED)
		{
			xSemaphoreGive(EEpromSem);
			return ERROR;
		}
		

		xSemaphoreGive(EEpromSem);
		return SUCCESS;
		
	}
	else
	{
		return ERROR;
	}
}



void InitEEprom()
{
	GIO_Attrs gioAttrs = GIO_ATTRS;
	i2cChanParams.hDma=i2cParams0.hDma;
	//i2cChanParams.hDma = NULL;
	i2cHandle[0] =  GIO_create("/i2c1",IODEV_INOUT,NULL,&i2cChanParams,&gioAttrs);
	vSemaphoreCreateBinary(EEpromSem);
}

ErrorStatus EEErasePage(uint16_t  addr)
{
	uint8_t  buf[E2PROM_PAGE_LENGTH];
	ErrorStatus status;

	if(addr%E2PROM_PAGE_LENGTH)
		return ERROR;

	memset(buf,0xff,E2PROM_PAGE_LENGTH);

	status = EEWriteData(buf, addr, E2PROM_PAGE_LENGTH);

	return status;	
}



ErrorStatus EEEraseAllMemory(void)
{
	uint16_t  addr;
	
	for(addr=0;addr<E2PROM_MEMORY_SIZE;addr+=E2PROM_PAGE_LENGTH)
	{
		if(ERROR == EEErasePage(addr))
			return ERROR;
		vTaskDelay(1);	
	}
			return SUCCESS;
}




ErrorStatus getApplicationIPSettings(struct sAppIpSettings *d)
{
uint8_t	*inBuffer = NULL;
uint32_t  len=0;
ErrorStatus status;

	inBuffer = (uint8_t *)d;
	
	if(inBuffer==NULL)
		return ERROR;
	else
	{
		len=(uint32_t)(sizeof(struct sAppIpSettings));
		status = EEReadData(inBuffer, IP_SETTINGS_ADDR , len);

		if(d->Ip.IPAddress==0xffffffff)
			return ERROR;
		 else 
		 	return status;
	}
}


ErrorStatus setApplicationIPSettings(struct sAppIpSettings *d)
{
uint8_t	*Buffer = NULL;
uint32_t  len=0;
ErrorStatus status;

	Buffer = (uint8_t *)d;
	
	if(Buffer==NULL)
		return ERROR;
	else
	{
		len=(uint32_t)(sizeof(struct sAppIpSettings));
		status = EEWriteData(Buffer, IP_SETTINGS_ADDR , len); 

		return status;
	}
}


ErrorStatus SetApplicationData(uint32_t *AbsOffset, uint16_t addr)
{
uint8_t	*Buffer = NULL;
uint32_t  len=0;
ErrorStatus status;

	Buffer = (uint8_t *)AbsOffset;
	
	if(Buffer==NULL)
		return ERROR;
	else
	{
		len=(uint32_t)(sizeof(uint32_t));
		status = EEWriteData(Buffer, addr , len); 

		return status;
	}
}



ErrorStatus GetApplicationData(uint32_t *AbsOffset, uint16_t addr)
{
uint8_t	*Buffer = NULL;
uint32_t  len=0;
ErrorStatus status;

	Buffer = (uint8_t *)AbsOffset;
	
	if(Buffer==NULL)
		return ERROR;
	else
	{
		len=(uint32_t)(sizeof(uint32_t));
		status = EEReadData(Buffer, addr , len); 

		
		if(*((uint32_t *)(Buffer))==0xffffffff)
			return ERROR;
		 else 
		 	return status;
	}
}

