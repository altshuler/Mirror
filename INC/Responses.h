#ifndef _RESPONSES_H
#define _RESPONSES_H

#include <stdint.h>
//#include "lspcmd.h"
//#include "records.h"

//			Type 				ASCII 		//code Meaning
#define		CMD_IS_ERROR			7			//Information represents a certain error 
#define		CMD_IS_WARN 			10			//Information is warning
#define		CMD_IS_ADC				11			//Information is direct from ADC
#define		CMD_IS_MODEL  			12			//Information is fully mathematical compensation model
#define		CMD_IS_TEXT 			13			//Information is valid
#define		CMD_IS_TEMP 			14			//Information is temperature
#define		CMD_IS_DEBUG			15			//Information is for debug
#define		CMD_IS_CREEP			17			//Information is after creep compensation
#define		CMD_IS_OFFSET			18			//Information is after offset compensation
#define		CMD_IS_GAIN 			19			//Information is after gain compensation
#define		CMD_IS_LIN				20			//Information is after linearity compensation
#define		CMD_IS_BAD				21			//Information is not to be trusted (overflow or power-related)*/



#define	RESP_SYSTEM_STATUS						1
#define	RESP_SW_VERSION							2
#define RESP_RESET_MAG_POT						30
#define RESP_SAVE_MAG_POT						31
#define RESP_LOAD_MAG_POT						32
#define RESP_READ_MAG_POT						33
#define RESP_WRITE_MAG_POT						34
#define RESP_READ_MAG_VAL						35
#define RESP_READ_A2D_REG						36
#define RESP_WRITE_A2D_REG						37
#define RESP_READ_A2D_VAL						38
#define RESP_ADC_VAL							76
#define RESP_BATTARY							77
#define RESP_BATTARY_STS						78
#define RESP_MAG_SET_RESET						79
#define RESP_SET_MAG_OFFSET						88
#define RESP_GET_MAG_OFFSET						89
#define RESP_ERROR_READ_A2D					    91
#define RESP_SET_UNIT_SN						105
#define RESP_GET_UNIT_SN						106
#define RESP_GET_IBIT							110
#define RESP_SET_CHECKSUM						111
#define RESP_GET_CHECKSUM						112
struct sSystemStatusInfoBits
{
	uint16_t ExternalPower:1;	// 0=Internal power, 1=External power
	uint16_t FlashMemoryBusy:1; // 0= Flash memory available, 1=Flash memory busy (in middle of operation)
	uint16_t PowerUpBitDone:1; // 0=No power up bit results, 1=Power up bit results available
	uint16_t RecorderActive:1; // 0=Flight recorder idle, 1=Flight recorder active
	uint16_t InBootloader:1;	// 0=Application, 1=Bootloader
	uint16_t ExternalPowerOnState:1; // 0=Internal power, 1=External power
	uint16_t GswitchState:1; // G switch input state
	uint16_t GswitchPwrUpState:1; // G switch input state at power up
	uint16_t ParametersReadState:1;
	uint16_t LSPCmdIgnore:1; // LSP cmd
	uint16_t unused:6;		// Unused bits, should be 0
};

#define SYSTEM_STATUS_INFO_EXTERNAL_POWER		0x0001
#define SYSTEM_STATUS_INFO_FLASH_MEMORY_BUSY	0x0002
#define SYSTEM_STATUS_INFO_POWER_UP_BIT_DONE	0x0004
#define SYSTEM_STATUS_INFO_RECORDER_ACTIVE		0x0008
#define SYSTEM_STATUS_INFO_IN_BOOTLOADER		0x0010
#define SYSTEM_STATUS_INFO_EXTERNAL_POWER_STATE 0x0020
#define SYSTEM_STATUS_INFO_GSWITCH_STATE		0x0040
#define SYSTEM_STATUS_INFO_GSWITCH_PWRUP_STATE	0x0080
#define SYSTEM_STATUS_INFO_PARAMETER_READ_STATE	0x0100

union uSystemStatusInfo
{
	uint16_t all;
	struct sSystemStatusInfoBits bit;
};

struct sResponseSystemStatus
{
	uint8_t CR;		// Command session identifier  (for future use)
	uint8_t Resalt[1];			// Response identifier (code)
	uint8_t Type;			// Command identifier (code)
	uint8_t EOF;
	union
	{
		union uSystemStatusInfo data;
		union uArg				uadata;
	} arg1;

};

struct sSoftwareVersionInfoBits
{
	char patch;
	unsigned char sub_minor;
	unsigned char minor;
	unsigned char major;
};

union uSoftwareVersionInfo
{
	uint32_t all;							// When all=0xffffffff no software is loaded, hence no software version
	struct sSoftwareVersionInfoBits bit;
};

struct sResponseSoftwareVersion
{
	uint16_t session_id;		// Command session identifier  (for future use)
	uint16_t resp_id;			// Response identifier (code)
	uint16_t cmd_id;			// Command identifier (code)
	uint16_t rsvd[1];
	union
	{
		union uSoftwareVersionInfo data;
		union uArg				   uadata;
	} arg1;

};


struct sResponseUnitSN
{
	uint16_t session_id;		// Command session identifier  (for future use)
	uint16_t resp_id;			// Response identifier (code)
	uint16_t cmd_id;			// Command identifier (code)
	uint16_t rsvd[1];
	union
	{
		uint32_t				UnitSN;	// Unit S/N
		union uArg				uadata;
	} arg1;
};

/*#include "Globalconf.h"

struct sResponseGetChecksum
{
	uint16_t session_id;		// Command session identifier  (for future use)
	uint16_t resp_id;			// Response identifier (code)
	uint16_t cmd_id;			// Command identifier (code)
	uint16_t rsvd[1];
	union
	{
		
		struct sChecksum		checksum;
		union uArg				uadata;
	} arg1;
};*/

#include "a2d_mng.h"
	
	struct sResponseReadA2DReg
	{
		uint16_t session_id;		// Command session identifier  (for future use)
		uint16_t resp_id;			// Response identifier (code)
		uint16_t cmd_id;			// Command identifier (code)
		uint16_t rsvd[1];
		union
		{
			struct sRegA2D			data;
			union uArg				uadata;
		} arg1;
	};
	
	struct sResponseWriteA2DReg
	{
		uint16_t session_id;		// Command session identifier  (for future use)
		uint16_t resp_id;			// Response identifier (code)
		uint16_t cmd_id;			// Command identifier (code)
		uint16_t rsvd[1];
		union
		{
			struct sRegA2D			data;
			union uArg				uadata;
		} arg1;
	};
	
	struct sResponseReadA2D
	{
		uint8_t CR;		// Command session identifier  (for future use)
		uint8_t Resalt[12];			// Response       ADC=00000000
		uint8_t Type;			// Command identifier (code)
		uint8_t EOF;
		/*union
		{
			struct sIntA2D			data;
			union uArg				uadata;
		} arg1;*/
	};
	

union uAllResponses
{
	struct sResponseHeader	hdr;
	struct sResponseSystemStatus sys_status;
	struct sResponseSoftwareVersion sw_ver;
	struct sResponseReadA2D read_a2d;
	struct sResponseWriteA2DReg write_a2d_reg;
	struct sResponseReadA2DReg read_a2d_reg;
	/*struct sResponseResetMagPot reset_mag_pot;
	struct sResponseLoadMagPot load_mag_pot;
	struct sResponseUnitSN unit_sn_res;
	struct sResponseGetChecksum get_checksum;*/
	// Place to add additional response structures
};

union uSecResponses
{
	uint8_t	hdr;
	uint16_t data;
};

#endif

