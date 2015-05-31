#ifndef __CBITTASK_H
#define __CBITTASK_H

#include "AbsEncoderSSI.h"


#define ABS_ENC_RADIUS			57.30
//#define ABS_ENC_OFFSET			0x8000 //20 deg

#ifndef RANISHOW
#define ABS_ENC_MAX_VAL			0x59FFF
#else
#define ABS_ENC_MAX_VAL			0x3FFFFFF
#endif

struct sDiscreteBits
{
	uint8_t discrete_1_4:4;
	uint8_t discrete_5_6:2;
	uint8_t spare:2;
};

union uDiscrete
{
	uint8_t all;
	struct sDiscreteBits bit;
};

void CBITTask(void * pvParameters);			// D.A. Line added
void Emergency_Int_EXTIConfig(void);
uint32_t DegToCnt(float angle, uint32_t offset);
void RamCheck(void);
void flashCheck(void);
int isBlank(uint8_t *p, size_t len);
void CheckDiscrets(void);
void GetTravelPinPosition(void);



#endif
