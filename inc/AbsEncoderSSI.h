/*
 * AbsEncoderSSI.h
 *
 *  Created on: Apr 23, 2014
 *      Author: davida
 */

#ifndef ABSENCODERSSI_H_
#define ABSENCODERSSI_H_


#define SSI_SECIAL_BITS 	3
#define SSI_RESOLUTION 		10 //can be 10 or 12
#define SSI_REVOLUTION 		15 //= SSI_NUM_OF_BITS - SSI_SECIAL_BITS - SSI_RESOLUTION
#define SSI_NUM_OF_BITS		28 // can be 28,31 or 33



//typedef struct
//{
//	uint32_t part2		:SSI_REVOLUTION;
//	uint32_t part1		:SSI_RESOLUTION;
//	uint32_t Error		:1;
//	uint32_t Warning	:1;
//	uint32_t Parity		:1;
//	uint32_t spare		:4;
//}SSI_DATA;



#ifndef RANISHOW

typedef struct
{
	uint32_t Parity		:1;
	uint32_t Warning	:1;
	uint32_t Error		:1;
	uint32_t part1		:SSI_RESOLUTION;
	uint32_t part2		:SSI_REVOLUTION;
	uint32_t spare		:4;
}SSI_DATA;

typedef union
{
	SSI_DATA ssiData;
	uint8_t rawData[4];
	uint32_t raw32Data;
}uSSI;


#else

typedef struct
{
	uint64_t res		:16;
	uint64_t spare		:1;
	uint64_t crc_l		:4;
	uint64_t crc_h		:2;
	uint64_t Warning	:1;
	uint64_t Error		:1;
	uint64_t position_l	:7;
	uint64_t position_h	:19;
	uint64_t zero		:1;
	uint64_t start		:1;
	uint64_t ack		:11;
}SSI_DATA;

typedef union
{
	SSI_DATA ssiData;
	uint16_t rawData[4];
	uint64_t raw32Data;

}uSSI;


#endif





void init_SSI();
uint32_t getAngle();
uSSI getGPIO_SSI_Angle();
void NVIC_Configuration(uint8_t ch, uint8_t prio, uint8_t subprio);

#endif /* ABSENCODERSSI_H_ */
