#include <stdint.h>
#include <stddef.h>
#include "crc16.h"

// Table for cached 4-bit CRC-16 lookups
static const unsigned short crctable[16] = {
	0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401,
	0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400 };



/********************************************************************
*																	*
*	Function: crc16													*
*																	*
*	Arguments:														*
*	Uint16 crc		-	Previous CRC value 							*
*	Uint8 const *buffer	-	Data pointer  									*
*	size_t len			-	Number of bytes in the buffer 					*
*																	*
*	Returns:															*
*	Uint16			-	Updated CRC value							*
*																	*
*	Description:														*
*	This function calculates CRC16 (polynomial x^16 + x^15 + x^2 + 1)		*
*	cyclyc redundancy check of a data buffer.								*
*																	*
********************************************************************/
unsigned short crc16 (unsigned short crc, char  *buffer, size_t len)
{
	while (len--)
	{
		// CRC the lower 4 bits
		crc = (crc >> 4) ^ crctable[((crc ^ (*buffer & 0xF)) & 0xF)];

		// CRC the upper 4 bits
		crc = (crc >> 4) ^ crctable[((crc ^ (*buffer >> 4)) & 0xF)];

		// Move on to the next element
		buffer++;
	}

	// Return the cumulative CRC-16 value
	return crc;
}

/********************************************************************
*																	*
*	Function: crc16_byte												*
*																	*
*	Arguments:														*
*	Uint16 crc		-	Previous CRC value 							*
*	Uint8 const *buffer	-	Data pointer  									*
*	size_t len			-	Number of bytes in the buffer 					*
*																	*
*	Returns:															*
*	Uint16			-	Updated CRC value							*
*																	*
*	Description:														*
*	This function calculates CRC16 (polynomial x^16 + x^15 + x^2 + 1)		*
*	cyclyc redundancy check of a data buffer.								*
*																	*
********************************************************************/
unsigned short crc16_byte(unsigned short crc, unsigned char byte)
{
	// CRC the lower 4 bits
	crc = (crc >> 4) ^ crctable[((crc ^ (byte & 0xF)) & 0xF)];
	
	// CRC the upper 4 bits
	crc = (crc >> 4) ^ crctable[((crc ^ (byte >> 4)) & 0xF)];

	// Return the cumulative CRC-16 value
	return crc;
}

uint16_t checkFrameCrc(char *data,size_t len)
{
	return (uint16_t)(crc16(0x0000, data,len));
}


uint16_t calcHostFrameCrc(char *data,size_t len)
{
	return (uint16_t)(crc16(0x0000, data+sizeof(char)+sizeof(char),len));
}



