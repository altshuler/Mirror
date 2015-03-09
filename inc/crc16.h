#ifndef __CRC16_H
#define __CRC16_H

#include <stdint.h>

//unsigned short crc16 ( unsigned short   crc, unsigned char const *buffer, size_t len);
unsigned short crc16 (unsigned short crc, char  *buffer, size_t len);

unsigned short crc16_byte(unsigned short crc, unsigned char byte);
uint16_t checkFrameCrc(char *data,size_t len);
uint16_t calcHostFrameCrc(char *data,size_t len);



#endif
