/**
* @file endianutils.h
* @brief endian manipulation support
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 26.06.2012
*/
#ifndef _ENDIANUTILS_H
#define _ENDIANUTILS_H

#include <stddef.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

static inline uint16_t shortBE2LE(uint16_t in)
{
	return (in<<8)|(in>>8);
}

static inline uint16_t shortLE2BE(uint16_t in)
{
	return (in<<8)|(in>>8);
}

static inline uint32_t longBE2LE(uint32_t in)
{
	return (in<<24)|((in>>8)&0x0000ff00)|((in<<8)&0x00ff0000)|(in>>24);
}

static inline uint32_t longLE2BE(uint32_t in)
{
	return (in<<24)|((in>>8)&0x0000ff00)|((in<<8)&0x00ff0000)|(in>>24);
}


#ifdef __cplusplus
}
#endif


#endif

