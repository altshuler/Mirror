/**
* @file flashcsum.h
* @brief Application flash memory checksum
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 24.10.2013
*/
#ifndef _FLASHCSUM_H
#define _FLASHCSUM_H
	
#include <stddef.h>
#include <stdint.h>

#ifdef DEBUG_FLASH_CSUM
extern const uint32_t flashChecksum;
#else
extern uint32_t flashChecksum;
#endif


#ifdef __cplusplus
extern "C" 
{
#endif

uint32_t calcFlashChecksum(void *start, size_t length);
uint32_t calcFlashChecksumByParts(uint32_t csum, void *start, size_t length, int last);

#ifdef __cplusplus
}
#endif

#endif

