/*
 * bootloaderrequest.c
 *
 *  Created on: Mar 11, 2014
 *      Author: davida
 */


#include <stdint.h>


uint32_t magicBootloaderStart[4] __attribute__ ((section (".bootloaderreqsig")));
