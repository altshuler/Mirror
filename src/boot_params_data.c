/**
* @file production_data.c
* @brief Production related information
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 06.03.2014
*/
#include <stddef.h>
#include <stdint.h>
#include "param_vol.h"

uint8_t bootParamVolArea[PARAM_DATA_BLOCK_SIZE*2] __attribute__ ((section (".bootparam")));


