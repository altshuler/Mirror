/**
* @file production_data.c
* @brief Production related information
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 03.11.2013
*/
#include <stddef.h>
#include <stdint.h>
#include "production_data.h"

union uBootProductionDataArea bootProductionData __attribute__ ((section (".bootfactoryinfo")));


