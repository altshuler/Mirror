/**
* @file appinfo.c
* @brief Application download information
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 24.10.2013
*/
#include <stddef.h>
#include <stdint.h>
#include "appinfo.h"

struct sAppInfoArea appInfo __attribute__ ((section (".appinfo")));

