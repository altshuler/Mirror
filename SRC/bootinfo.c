/**
* @file bootinfo.c
* @brief Bootloader-Application shared information
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 24.10.2013
*/
#include <stddef.h>
#include <stdint.h>
#include "bootinfo.h"

struct sBootInfoArea bootInfo __attribute__ ((section (".bootinfo")));

