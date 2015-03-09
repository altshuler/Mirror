#ifndef __TIMESTAMP_H
#define __TIMESTAMP_H

#include <stdint.h>
#include "freertos.h"
#include "task.h"

typedef uint32_t TIMESTAMP;

#define readTimeStampGen() ((TIMESTAMP)xTaskGetTickCount())
#define readTimeStampGenFromIsr() ((TIMESTAMP)xTaskGetTickCountFromISR())
void SyncTimestampe (void);


#endif

