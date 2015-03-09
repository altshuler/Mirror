/**
* @file Eeprom.h
* @brief system portable functions
*
* @author Evgeny Altshuler
*
* @version 0.0.1
* @date 06.11.2014
*/

#ifndef EEPROM_H_
#define EEPROM_H_

#include "ip_param_config.h"

#define E2PROM_WRITE_PROTECT	GPIO_SetBits(WP_GPIO_PORT,WP_PIN);
#define E2PROM_WRITE_ALLOW		GPIO_ResetBits(WP_GPIO_PORT,WP_PIN);

#define E2PROM_PAGE_LENGTH		64
#define E2PROM_MEMORY_SIZE		0x8000 //24LC256 - 32Kbytes Memory

#define IP_SETTINGS_ADDR		0
#define IP_SETTINGS_SIZE		28
#define ABS_X_ENC_OFFSET_ADDR		IP_SETTINGS_ADDR+IP_SETTINGS_SIZE
#define ABS_X_ENC_OFFSET_SIZE		4
#define ABS_Y_ENC_OFFSET_ADDR		ABS_X_ENC_OFFSET_ADDR+ABS_X_ENC_OFFSET_SIZE
#define ABS_Y_ENC_OFFSET_SIZE		4


ErrorStatus EEWriteData(uint8_t *p, uint16_t  addr, uint32_t  datalen);
ErrorStatus EEReadData(uint8_t *p, uint16_t  addr, uint32_t  datalen);
ErrorStatus getApplicationIPSettings(struct sAppIpSettings *d);
ErrorStatus setApplicationIPSettings(struct sAppIpSettings *d);
ErrorStatus SetApplicationData(uint32_t *AbsOffset, uint16_t addr);
ErrorStatus GetApplicationData(uint32_t *AbsOffset, uint16_t addr);
void InitEEprom();
ErrorStatus EEErasePage(uint16_t  addr);
ErrorStatus EEEraseAllMemory(void);


#endif /* EEPROM_H_ */
