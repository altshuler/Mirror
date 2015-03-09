/*
 * voltage_check.h
 *
 *  Created on: April 7, 2014
 *      Author: David Anidjar
 */

#ifndef VOLTAGE_CHECK_H_
#define VOLTAGE_CHECK_H_


void ADC3_Config(void);
void ADC1_Config(void);
int check_voltage(void);
void CheckTemperature(void);


#endif /* VOLTAGE_CHECK_H_ */
