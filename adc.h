/*
 * adc.h
 *
 *  Created on: 2022. 5. 13.
 *      Author: hc.ro
 */

#ifndef ADC_H_
#define ADC_H_

#include "msp430.h"
#include "stdint.h"

void Init_ADC_GPIO(void);
int AnalogRead(uint8_t channel);

#endif /* ADC_H_ */
