/*
 * adc.c
 *
 *  Created on: 2022. 5. 13.
 *      Author: hc.ro
 */



#ifndef ADC_C_
#define ADC_C_

#include "adc.h"

void Init_ADC_GPIO(void)
{
    // Configure ADC12 - A12, A13, A14, A15 GPIO
    // arm, back , leg, roller_1
    P3SEL1 |= BIT0 | BIT1 | BIT2 | BIT3;                  // Configure P3.0 , P3.1, P3.2 and P3.3 for ADC
    P3SEL0 |= BIT0 | BIT1 | BIT2 | BIT3;

    // Configure ADC12 - A8, A9, A10, A11 GPIO
    // roller_2, roller_3, roller4
    P4SEL1 |= BIT0 | BIT1 | BIT2 | BIT3;                         // Configure P4.0 , P4.1 and P4.2 for ADC
    P4SEL0 |= BIT0 | BIT1 | BIT2 | BIT3;
}

int AnalogRead(uint8_t channel)
{
    /* Configure ADC Channel */
    ADC12CTL0 &= ~ADC12ENC; //make sure ENC is 0 so we can configure the read
    ADC12CTL0 = ADC12SHT0_3 + ADC12ON; //64 clk ticks, ADC on, enable interrupt
    ADC12CTL1 = ADC12SSEL_0 | ADC12SHP_1; // ADC12OSC
    ADC12MCTL0 = channel; // channel
    //ADC12MCTL0 = channel | ADC12VRSEL_1; // channel + Vref
    ADC12CTL0 |= ADC12ENC + ADC12SC; //Enable and start conversion
    while ((ADC12BUSY & ADC12CTL1) == 0x01); //Wait for conversion to end


    return ADC12MEM0;
}

#endif /* ADC_C_ */
