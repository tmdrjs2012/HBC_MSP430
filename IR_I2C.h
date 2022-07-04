/*
 * IR_I2C.h
 *
 *  Created on: 2022. 6. 30.
 *      Author: hc.ro
 */

#ifndef IR_I2C_H_
#define IR_I2C_H_

#include "msp430.h"
#include "stdint.h"

#define TBP_ADDR 0x3A

int     RX_Byte_Ctr,        // Coutner to make sure all of the information is received
        TX_Byte_Ctr,        // Counter to make sure all of the information is sent
        i;                  // Integer used for counting sent bytes
char    i2cData[3],          // Creates an array to store data
        //sending[1],         // Creates an array to store the sent data
        *pointer,           // Creates a pointer to access the array
        *txPtr;

int16_t _rawObject;
int16_t result_object;
//int16_t _rawSensor;
uint8_t BUF[5];
int16_t *dest_call;
int PEC_ture;

void i2cInit(void);                                 // IR Temperature I2C CLK Speed and GPIO initialize
float CalcTemp(int rawTemp);                        // temperature calculation
uint8_t GetObject(void);                            // Read Object Temperature
uint8_t CalPEC(uint8_t *crc, uint8_t nBytes);       // PEC calculation


#endif /* IR_I2C_H_ */
