/*
 * IR_I2C.c
 *
 *  Created on: 2022. 6. 30.
 *      Author: hc.ro
 */

#ifndef IR_I2C_C_
#define IR_I2C_C_

#include "IR_I2C.h"

//I2C temperature Sensor CRC table (Please don't change this value)
const unsigned char crc8_table[256]=
{
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,    0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,    0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

void i2cInit(void){
    P7SEL1 &= ~BIT1;    // SCL
    P7SEL0 |= BIT1;

    P7SEL1 &= ~BIT0;    // SDA
    P7SEL0 |= BIT0;

    UCB2CTLW0 |= UCSWRST;   // Enters reset state, USCI stops operation

    //UCB2TBCNT = UCTBCNT3;   // Expecting to receive 3 bytes of data
    UCB2CTLW1 |= UCASTP_2;  // Sends stop bit when UCTBCNT is reached

    UCB2CTLW0 |= UCMST      // Master Mode
              |  UCMODE_3   // I2C Mode
              |  UCSSEL_3;  // Sets SMCLK as source
    UCB2BRW    = 0x0028;    // SMCLK/10

    UCB2CTLW0 &= ~UCSWRST;  // Exits reset mode, USCI enters operation
    UCB2IE    |= UCTXIE0    // Data received interrupt enable
              |  UCRXIE0    // Data ready to transmit interrupt enable
              |  UCNACKIE;  // NACK interrupt enable
}

// I2C Temperature Get Data
void getData(void){

    pointer = &i2cData;                     // Sets the pointer to the height array
    RX_Byte_Ctr = 3;                        // Determines the number of bytes received
    TX_Byte_Ctr = 1;                        // Determines the number of bytes sent
    UCB2I2CSA = TBP_ADDR;                   // Sets slave address

    UCB2TBCNT = 0x01;                       // Expecting to receive 3 bytes of data
    UCB2CTLW0 |= UCTR | UCTXSTT;            // Enables TX Mode, Sends start condition
    //__bis_SR_register(LPM0_bits | GIE);     // Enters Low-Power mode and enables global interrupt ???

    //UCB2CTLW1 |= UCASTP_2;  // Sends stop bit when UCTBCNT is reached
    UCB2TBCNT = 3;                          // Expecting to receive 3 bytes of data
    //__delay_cycles(20);
    //__delay_cycles(2000);
    __delay_cycles(2000);
    UCB2CTLW0 &= ~UCTR;                       // Enters RX Mode
    UCB2CTLW0 |= UCTXSTT;                     // Sends start condition
    //__bis_SR_register(LPM0_bits | GIE);     // Enters Low-Power mode and enables global interrupt  ???
}

float CalcTemp(int rawTemp)                                     // Temperature Calculation
{
  float retTemp;
  retTemp = ((float)rawTemp)*0.02;
  retTemp -= 273.15;
  return retTemp;
}

uint8_t GetObject(void)                                         // Read Object Temperature
{
  int16_t rawObj;
  if(PEC_ture == 1)
  {
    if (rawObj & 0x8000)
    {
      return 0;
    }
    result_object = _rawObject;
    return 1;
  }
}

uint8_t CalPEC(uint8_t *crc, uint8_t nBytes)                        // PEC Calculation
{
  uint8_t data, count;
  uint16_t remainder = 0;

  for(count=0; count<nBytes; ++count)
  {
     data = *(crc++) ^ remainder;
     remainder = crc8_table[data] ^ (remainder >> 8);
  }
  return (uint8_t)remainder;
}

#pragma vector = USCI_B2_VECTOR
__interrupt void USCI_B2_ISR(void)
{
    switch(__even_in_range(UCB2IV, USCI_I2C_UCBIT9IFG))
    {
        case USCI_I2C_UCNACKIFG:                                    // NACK Interrupt
            UCB2CTLW0 |= UCTXSTT;                                   // I2C start condition
        break;

        case USCI_I2C_UCRXIFG0:                                     // I2C RX Interrupt
            if (RX_Byte_Ctr > 1){                                   // Checks if there is more data to be received
                *pointer = UCB2RXBUF;                               // Loads the data array
                pointer++;                                          // Increments the pointer
                RX_Byte_Ctr--;                                      // Decrement RX byte counter
} else if(RX_Byte_Ctr == 1){                                        // If all of the data is received
                *pointer = UCB2RXBUF;
                pointer = 0;
            }
        break;

        case USCI_I2C_UCTXIFG0:                                     // I2C TX Interrupt
            if(TX_Byte_Ctr > 1){                                    // If there is more data to send
                UCB2TXBUF = *txPtr;                                 // Loads TX buffer from array
                i++;                                                // Increments array position
                TX_Byte_Ctr--;                                      // Decrements TX byte counter
            } else if(TX_Byte_Ctr == 1){                            // If there is nothing left to say
                UCB2TXBUF = 0x07;
            }
        break;
    }
}

#endif /* IR_I2C_C_ */
