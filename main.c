
//--------------------------------------//
//          ADC 7 channel               //
//          3.0, 3.1, 3.2, 3.3 PIN      //
//          A12, A13, A14, A15          //
//          4.0, 4.1, 4.2 PIN           //
//          A8,  A9,  A10               //
//                                      //
//          PWM 4 channel               //
//          3.4, 3.5, 3.6, 3.7 PIN      //
//          TB0.3, TB0.4 TB0.5 TB0.6    //
//                                      //
//          UART ( <-> Display)         //
//          P2.5, P2.6                  //
//          UCA1TXD, UCA1RXD            //
//                                      //
//          UART ( <-> PC )             //
//          P5.4, P5.5                  //
//          UCA2TXD,, UCA2RXD           //
//                                      //
//          Temperature Send            //
//          PID Control                 //
//          AutoTuning Mode             //
//--------------------------------------//


// Normal Mode
// temperature controller 0x90 -> PC
// temperature controller <- PC 0x80

// AT Mode START
// temperature controller 0x92 -> PC
// temperature controller <- PC 0x82

// AT Mode STOP
// temperature controller 0x91 -> PC
// temperature controller <- PC 0x81


//-------------------------------------------------------------------
//RX data is 1byte - 2byte - 2byte - 2byte - 2byte - 1byte - 1byte
//RX data[11]
//Rx data([1]+[2])/10 = CH1 target temp
//RX data([3]+[4])/10 = CH2 target temp
//RX data([5]+[6])/10 = CH3 target temp
//RX data([7]+[8])/10 = CH4 target temp
//RX data[9] = MSB --- Heater On/Off (State/Control bit)

// UCA3TXD -> UCA2TXD
// UCA3RXD -> UCA2RXD

//-------------------------------------------------------------------

#include <msp430.h>
#include "uart.h"
#include "adc.h"
#include <IR_I2C.h>

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
//#include <time.h>
#include <math.h>

#define OFFSETVOLTAGE 0.1
#define MSP_CPU_CLK 4000000

//---------------------- PID Command ---------------------------

float Kp_Value[4] = {1.8, 1.75, 1.65, 1.75};
float Ki_Value[4] = {0.6, 0.5, 0.55, 0.5};
float Kd_Value[4] = {3.75, 3.65, 3.55, 3.65};

// 1.44     0.072   0.03
// 2.4      0.018   0.01
// 2.4      0.018   0.01
// 2.4      0.018   0.01
// Ziegler Nicholes Methode 참조

unsigned char P_CastingBuffer[4][4];
unsigned char I_CastingBuffer[4][4];
unsigned char D_CastingBuffer[4][4];

unsigned char pid_CastingBuffer[4][4];

unsigned char FtoCBuffer[48];
volatile unsigned char CtoFBuffer[48];

volatile unsigned int PID_Flag[4] = {0, };

volatile float pidResult[4] = {0, 0, 0, 0};     // pull up
float AT_pidResult[4] = {0, 0, 0, 0};

volatile float ConfigTemp[4] = {35, 35, 35, 35};
volatile unsigned int SendGpioFlag[4] = {0, };

float TB_Temp;

//--------------------------------------------------------------

//---------------------- UART Variables --------------------------

unsigned char CheckQueue[11];

unsigned char PC_SendMessageFlag;
unsigned char DP_SendMessageFlag;

volatile unsigned int xFlag = 0;
volatile int a;

unsigned short int tempIntVal_ch1;          // integer temperature Channel 1 Variable
unsigned short int targetIntVal_ch1;        // integer target temperature channel 1 Variable

unsigned short int tempIntVal_ch2;          // integer temperature Channel 2 Variable
unsigned short int targetIntVal_ch2;        // integer target temperature channel 2 Variable
unsigned short int tempIntVal_ch3;          // integer temperature Channel 3 Variable
unsigned short int targetIntVal_ch3;        // integer target temperature channel 3 Variable

unsigned short int tempIntVal_ch4;          // integer temperature Channel 4 Variable
unsigned short int targetIntVal_ch4;        // integer target temperature channel 4 Variable

unsigned short tempIntVal_i2c;              // integer IR temperature Channel 1 Variable

unsigned char PC_reQuestFlag = 0;
unsigned char PC_TransmitStart = 0;
unsigned char DP_reQuestFlag = 0;
unsigned char DP_TransmitStart = 0;

//--------------------------------------------------------------

//////////////////////////       ADC       //////////////////////////

float voltageArray[4];
float TempVal[4];

float TempI2C = 45.2;

unsigned int ADC_Result[4];
volatile int ADC_CalcurationFlag;

float ADC_Sample[4];
volatile unsigned int ADC_Temp[4][5];
unsigned char SaveOnOffState = 0x00;

int sample_index = 0;
int ADC_CH1_Controller;
int I2C_Sensor;

int SampleComplete;
int ADC_FLAG;
int END_FLAG;

void _commandZone(unsigned char *_newCommand);
void Init_PWM_GPIO(void);
void Init_SMCLK_4MHZ(void);
void Init_ADC_GPIO(void);
void Init_ADC(void);

void i2cInit(void);                                 // IR Temperature I2C CLK Speed and GPIO initialize
float CalcTemp(int rawTemp);                        // temperature calculation
uint8_t GetObject(void);                            // Read Object Temperature
uint8_t CalPEC(uint8_t *crc, uint8_t nBytes);       // PEC calculation

void Init_TIMER_A0(void);

// change parameter and return value as float -> double
float PID_Contorller(float kp, float ki, float kd, float targetValue, float readValue);

int AnalogRead(uint8_t channel);
void MessageTx(void);
void MessageTx2(void);
void Tx_PID_Tuning(unsigned char commandTx, unsigned char SelectedChannel);

void stopWatchDog(void)
{
    WDTCTL = WDTPW + WDTHOLD;         // Stop Watch Dog timer
}

int timeSamplingCount;
int timeCount_PID;

extern unsigned char CheckBreak;

//------------------------------- TB I2C -----------------------------------------------------------
int TB_Sensor_Flag;
//----------------------------------------------------------------------------------------------------------------------

//-------------------------------------- FRAM ACCESS ----------------------------------------------------
// 4byte (variables Type = float) * 3 (control number = P, I, D) * 4 (Channel number) = 48

#define WRITE_SIZE      48              // 90 +48
#define FRAM_ADDR       0x004000     // allocation 300(decimal)

unsigned char RAMData;

//void FRAMWrite(unsigned char inputData);
void FRAMWrite(unsigned char *inputData);
void FloatToByte(void);
void ByteToFloat(void);

unsigned char P_MemoryBuffer[4][4];      // P-Gain  4 Channel and 4byte
unsigned char I_MemoryBuffer[4][4];      // P-Gain  4 Channel and 4byte
unsigned char D_MemoryBuffer[4][4];      // P-Gain  4 Channel and 4byte

float Kp_Temp[4];
float Ki_Temp[4];
float Kd_Temp[4];

int FLASH_FLAG;


#if defined(__TI_COMPILER_VERSION__)
#pragma PERSISTENT(FRAM_write)
unsigned char FRAM_write[WRITE_SIZE] = {0};
#elif defined(__IAR_SYSTEMS_ICC__)
__persistent unsigned char FRAM_write[WRITE_SIZE] = {0};

#elif defined(__GNUC__)
unsigned char __attribute__((persistent)) FRAM_write[WRITE_SIZE] = {0};

#else
#error Compiler not supported!
#endif

extern int BufferCount;
extern int ShiftCount;
extern unsigned char BufferChecking[12];

extern int BufferCount2;
extern unsigned char BufferChecking2[12];
//int AT_MODE_FLAG;

unsigned char AT_NORMAL_FLAG = 0;
unsigned char AT_STOP_FLAG = 0;
unsigned char AT_START_FLAG = 0;
unsigned char DP_NORMAL_FLAG = 0;

unsigned char Selected_Channel;
unsigned short int AT_currentTemp;
unsigned short int AT_targetTemp;

unsigned char TuningStartFlag = 0;
unsigned char TuningStopFlag = 0;

float voltage_check[4];
float roller_voltage;


unsigned char rxReFlag;
int pwmResult;

int main(void)
{
    int k = 0;

    Init_ADC_GPIO();
    Init_PWM_GPIO();

    // Relay GPIO
    P2DIR |= BIT2;


    Init_SMCLK_4MHZ();
    stopWatchDog();

    i2cInit();
    __delay_cycles(4800000);        // 16MHz / 2 about 0.5sec
    Init_UART();                        // uart.h
    Init_UART2();
    QueueInit(&q);                     // Queue initialize

    Init_TIMER_A0();


    int readIndex = 0;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // To read FRAM it, you have to uncomment it
/*
    RAMData = ( *(volatile unsigned char *)(FRAM_ADDR) );      // Initialize Fram Position


        for(readIndex = 0; readIndex < 48; readIndex++)
        {
            // array to save

            int tempP;
            CtoFBuffer[readIndex] = (*(volatile unsigned char *)(FRAM_ADDR + readIndex));
        }

    ByteToFloat();
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;


    // Interrupt enable for global FRAM memory protection
    GCCTL0 |= WPIE;


    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
    TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
    TB0CCTL1 = OUTMOD_7;                    // CCR1 reset/set
    TB0CCR1 = (MSP_CPU_CLK/5000);                          // CCR1 PWM duty cycle
    TB0CCTL3 = OUTMOD_7;                    // CCR1 reset/set
    TB0CCR3 = (MSP_CPU_CLK/5000);                          // CCR1 PWM duty cycle
    TB0CCTL4 = OUTMOD_7;                    // CCR1 reset/set
    TB0CCR4 = (MSP_CPU_CLK/5000);                          // CCR1 PWM duty cycle
    TB0CCTL5 = OUTMOD_7;                    // CCR1 reset/set
    TB0CCR5 = (MSP_CPU_CLK/5000);                          // CCR1 PWM duty cycle
    TB0CCTL6 = OUTMOD_7;                    // CCR1 reset/set
    TB0CCR6 = (MSP_CPU_CLK/5000);                          // CCR1 PWM duty cycle

    __enable_interrupt();

    // Relay GPIO HIGH -> Relay Off state
    // Relay GPIO LOW -> RElay ON state
    P2OUT |= BIT2;              // Relay GPIO HIGH
    //P2OUT &= ~BIT2;           // RElay GPIO LOW

    while (1)
    {
        // temperature controller normally operate mode
        // Connected Verify

        if(ADC_CalcurationFlag == 1)
        {
            ADC_Temp[0][sample_index] = ADC_Result[0];
            ADC_Temp[1][sample_index] = ADC_Result[1];
            ADC_Temp[2][sample_index] = ADC_Result[2];
            ADC_Temp[3][sample_index] = ADC_Result[3];

            sample_index++;

            if(sample_index == 5)
            {
                ADC_Sample[0] = (float)((ADC_Temp[0][0] + ADC_Temp[0][1] + ADC_Temp[0][2] + ADC_Temp[0][3] + ADC_Temp[0][4])/5);
                ADC_Sample[1] = (float)((ADC_Temp[1][0] + ADC_Temp[1][1] + ADC_Temp[1][2] + ADC_Temp[1][3] + ADC_Temp[1][4])/5);
                ADC_Sample[2] = (float)((ADC_Temp[2][0] + ADC_Temp[2][1] + ADC_Temp[2][2] + ADC_Temp[2][3] + ADC_Temp[2][4])/5);
                ADC_Sample[3] = (float)((ADC_Temp[3][0] + ADC_Temp[3][1] + ADC_Temp[3][2] + ADC_Temp[3][3] + ADC_Temp[3][4])/5);

                sample_index = 0;
                SampleComplete = 1;
            }

            ADC_CalcurationFlag = 0;
        }

        if(SampleComplete)
        {
            for(k =0; k<3; k++){
                voltageArray[k] = (0.000806)*ADC_Sample[k];

                TempVal[k] = ((voltageArray[k] + OFFSETVOLTAGE) / 0.06000) + 20.00;
            }
            roller_voltage = (0.000806)*ADC_Sample[3];

            TempVal[3] = ((roller_voltage - 0.0) / 0.0440) + 0.00;

            SampleComplete = 0;
        }

        // 1. recognize connection status and mode setting
        if(RxData == 0xa5)
        {
            if(BufferCount >= 12)
            {
                // PC command process
                BufferCount = 0;
                if(BufferChecking[0] == 0xfd && BufferChecking[11] == 0xa5)
                {
                    _commandZone(BufferChecking);
                }
            }
        }

        // 1. 전시기 명령어 처리 구간 / 튜닝 진입시 처리 안함
        if(RxData2 == 0xa5)
        {
            if(BufferCount2 >= 12)
            {
                // Displayer command process
                BufferCount2 = 0;
                if(BufferChecking2[0] == 0xfc && BufferChecking2[11] == 0xa5)
                {
                    _commandZone(BufferChecking2);
                }
            }
        }

        /*
        if(AT_NORMAL_FLAG == 1 || AT_START_FLAG == 1)
        {
            // 2. I2C sensor process
            if(TB_Sensor_Flag)
            {
                getData();                                  // period per

                //printf("LOW_BYTE %d\r\n", (uint8_t)i2cData[0]);
                //printf("HIGH BYTE %d\r\n", (uint8_t)i2cData[1]);
                //printf("PEC %d\r\n", (uint8_t)i2cData[2]);

                //PEC ������ ���� ������ ����
                BUF[0] = TBP_ADDR<<1;
                BUF[1] = 0x07;
                BUF[2] = (TBP_ADDR<<1) | 0x01;
                BUF[3] = (uint8_t)i2cData[0];
                BUF[4] = (uint8_t)i2cData[1];

                if ((uint8_t)i2cData[2] == CalPEC(BUF, 5))              // ������ ������ PEC ���� �´��� ����
                {
                  *dest_call = ((uint8_t)i2cData[1]<<8) | (uint8_t)i2cData[0];    // ������ ������ �µ� ������ ����
                  dest_call = &_rawObject;
                  PEC_ture = 1;
                }
                else
                {
                    PEC_ture = 0;
                }

                if(GetObject())              // ���µ� �� �����µ� Read �Ϸ�Ǹ�
                {
                    TempI2C =  CalcTemp(result_object);
                }

                TB_Sensor_Flag = 0;
            }

        }*/


        // common process heater ON/OFF
        if(AT_START_FLAG == 1)
        {
            if(SendGpioFlag[0])
            {
                // PWM Control Start
                // duty is PID result ( = range)
                // ---- Heater PWM Reverse ---- //
                SendGpioFlag[0] = 0;

                TB0CCR3 = ((float)( (MSP_CPU_CLK/5000) )/100.0)*((int)( (1-AT_pidResult[0]) * 100));                 // CCR3(CH1) PWM duty cycle
                TB0CTL &= ~(TBIFG);                 // Clear FLAG
            }

            if(SendGpioFlag[1])
            {
                // PWM Control Start
                // duty is PID result ( = range)
                // ---- Heater PWM Reverse ---- //
                SendGpioFlag[1] = 0;

                TB0CCR4 = ((float)( (MSP_CPU_CLK/5000) )/100.0)*((int)( (1-AT_pidResult[1]) * 100));                 // CCR4(CH2) PWM duty cycle
                TB0CTL &= ~(TBIFG);     // Clear FLAG
            }
            if(SendGpioFlag[2])
            {
                // PWM Control Start
                // duty is PID result ( = range)
                // ---- Heater PWM Reverse ---- //
                SendGpioFlag[2] = 0;

                TB0CCR5 = ((float)( (MSP_CPU_CLK/5000) )/100.0)*((int)( (1-AT_pidResult[2]) * 100));                 // CCR5(CH3) PWM duty cycle
                TB0CTL &= ~(TBIFG);     // Clear FLAG
            }
            if(SendGpioFlag[3])
            {
                // PWM Control Start
                // duty is PID result ( = range)
                // ---- Heater PWM Reverse ---- //
                SendGpioFlag[3] = 0;

                //TB0CCR6 = ((float)( (MSP_CPU_CLK/5000) )/100.0)*((int)( (1-AT_pidResult[3]) * 100));                 // CCR6(CH4) PWM duty cycle
                TB0CCR1 = ((float)( (MSP_CPU_CLK/5000) )/100.0)*((int)( (1-AT_pidResult[3]) * 100));                 // CCR1(CH5) PWM duty cycle
                TB0CTL &= ~(TBIFG);     // Clear FLAG
            }
        }

        if((DP_NORMAL_FLAG || AT_NORMAL_FLAG) && AT_START_FLAG != 1)
        {
            if(SendGpioFlag[0])
            {
                // PWM Control Start
                // duty is PID result ( = range)
                // ---- Heater PWM Reverse ---- //
                SendGpioFlag[0] = 0;

                TB0CCR3 = ((float)( (MSP_CPU_CLK/5000) )/100.0)*((int)( (1-pidResult[0]) * 100));                 // CCR3(CH1) PWM duty cycle
                TB0CTL &= ~(TBIFG);                 // Clear FLAG
            }

            if(SendGpioFlag[1])
            {
                // PWM Control Start
                // duty is PID result ( = range)
                // ---- Heater PWM Reverse ---- //
                SendGpioFlag[1] = 0;

                TB0CCR4 = ((float)( (MSP_CPU_CLK/5000) )/100.0)*((int)( (1-pidResult[1]) * 100));                 // CCR4(CH2) PWM duty cycle
                TB0CTL &= ~(TBIFG);     // Clear FLAG
            }
            if(SendGpioFlag[2])
            {
                // PWM Control Start
                // duty is PID result ( = range)
                // ---- Heater PWM Reverse ---- //
                SendGpioFlag[2] = 0;

                TB0CCR5 = ((float)( (MSP_CPU_CLK/5000) )/100.0)*((int)( (1-pidResult[2]) * 100));                 // CCR5(CH3) PWM duty cycle
                TB0CTL &= ~(TBIFG);     // Clear FLAG
            }
            if(SendGpioFlag[3])
            {
                // PWM Control Start
                // duty is PID result ( = range)
                // ---- Heater PWM Reverse ---- //
                SendGpioFlag[3] = 0;

                //TB0CCR6 = ((float)( (MSP_CPU_CLK/5000) )/100.0)*((int)( (1-pidResult[3]) * 100));                 // CCR6(CH4) PWM duty cycle
                TB0CCR1 = ((float)( (MSP_CPU_CLK/5000) )/100.0)*((int)( (1-pidResult[3]) * 100));                 // CCR1(CH5) PWM duty cycle
                TB0CTL &= ~(TBIFG);     // Clear FLAG
            }
        }

        if(AT_NORMAL_FLAG || AT_START_FLAG)
        {
            MessageTx();
        }

        if(DP_NORMAL_FLAG)
        {
            MessageTx2();
        }

        if(AT_NORMAL_FLAG == 1 && AT_START_FLAG != 0)
        {
            if(FLASH_FLAG == 1)
            {
                __disable_interrupt();
                FRAMWrite(FtoCBuffer);
                __enable_interrupt();

                FLASH_FLAG = 0;
            }
        }
    }
}

void Init_TIMER_A0(void)
{
    // Timer

    //TA0CCR0 = 8000;                         // 2KHz = 0.5ms
    TA0CCR0 = 1000;                           // Frequency -> (16Mhz/1600)/8 = 1250Hz
    //TA0CTL = TASSEL__SMCLK | MC__CONTINOUS; // SMCLK, continuous mode
    TA0CTL = TASSEL_2 | ID_3 | MC_1;          // SMCLK, continuous mode
    TA0CCTL0 = CCIE;                          // TACCR0 interrupt enabled
}

void Init_PWM_GPIO(void)
{
    // Configure GPIO
    //TimerB PWM GPIO

    P3DIR |= BIT4 | BIT5 | BIT6 | BIT7;                   // P3.4, P3.5, P3.6 and P3.7 output
    P3SEL0 |= BIT4 | BIT5 | BIT6 | BIT7;                  // P3.4, P3.5, P3.6 and P3.7 options select
    P3SEL1 &= ~(BIT4 | BIT5 | BIT6 | BIT7);

    P1DIR |= BIT4;                                  // P1.4 Channel 5
    P1SEL0 |= BIT4;                                 // P1.4 options select
    P1SEL1 &= ~(BIT4);

}

void Init_SMCLK_4MHZ(void)
{
    /////////////////// SMCLK = 16MHZ ////////////////////////////////
    P5DIR |= BIT5;
    P5SEL0 |= BIT5;                         // Output ACLK
    P5SEL1 |= BIT5;

    P5DIR |= BIT6;
    P5SEL1 |= BIT6;                         // Output SMCLK
    P5SEL0 |= BIT6;

    // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRCTL0 = FRCTLPW | NWAITS_1;

    // Clock System Setup
    CSCTL0_H = CSKEY_H;                     // Unlock CS registers
    CSCTL1 = DCOFSEL_0;                     // Set DCO to 1MHz
    // Set SMCLK = MCLK = DCO, ACLK = VLOCLK
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
    // Per Device Errata set divider to 4 before changing frequency to
    // prevent out of spec operation from overshoot transient
    CSCTL3 = DIVA__4 | DIVS__4 | DIVM__4;   // Set all corresponding clk sources to divide by 4 for errata
    CSCTL1 = DCOFSEL_4 | DCORSEL;           // Set DCO to 16MHz
    // Delay by ~10us to let DCO settle. 60 cycles = 20 cycles buffer + (10us / (1/4MHz))
    __delay_cycles(60);
    //CSCTL3 = DIVA__1 | DIVS__4 | DIVM__1;   // Set all dividers to 1 for 16MHz operation
    CSCTL3 = DIVA__1 | DIVS__4 | DIVM__1;   // Set all dividers to 1 for 16MHz operation
    CSCTL0_H = 0;                            // Lock CS registers                      // Lock CS registers
    /////////////////// SMCLK = 16MHZ ////////////////////////////////
    ////////////////// SMCLK -> 4MHz  ////////////////////////////////
}

void _commandZone(unsigned char *_newCommand)
{
    // Displayer - normal mode
    if(_newCommand[0] == 0xfc && _newCommand[1] == 0xB0 &&_newCommand[11] == 0xa5 && DP_reQuestFlag == 0)
    {
        // Receive Configuration Temperature and Heater ON/OFF Status (DP 3.1)

        SaveOnOffState = _newCommand[10];

        if(((_newCommand[2]<<8)+_newCommand[3]) <= 3000)
        //if(((_newCommand[2]<<8)+_newCommand[3]) <= 650)
        {
            //CH1 Temperature
            ConfigTemp[0] = ((float)((_newCommand[2]<<8)+_newCommand[3]))/10.0;
        }
        if(((_newCommand[4]<<8)+_newCommand[5]) <= 3000)
        //if(((_newCommand[4]<<8)+_newCommand[5]) <= 650)
        {
            //CH2 Temperature
            ConfigTemp[1] = ((float)((_newCommand[4]<<8)+_newCommand[5]))/10.0;
        }
        if(((_newCommand[6]<<8)+_newCommand[7]) <= 3000)
        //if(((_newCommand[6]<<8)+_newCommand[7]) <= 650)
        {
            //CH3 Temperature
            ConfigTemp[2] = ((float)((_newCommand[6]<<8)+_newCommand[7]))/10.0;
        }
        if(((_newCommand[8]<<8)+_newCommand[9]) <= 3000)
        //if(((_newCommand[8]<<8)+_newCommand[9]) <= 65.0)
        {
            //CH4 Temperature
            ConfigTemp[3] = ((float)((_newCommand[8]<<8)+_newCommand[9]))/10.0;
        }


        // Heater On/Off
        if( (SaveOnOffState & (0x1 << 7 )) >= 1)
        {
            PID_Flag[0] = 1;
        }
        else
        {
            PID_Flag[0] = 0;
        }
        if( (SaveOnOffState & (0x1 << 6 )) >= 1)
        {
            PID_Flag[1] = 1;
        }
        else
        {
            PID_Flag[1] = 0;
        }
        if( (SaveOnOffState & (0x1 << 5 )) >= 1)
        {
            PID_Flag[2] = 1;
        }
        else
        {
            PID_Flag[2] = 0;
        }
        if( (SaveOnOffState & (0x1 << 4 )) >= 1)
        {
            PID_Flag[3] = 1;
        }
        else
        {
            PID_Flag[3] = 0;
        }
        if( (SaveOnOffState & (0x1 << 3 )) >= 1)
        {
            ADC_CH1_Controller = 0;
            I2C_Sensor = 1;
        }
        else
        {
            ADC_CH1_Controller = 1;
            I2C_Sensor = 0;
        }
        AT_NORMAL_FLAG = 0;
        AT_STOP_FLAG = 0;
        AT_START_FLAG = 0;
    }


    // PC - normal mode
    if(_newCommand[0] == 0xfd && _newCommand[1] == 0x80 &&_newCommand[11] == 0xa5 && PC_reQuestFlag == 0)
    {

        // 1byte shift
        // receive Gain setting command (PC 3.2)
        if(_newCommand[10] == 0x07)
        {
            unsigned char ChannelTemp = _newCommand[6];     // ChannelTemp = Channel - 1

            // it is command that set Gain (���̺��ϰ� ���� ���� ���� �Ȱ� �ѹ� ����)
            if(_newCommand[7] == 0x00)
            {
                float FloatP_Gain;
                // P- Gain Set
                // P- Gain Buffer (Channel Select)

                // test RealTerm Test
                P_CastingBuffer[ChannelTemp][0] = _newCommand[5];
                P_CastingBuffer[ChannelTemp][1] = _newCommand[4];
                P_CastingBuffer[ChannelTemp][2] = _newCommand[3];
                P_CastingBuffer[ChannelTemp][3] = _newCommand[2];

                memcpy(&FloatP_Gain, P_CastingBuffer[ChannelTemp], sizeof(float));              // unsisgned char  ->  Float
                Kp_Value[ChannelTemp] = FloatP_Gain;

                // Flash Memory Write
                FloatToByte();

                //FRAMWrite(FtoCBuffer);
                FLASH_FLAG = 1;

                Tx_PID_Tuning(1, ChannelTemp);
            }
            else if(_newCommand[7] == 0x01)
            {
                // I-Gain Set
                float FloatI_Gain;
                // I- Gain Set
                // I- Gain Buffer (Channel Select)
                // test RealTerm Test
                I_CastingBuffer[ChannelTemp][0] = _newCommand[5];
                I_CastingBuffer[ChannelTemp][1] = _newCommand[4];
                I_CastingBuffer[ChannelTemp][2] = _newCommand[3];
                I_CastingBuffer[ChannelTemp][3] = _newCommand[2];

                memcpy(&FloatI_Gain, I_CastingBuffer[ChannelTemp], sizeof(float));

                Ki_Value[ChannelTemp] = FloatI_Gain;

                // Flash Memory Write
                FloatToByte();
                FLASH_FLAG = 1;

                //FRAMWrite(FtoCBuffer);

                Tx_PID_Tuning(1, ChannelTemp);

            }
            else if(_newCommand[7] == 0x02)
            {
                // D-Gain Set
                //unsigned char GainTemp[4];
                float FloatD_Gain;
                // D- Gain Set
                // D- Gain Buffer (Channel Select)
                // test RealTerm Test
                D_CastingBuffer[ChannelTemp][0] = _newCommand[5];
                D_CastingBuffer[ChannelTemp][1] = _newCommand[4];
                D_CastingBuffer[ChannelTemp][2] = _newCommand[3];
                D_CastingBuffer[ChannelTemp][3] = _newCommand[2];

                memcpy(&FloatD_Gain, D_CastingBuffer[ChannelTemp], sizeof(float));
                Kd_Value[ChannelTemp] = FloatD_Gain;

                // Flash Memory Write
                FloatToByte();

                //FRAMWrite(FtoCBuffer);

                FLASH_FLAG = 1;

                Tx_PID_Tuning(1, ChannelTemp);

            }
        }
        else if(_newCommand[10] == 0x05)
        {
            unsigned char ChannelTemp = _newCommand[6];
            // ��� ä���� ���� ���¸� �ѹ��� ����

            memcpy(P_CastingBuffer[0], &Kp_Value[0], sizeof(float));        //  float -> unsigned char  P-Gain Channel 1
            memcpy(P_CastingBuffer[1], &Kp_Value[1], sizeof(float));
            memcpy(P_CastingBuffer[2], &Kp_Value[2], sizeof(float));
            memcpy(P_CastingBuffer[3], &Kp_Value[3], sizeof(float));

            memcpy(I_CastingBuffer[0], &Ki_Value[0], sizeof(float));        //  float -> unsigned char  P-Gain Channel 1
            memcpy(I_CastingBuffer[1], &Ki_Value[1], sizeof(float));
            memcpy(I_CastingBuffer[2], &Ki_Value[2], sizeof(float));
            memcpy(I_CastingBuffer[3], &Ki_Value[3], sizeof(float));


            memcpy(D_CastingBuffer[0], &Kd_Value[0], sizeof(float));        //  float -> unsigned char  P-Gain Channel 1
            memcpy(D_CastingBuffer[1], &Kd_Value[1], sizeof(float));
            memcpy(D_CastingBuffer[2], &Kd_Value[2], sizeof(float));
            memcpy(D_CastingBuffer[3], &Kd_Value[3], sizeof(float));

            Tx_PID_Tuning(2, ChannelTemp);
        }
        else if(_newCommand[10] == 0x02)
        {
            unsigned char ChannelTemp = _newCommand[6];

            memcpy(P_CastingBuffer[ChannelTemp], &Kp_Value[ChannelTemp], sizeof(float));        //  float -> unsigned char  P-Gain Channel 1

            memcpy(I_CastingBuffer[ChannelTemp], &Ki_Value[ChannelTemp], sizeof(float));        //  float -> unsigned char  P-Gain Channel 1

            memcpy(D_CastingBuffer[ChannelTemp], &Kd_Value[ChannelTemp], sizeof(float));        //  float -> unsigned char  P-Gain Channel 1

            // ä���� �о �� ä���� ���¸� ����
            Tx_PID_Tuning(3, ChannelTemp);
        }
        else
        {
            // Configuration target Temperature (PC 2.2)

            SaveOnOffState = _newCommand[10];

            if(((_newCommand[2]<<8)+_newCommand[3]) <= 3000)
            {
                //CH1 Temperature
                ConfigTemp[0] = ((float)((_newCommand[2]<<8)+_newCommand[3]))/10.0;
            }
            if(((_newCommand[4]<<8)+_newCommand[5]) <= 3000)
            {
                //CH2 Temperature
                ConfigTemp[1] = ((float)((_newCommand[4]<<8)+_newCommand[5]))/10.0;
            }
            if(((_newCommand[6]<<8)+_newCommand[7]) <= 3000)
            {
                //CH3 Temperature
                ConfigTemp[2] = ((float)((_newCommand[6]<<8)+_newCommand[7]))/10.0;
            }
            if(((_newCommand[8]<<8)+_newCommand[9]) <= 3000)
            {
                //CH4 Temperature
                ConfigTemp[3] = ((float)((_newCommand[8]<<8)+_newCommand[9]))/10.0;
            }


            // Heater On/Off
            if( (SaveOnOffState & (0x1 << 7 )) >= 1)
            {
                PID_Flag[0] = 1;
            }
            else
            {
                PID_Flag[0] = 0;
            }
            if( (SaveOnOffState & (0x1 << 6 )) >= 1)
            {
                PID_Flag[1] = 1;
            }
            else
            {
                PID_Flag[1] = 0;
            }
            if( (SaveOnOffState & (0x1 << 5 )) >= 1)
            {
                PID_Flag[2] = 1;
            }
            else
            {
                PID_Flag[2] = 0;
            }
            if( (SaveOnOffState & (0x1 << 4 )) >= 1)
            {
                PID_Flag[3] = 1;
            }
            else
            {
                PID_Flag[3] = 0;
            }
            if( (SaveOnOffState & (0x1 << 3 )) >= 1)
            {
                ADC_CH1_Controller = 0;
                I2C_Sensor = 1;
            }
            else
            {
                ADC_CH1_Controller = 1;
                I2C_Sensor = 0;
            }
        }

        AT_NORMAL_FLAG = 1;
        AT_STOP_FLAG = 0;
        AT_START_FLAG = 0;
    }
    else if(_newCommand[0] == 0xfd && _newCommand[1] == 0x82 &&_newCommand[11] == 0xa5 && TuningStartFlag == 1)
    {
        // AT-mode Start
        // receive P-I-D result, channel (PC 4.3)

        unsigned char ChannelTemp = _newCommand[10];
        if(ChannelTemp == 0x00)
        {
            // channel 1
            Selected_Channel = 0x80;
        }
        else if(ChannelTemp == 0x01)
        {
            // channel 2
            Selected_Channel = 0x40;
        }
        else if(ChannelTemp == 0x02)
        {
            // channel 3
            Selected_Channel = 0x20;
        }
        else if(ChannelTemp == 0x03)
        {
            // channel 4
            Selected_Channel = 0x10;
        }


        //float AT_curretTemp;

        unsigned char pidNgain = _newCommand[9];
        SaveOnOffState = Selected_Channel;
        // Tuning START
        /*
        if(((_newCommand[2]<<8)+_newCommand[3]) <= 3000)
        {
            //CH1 Temperature
            ConfigTemp[ChannelTemp] = ((float)((_newCommand[2]<<8)+_newCommand[3]))/10.0;
        }
        */
        if(pidNgain == 0x00)
        {
            // pidResult
            float Floatpid_Gain;
            // D- Gain Set
            // D- Gain Buffer (Channel Select)
            // test RealTerm Test
            pid_CastingBuffer[ChannelTemp][0] = _newCommand[5];
            pid_CastingBuffer[ChannelTemp][1] = _newCommand[4];
            pid_CastingBuffer[ChannelTemp][2] = _newCommand[3];
            pid_CastingBuffer[ChannelTemp][3] = _newCommand[2];

            memcpy(&Floatpid_Gain, pid_CastingBuffer[ChannelTemp], sizeof(float));
            AT_pidResult[ChannelTemp] = Floatpid_Gain;
        }

        // Heater On/Off
        if( (SaveOnOffState & (0x1 << 7 )) >= 1)
        {
            PID_Flag[0] = 1;
        }
        else
        {
            PID_Flag[0] = 0;
        }
        if( (SaveOnOffState & (0x1 << 6 )) >= 1)
        {
            PID_Flag[1] = 1;
        }
        else
        {
            PID_Flag[1] = 0;
        }
        if( (SaveOnOffState & (0x1 << 5 )) >= 1)
        {
            PID_Flag[2] = 1;
        }
        else
        {
            PID_Flag[2] = 0;
        }
        if( (SaveOnOffState & (0x1 << 4 )) >= 1)
        {
            PID_Flag[3] = 1;
        }
        else
        {
            PID_Flag[3] = 0;
        }

        AT_NORMAL_FLAG = 0;
        AT_STOP_FLAG = 0;
        AT_START_FLAG = 1;
    }
    else if(_newCommand[0] == 0xfd && _newCommand[1] == 0x82 &&_newCommand[11] == 0xa5 && TuningStartFlag == 0)
    {
        // response Tuning Start (PC 4.1)
        TuningStartFlag = 1;
        Selected_Channel = _newCommand[10];
        AT_NORMAL_FLAG = 0;
        AT_STOP_FLAG = 0;
        AT_START_FLAG = 1;

        //TuningStopFlag = 0;
        MessageTx();            // Feedback Message (Start Tuning mode)
    }
    else if(_newCommand[0] == 0xfd && _newCommand[1] == 0x81 &&_newCommand[11] == 0xa5 && TuningStartFlag == 1)
    {
        // Auto Tuning Stop (PC 4.4)
        // pid-Gain
        // Send PID Stop Signal
        Selected_Channel = 0x00;
        SaveOnOffState = 0x00;

        TuningStartFlag = 0;
        TuningStopFlag = 1;

        AT_START_FLAG = 0;
        AT_STOP_FLAG = 1;
        // Send Stop feedback

        MessageTx();        // Feedback Message

        // return Normal Mode
        AT_NORMAL_FLAG = 1;
        AT_STOP_FLAG = 0;

        TuningStartFlag = 0;
    }


    // With PC
    //////////////////////  read: 12 byte ////////////////////////

    if(_newCommand[0] == 0xfd && _newCommand[1] == 0xC0 && _newCommand[11] == 0xa5 && PC_reQuestFlag == 1)
    {
        // Start Transmit (PC 1.3)
        PC_reQuestFlag = 0;
        AT_NORMAL_FLAG = 1;
    }

    if(_newCommand[0] == 0xfd && _newCommand[1] == 0xA0 && _newCommand[11] == 0xa5 && PC_reQuestFlag == 0)
    {
        // Verify connect request (PC 1.1)
        PC_reQuestFlag = 1;

        MessageTx();    // Feedback Message
    }


    // With Displayer

    if(_newCommand[0] == 0xfc && _newCommand[1] == 0x90 && _newCommand[11] == 0xa5 && DP_reQuestFlag == 1 )
    {
        // Start Transmit (DP 1.3)

        DP_reQuestFlag = 0;
        DP_NORMAL_FLAG = 1;
    }

    if(_newCommand[0] == 0xfc && _newCommand[1] == 0x70 && _newCommand[11] == 0xa5 && (DP_reQuestFlag == 0 || rxReFlag == 0))
    {
        // Verify connect request (DP 1.1)
        rxReFlag = 1;
        DP_reQuestFlag = 1;

        MessageTx2();    // Feedback Message
    }

}

// Timer0_A0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer0_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    timeCount_PID += 1;
    timeSamplingCount += 1;
    if(timeSamplingCount >= 10)         // 0.02sec       51ms
    {
        //Sampling until 0.1sec
        // ADC sampling

        // multi 4 channel
        ADC_Result[0] = AnalogRead(ADC12INCH_12);
        ADC_Result[1] = AnalogRead(ADC12INCH_13);
        ADC_Result[2] = AnalogRead(ADC12INCH_14);
        //ADC_Result[3] = AnalogRead(ADC12INCH_15);
        ADC_Result[3] = AnalogRead(ADC12INCH_8);

        // use one adc channel in roller

        //ADC_Result[0] = AnalogRead(ADC12INCH_8);
        //ADC_Result[1] = AnalogRead(ADC12INCH_9);
        //ADC_Result[2] = AnalogRead(ADC12INCH_10);
        //ADC_Result[3] = AnalogRead(ADC12INCH_11);

        ADC_CalcurationFlag = 1;
        timeSamplingCount = 0;
    }

    if(timeCount_PID >= 50)            // 125 = 0.1s                25 = 178.6ms         5 = 51ms    10 = 83ms   15   83
    {
        TB_Sensor_Flag = 1;

        if(TempVal[0] < 0)
        {
            TempVal[0] = 0;
        }
        else if(TempVal[0] > 300)
        {
            TempVal[0] = 300;
        }
        if(TempVal[1] < 0)
        {
            TempVal[1] = 0;
        }
        else if(TempVal[1] > 300)
        {
            TempVal[1] = 300;
        }
        if(TempVal[2] < 0)
        {
            TempVal[2] = 0;
        }
        else if(TempVal[2] > 300)
        {
            TempVal[2] = 300;
        }
        if(TempVal[3] < 0)
        {
            TempVal[3] = 0;
        }
        else if(TempVal[3] > 300)
        {
            TempVal[3] = 300;
        }

        // ch1 byte split
        tempIntVal_ch1 = ((floor(TempVal[0]*10)/10) *10);
        targetIntVal_ch1 = (ConfigTemp[0]*10);


        tempIntVal_ch2 = ((floor(TempVal[1]*10)/10) *10);
        targetIntVal_ch2 = (ConfigTemp[1]*10);

        // ch3 byte split
        tempIntVal_ch3 = ((floor(TempVal[2]*10)/10) *10);
        targetIntVal_ch3 = (ConfigTemp[2]*10);

        // ch4 byte split
        tempIntVal_ch4 = ((floor(TempVal[3]*10)/10) *10);
        targetIntVal_ch4 = (ConfigTemp[3]*10);


        // I2C byte split
        tempIntVal_i2c = ((floor(TempI2C*10)/10) *10);
        //*p_I2C_temp = (char *)&tempIntVal_i2c;

        if(AT_START_FLAG == 1)
        {
            AT_currentTemp = ((floor(TempVal[Selected_Channel] *10) /10) *10);
            //AT_targetTemp = (ConfigTemp[Selected_Channel]*10);
        }

        PC_SendMessageFlag = 1;
        DP_SendMessageFlag = 1;

        if(PID_Flag[0])
        {

             if(ADC_CH1_Controller == 1)
             {
                 pidResult[0] = PID_Contorller(Kp_Value[0], Ki_Value[0], Kd_Value[0], ConfigTemp[0], TempVal[0]);
                 //pidResult[0] = PID_Contorller(Kp_Value[0], Ki_Value[0], Kd_Value[0], ConfigTemp[0], 44.8);
                 SendGpioFlag[0] = 1;
             }
             if(I2C_Sensor == 1)
             {
                 pidResult[0] = PID_Contorller(Kp_Value[0], Ki_Value[0], Kd_Value[0], ConfigTemp[0], TempI2C);
                 SendGpioFlag[0] = 1;
             }
        }
        else
        {
            // Heater CH1 Off
            pidResult[0] = 0;
            TB0CCR3 = (MSP_CPU_CLK/5000);                          // CCR3 PWM duty cycle
            TB0CTL &= ~(TBIFG);
        }

        if(PID_Flag[1])
        {
             pidResult[1] = PID_Contorller(Kp_Value[1], Ki_Value[1], Kd_Value[1], ConfigTemp[1], TempVal[1]);
             SendGpioFlag[1] = 1;
        }
        else
        {
            // Heater CH2 Off
            pidResult[1] = 0;
            TB0CCR4 = (MSP_CPU_CLK/5000);                          // CCR4 PWM full duty cycle
            TB0CTL &= ~(TBIFG);
        }

        if(PID_Flag[2])
        {
             pidResult[2] = PID_Contorller(Kp_Value[2], Ki_Value[2], Kd_Value[2], ConfigTemp[2], TempVal[2]);
             SendGpioFlag[2] = 1;
        }
        else
        {
            // Heater CH3 Off
            pidResult[2] = 0;
            TB0CCR5 = (MSP_CPU_CLK/5000);                          // CCR5 PWM duty cycle
            TB0CTL &= ~(TBIFG);
        }

        if(PID_Flag[3])
        {
            pidResult[3] = PID_Contorller(Kp_Value[3], Ki_Value[3], Kd_Value[3], ConfigTemp[3], TempVal[3]);
            SendGpioFlag[3] = 1;
        }
        else
        {
            // Heater CH4, CH5 Off
            pidResult[3] = 0;
            TB0CCR6 = (MSP_CPU_CLK/5000);                          // CCR6 PWM duty cycle
            TB0CCR1 = (MSP_CPU_CLK/5000);                          // CCR1 PWM duty cycle
            TB0CTL &= ~(TBIFG);
        }

        timeCount_PID = 0;
    }

}

float PID_Contorller(float kp, float ki, float kd, float targetValue, float readValue)          // PID Calculator
{
    static float PID_Value, pidP, pidI, pidD, pidError, preError;

    pidError = targetValue - readValue;

    pidP = kp * pidError;
    pidI = ki * (pidI + pidError);
    pidD = kd * ( (pidError - preError));

    PID_Value = pidP + pidI + pidD;

    if(PID_Value <= 0)
    {
        PID_Value = 0;
    }
    if(PID_Value > 1)
    {
        PID_Value = 1;
    }

    preError = pidError;
    return PID_Value;
}

void MessageTx(void)                    // Send PC Command
{
    // if connection request is received from PC, Controller Send Feedback (PC 1.2)
    if(PC_SendMessageFlag && PC_reQuestFlag)
    {
        fput_data(0xfe);

        fput_data(0xB0);

        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);

        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);

        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);

        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);

        fput_data(0xa5);

        PC_SendMessageFlag = 0;

    }

    // Send message about current Temperature and target temperature (PC 1.4 / 2.1 / 3.1)
    if(PC_SendMessageFlag == 1 && AT_NORMAL_FLAG == 1)
    {

        if( ((tempIntVal_ch1 & (0xFF << 8)) >> 8)  < 0x03 && ((tempIntVal_ch2 & (0xFF << 8)) >> 8)  < 0x03
                && ((tempIntVal_ch3 & (0xFF << 8)) >> 8)  < 0x03 && ((tempIntVal_ch4 & (0xFF << 8)) >> 8)  < 0x03
                && ((targetIntVal_ch1 & (0xFF << 8)) >> 8) < 0x03 && ((targetIntVal_ch2 & (0xFF << 8)) >> 8) < 0x03
                && ((targetIntVal_ch3 & (0xFF << 8)) >> 8) < 0x03 && ((targetIntVal_ch4 & (0xFF << 8)) >> 8) < 0x03
        )
        {

            fput_data(0xfe);

            fput_data(0x90);

            fput_data( (tempIntVal_ch1 & (0xFF << 8)) >> 8 );
            fput_data( (tempIntVal_ch1 & (0xFF << 0)) >> 0 );

            fput_data( (targetIntVal_ch1 & (0xFF << 8)) >> 8 );
            fput_data( (targetIntVal_ch1 & (0xFF << 0)) >> 0 );

            fput_data( (tempIntVal_ch2 & (0xFF << 8)) >> 8 );
            fput_data( (tempIntVal_ch2 & (0xFF << 0)) >> 0 );

            fput_data( (targetIntVal_ch2 & (0xFF << 8)) >> 8 );
            fput_data( (targetIntVal_ch2 & (0xFF << 0)) >> 0 );

            fput_data( (tempIntVal_ch3 & (0xFF << 8)) >> 8 );
            fput_data( (tempIntVal_ch3 & (0xFF << 0)) >> 0 );

            fput_data( (targetIntVal_ch3 & (0xFF << 8)) >> 8 );
            fput_data( (targetIntVal_ch3 & (0xFF << 0)) >> 0 );

            fput_data( (tempIntVal_ch4 & (0xFF << 8)) >> 8 );
            fput_data( (tempIntVal_ch4 & (0xFF << 0)) >> 0 );

            fput_data( (targetIntVal_ch4 & (0xFF << 8)) >> 8 );
            fput_data( (targetIntVal_ch4 & (0xFF << 0)) >> 0 );

            fput_data( (tempIntVal_i2c & (0xFF << 8)) >> 8 );
            fput_data( (tempIntVal_i2c & (0xFF << 0)) >> 0 );

            fput_data( SaveOnOffState );

            fput_data( 0xa5 );

            PC_SendMessageFlag = 0;
        }
    }

    // Send message about current Temperature in Tuning Start (4.2)
    if(PC_SendMessageFlag && (AT_START_FLAG))
    {
        if( ((AT_currentTemp & (0xFF << 8)) >> 8)  < 0x03 )
        {
            fput_data(0xfe);                                            // 1

            fput_data(0x92);                                            // 2

            fput_data( (AT_currentTemp & (0xFF << 8)) >> 8 );            // 3
            fput_data( (AT_currentTemp & (0xFF << 0)) >> 0 );            // 4

            fput_data(0x00);                                            // 5
            fput_data(0x00);                                            // 6
            fput_data(0x00);                                            // 7
            fput_data(0x00);                                            // 8
            fput_data(0x00);                                            // 9
            fput_data(0x00);                                            // 10

            fput_data(0x00);                                            // 11
            fput_data(0x00);                                            // 12
            fput_data(0x00);                                            // 13
            fput_data(0x00);                                            // 14
            fput_data(0x00);                                            // 15

            fput_data(0x00);                                            // 16
            fput_data(0x00);                                            // 17
            fput_data(0x00);                                            // 18
            fput_data(0x00);                                            // 19
            fput_data(0x00);                                            // 20

            fput_data(Selected_Channel);                                  // 21

            fput_data( 0xa5 );                                          // 22

            PC_SendMessageFlag = 0;
        }
    }

    // Send message about Tuning Stop (4.5)
    if(PC_SendMessageFlag && AT_STOP_FLAG)
    {
        fput_data(0xfe);                                            // 1

        fput_data(0x91);                                            // 2

        fput_data(0x00);                                            // 3
        fput_data(0x00);                                            // 4

        fput_data(0x00);                                            // 5
        fput_data(0x00);                                            // 6
        fput_data(0x00);                                            // 7
        fput_data(0x00);                                            // 8
        fput_data(0x00);                                            // 9
        fput_data(0x00);                                            // 10

        fput_data(0x00);                                            // 11
        fput_data(0x00);                                            // 12
        fput_data(0x00);                                            // 13
        fput_data(0x00);                                            // 14
        fput_data(0x00);                                            // 15

        fput_data(0x00);                                            // 16
        fput_data(0x00);                                            // 17
        fput_data(0x00);                                            // 18
        fput_data(0x00);                                            // 19
        fput_data(0x00);                                            // 20

        //fput_data2( SaveOnOffState );

        fput_data(Selected_Channel);                                  // 21

        fput_data( 0xa5 );                                          // 22

        PC_SendMessageFlag = 0;

    }
}

void MessageTx2(void)           // Send Display Command
{
    // if connection request is received from Display, Controller Send Feedback (DP 1.2)
    if(DP_SendMessageFlag && (DP_reQuestFlag || rxReFlag))
    {
        rxReFlag = 0;
        fput_data2(0xfb);

        fput_data2(0x80);

        fput_data2(0x00);
        fput_data2(0x00);

        fput_data2( (targetIntVal_ch1 & (0xFF << 8)) >> 8 );
        fput_data2( (targetIntVal_ch1 & (0xFF << 0)) >> 0 );

        fput_data2(0x00);
        fput_data2(0x00);
        fput_data2(0x00);
        fput_data2(0x00);

        fput_data2(0x00);
        fput_data2(0x00);
        fput_data2(0x00);
        fput_data2(0x00);
        fput_data2(0x00);

        fput_data2(0x00);
        fput_data2(0x00);
        fput_data2(0x00);
        fput_data2(0x00);

        fput_data2(0xa5);

        DP_SendMessageFlag = 0;
    }

    // Send message about current Temperature and target temperature (DP 2.1 / 3.2)
    if(DP_SendMessageFlag == 1 && DP_NORMAL_FLAG == 1)
    {

        if( ((tempIntVal_ch1 & (0xFF << 8)) >> 8)  < 0x03 && ((tempIntVal_ch2 & (0xFF << 8)) >> 8)  < 0x03
                && ((tempIntVal_ch3 & (0xFF << 8)) >> 8)  < 0x03 && ((tempIntVal_ch4 & (0xFF << 8)) >> 8)  < 0x03
                && ((targetIntVal_ch1 & (0xFF << 8)) >> 8) < 0x03 && ((targetIntVal_ch2 & (0xFF << 8)) >> 8) < 0x03
                && ((targetIntVal_ch3 & (0xFF << 8)) >> 8) < 0x03 && ((targetIntVal_ch4 & (0xFF << 8)) >> 8) < 0x03
        )
        {
            fput_data2(0xfb);

            fput_data2(0xA0);

            fput_data2( (tempIntVal_ch1 & (0xFF << 8)) >> 8 );
            fput_data2( (tempIntVal_ch1 & (0xFF << 0)) >> 0 );

            fput_data2( (targetIntVal_ch1 & (0xFF << 8)) >> 8 );
            fput_data2( (targetIntVal_ch1 & (0xFF << 0)) >> 0 );

            fput_data2( (tempIntVal_ch2 & (0xFF << 8)) >> 8 );
            fput_data2( (tempIntVal_ch2 & (0xFF << 0)) >> 0 );

            fput_data2( (targetIntVal_ch2 & (0xFF << 8)) >> 8 );
            fput_data2( (targetIntVal_ch2 & (0xFF << 0)) >> 0 );

            fput_data2( (tempIntVal_ch3 & (0xFF << 8)) >> 8 );
            fput_data2( (tempIntVal_ch3 & (0xFF << 0)) >> 0 );

            fput_data2( (targetIntVal_ch3 & (0xFF << 8)) >> 8 );
            fput_data2( (targetIntVal_ch3 & (0xFF << 0)) >> 0 );

            fput_data2( (tempIntVal_ch4 & (0xFF << 8)) >> 8 );
            fput_data2( (tempIntVal_ch4 & (0xFF << 0)) >> 0 );

            fput_data2( (targetIntVal_ch4 & (0xFF << 8)) >> 8 );
            fput_data2( (targetIntVal_ch4 & (0xFF << 0)) >> 0 );

            fput_data2( SaveOnOffState );

            fput_data2( 0xa5 );

            DP_SendMessageFlag = 0;

        }
    }
}

void FRAMWrite(unsigned char *inputData)    // write FRAM
{

    unsigned int i = 0;

    for (i = 0; i < WRITE_SIZE; i++)
    {
        FRAM_write[i] = inputData[i];
    }

}

// PID Tuning Transmission Command
void Tx_PID_Tuning(unsigned char commandTx, unsigned char SelectedChannel)
{
    unsigned char tIndex = 0;
    if(commandTx == 1)
    {
        // save to return one cycle

        fput_data(0xfe);

        fput_data(0x90);

        fput_data( P_CastingBuffer[SelectedChannel][3] );     // P-Gain
        fput_data( P_CastingBuffer[SelectedChannel][2] );     // P-Gain
        fput_data( P_CastingBuffer[SelectedChannel][1] );     // P-Gain
        fput_data( P_CastingBuffer[SelectedChannel][0] );     // P-Gain

        fput_data( I_CastingBuffer[SelectedChannel][3] );     // I-Gain
        fput_data( I_CastingBuffer[SelectedChannel][2] );     // I-Gain
        fput_data( I_CastingBuffer[SelectedChannel][1] );     // I-Gain
        fput_data( I_CastingBuffer[SelectedChannel][0] );     // I-Gain

        fput_data( D_CastingBuffer[SelectedChannel][3] );     // D-Gain
        fput_data( D_CastingBuffer[SelectedChannel][2] );     // D-Gain
        fput_data( D_CastingBuffer[SelectedChannel][1] );     // D-Gain
        fput_data( D_CastingBuffer[SelectedChannel][0] );     // D-Gain

        fput_data(SelectedChannel);     // Channel      0~3 (4-channels)

        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);

        fput_data(0x07);        // block command

        fput_data(0xa5);
    }
    if(commandTx == 2)
    {
        // save to return four cycle (all channels)
        for(tIndex = 0; tIndex < 4; tIndex++)
        {
            fput_data(0xfe);

            fput_data(0x90);

            fput_data( P_CastingBuffer[tIndex][3] );     // P-Gain
            fput_data( P_CastingBuffer[tIndex][2] );     // P-Gain
            fput_data( P_CastingBuffer[tIndex][1] );     // P-Gain
            fput_data( P_CastingBuffer[tIndex][0] );     // P-Gain

            fput_data( I_CastingBuffer[tIndex][3] );     // I-Gain
            fput_data( I_CastingBuffer[tIndex][2] );     // I-Gain
            fput_data( I_CastingBuffer[tIndex][1] );     // I-Gain
            fput_data( I_CastingBuffer[tIndex][0] );     // I-Gain

            fput_data( D_CastingBuffer[tIndex][3] );     // D-Gain
            fput_data( D_CastingBuffer[tIndex][2] );     // D-Gain
            fput_data( D_CastingBuffer[tIndex][1] );     // D-Gain
            fput_data( D_CastingBuffer[tIndex][0] );     // D-Gain

            fput_data(tIndex);     // Channel      0~3 (4-channels)

            fput_data(0x00);
            fput_data(0x00);
            fput_data(0x00);
            fput_data(0x00);
            fput_data(0x00);

            fput_data(0x07);        // block command

            fput_data(0xa5);
        }
    }
    if(commandTx == 3)
    {
        // save to return one cycle about SelectedChannel

        fput_data(0xfe);

        fput_data(0x90);

        fput_data( P_CastingBuffer[SelectedChannel][3] );     // P-Gain
        fput_data( P_CastingBuffer[SelectedChannel][2] );     // P-Gain
        fput_data( P_CastingBuffer[SelectedChannel][1] );     // P-Gain
        fput_data( P_CastingBuffer[SelectedChannel][0] );     // P-Gain

        fput_data( I_CastingBuffer[SelectedChannel][3] );     // I-Gain
        fput_data( I_CastingBuffer[SelectedChannel][2] );     // I-Gain
        fput_data( I_CastingBuffer[SelectedChannel][1] );     // I-Gain
        fput_data( I_CastingBuffer[SelectedChannel][0] );     // I-Gain

        fput_data( D_CastingBuffer[SelectedChannel][3] );     // D-Gain
        fput_data( D_CastingBuffer[SelectedChannel][2] );     // D-Gain
        fput_data( D_CastingBuffer[SelectedChannel][1] );     // D-Gain
        fput_data( D_CastingBuffer[SelectedChannel][0] );     // D-Gain

        fput_data(SelectedChannel);     // Channel      0~3 (4-channels)

        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);

        fput_data(0x07);        // block command

        fput_data(0xa5);
    }

}

// Change Float to Byte
void FloatToByte(void)
{

    // from Writing to save
    unsigned char TempBuffer[48];       // Float Type data size 4*12

    unsigned char P_TempBuffer[4][4];
    unsigned char I_TempBuffer[4][4];
    unsigned char D_TempBuffer[4][4];

    unsigned int jIndex = 0;
    unsigned int mIndex = 0;
    float Kp_Temp[4];
    float Ki_Temp[4];
    float Kd_Temp[4];

    for(jIndex = 0; jIndex < 4; jIndex++)
    {
        Kp_Temp[jIndex] = Kp_Value[jIndex];
        Ki_Temp[jIndex] = Ki_Value[jIndex];
        Kd_Temp[jIndex] = Kd_Value[jIndex];
    }

    memcpy(P_TempBuffer[0], &Kp_Temp[0], sizeof(float));        //  float -> unsigned char  P-Gain Channel 1
    memcpy(P_TempBuffer[1], &Kp_Temp[1], sizeof(float));        //  float -> unsigned char  P-Gain Channel 2
    memcpy(P_TempBuffer[2], &Kp_Temp[2], sizeof(float));        //  float -> unsigned char  P-Gain Channel 3
    memcpy(P_TempBuffer[3], &Kp_Temp[3], sizeof(float));        //  float -> unsigned char  P-Gain Channel 4

    memcpy(I_TempBuffer[0], &Ki_Temp[0], sizeof(float));        //  float -> unsigned char  I-Gain Channel 1
    memcpy(I_TempBuffer[1], &Ki_Temp[1], sizeof(float));        //  float -> unsigned char  I-Gain Channel 2
    memcpy(I_TempBuffer[2], &Ki_Temp[2], sizeof(float));        //  float -> unsigned char  I-Gain Channel 3
    memcpy(I_TempBuffer[3], &Ki_Temp[3], sizeof(float));        //  float -> unsigned char  I-Gain Channel 4

    memcpy(D_TempBuffer[0], &Kd_Temp[0], sizeof(float));        //  float -> unsigned char  D-Gain Channel 1
    memcpy(D_TempBuffer[1], &Kd_Temp[1], sizeof(float));        //  float -> unsigned char  D-Gain Channel 2
    memcpy(D_TempBuffer[2], &Kd_Temp[2], sizeof(float));        //  float -> unsigned char  D-Gain Channel 3
    memcpy(D_TempBuffer[3], &Kd_Temp[3], sizeof(float));        //  float -> unsigned char  D-Gain Channel 4

    for(mIndex = 0; mIndex <4; mIndex++)
    {
        for(jIndex = 0; jIndex < 4; jIndex++)
        {
            FtoCBuffer[(12*mIndex)+jIndex] = P_TempBuffer[mIndex][jIndex];        // Temperature -> char
        }
    }
    for(mIndex = 0; mIndex <4; mIndex++)
    {
        for(jIndex = 0; jIndex < 4; jIndex++)
        {
            FtoCBuffer[(12*mIndex)+jIndex+4] = I_TempBuffer[mIndex][jIndex];      // Temperature -> char
        }
    }
    for(mIndex = 0; mIndex <4; mIndex++)
    {
        for(jIndex = 0; jIndex < 4; jIndex++)
        {
            FtoCBuffer[(12*mIndex)+jIndex+8] = D_TempBuffer[mIndex][jIndex];      // Temperature -> char
        }
    }
}

// Change Byte to Float
void ByteToFloat(void)
{
    int fIndex = 0;
    int mIndex = 0;

    float Kp_Temp1;
    float Kp_Temp2;
    float Kp_Temp3;
    float Kp_Temp4;

    float Ki_Temp1;
    float Ki_Temp2;
    float Ki_Temp3;
    float Ki_Temp4;

    float Kd_Temp1;
    float Kd_Temp2;
    float Kd_Temp3;
    float Kd_Temp4;

    for(mIndex = 0; mIndex <4; mIndex++)
    {
        for(fIndex = 0; fIndex < 4; fIndex++)
        {
            P_MemoryBuffer[mIndex][fIndex] = CtoFBuffer[(12*mIndex)+fIndex];            // Float->MemoryBuffer
        }
    }
    for(mIndex = 0; mIndex <4; mIndex++)
    {
        for(fIndex = 0; fIndex < 4; fIndex++)
        {
            I_MemoryBuffer[mIndex][fIndex] = CtoFBuffer[(12*mIndex)+fIndex+4];          // Float->MemoryBuffer
        }
    }
    for(mIndex = 0; mIndex <4; mIndex++)
    {
        for(fIndex = 0; fIndex < 4; fIndex++)
        {
            D_MemoryBuffer[mIndex][fIndex] = CtoFBuffer[(12*mIndex)+fIndex+8];          // Float->MemoryBuffer
        }
    }

    memcpy(&Kp_Temp1, P_MemoryBuffer[0], sizeof(float));        //  unsigned char -> float  P-Gain Channel 1
    memcpy(&Kp_Temp2, P_MemoryBuffer[1], sizeof(float));        //  unsigned char -> float  P-Gain Channel 2
    memcpy(&Kp_Temp3, P_MemoryBuffer[2], sizeof(float));        //  unsigned char -> float  P-Gain Channel 3
    memcpy(&Kp_Temp4, P_MemoryBuffer[3], sizeof(float));        //  unsigned char -> float  P-Gain Channel 3

    memcpy(&Ki_Temp1, I_MemoryBuffer[0], sizeof(float));        //  unsigned char -> float  I-Gain Channel 1
    memcpy(&Ki_Temp2, I_MemoryBuffer[1], sizeof(float));        //  unsigned char -> float  I-Gain Channel 2
    memcpy(&Ki_Temp3, I_MemoryBuffer[2], sizeof(float));        //  unsigned char -> float  I-Gain Channel 3
    memcpy(&Ki_Temp4, I_MemoryBuffer[3], sizeof(float));        //  unsigned char -> float  I-Gain Channel 3

    memcpy(&Kd_Temp1, D_MemoryBuffer[0], sizeof(float));        //  unsigned char -> float  D-Gain Channel 1
    memcpy(&Kd_Temp2, D_MemoryBuffer[1], sizeof(float));        //  unsigned char -> float  D-Gain Channel 2
    memcpy(&Kd_Temp3, D_MemoryBuffer[2], sizeof(float));        //  unsigned char -> float  D-Gain Channel 3
    memcpy(&Kd_Temp4, D_MemoryBuffer[3], sizeof(float));        //  unsigned char -> float  D-Gain Channel 3

    Kp_Value[0] = Kp_Temp1;
    Kp_Value[1] = Kp_Temp2;
    Kp_Value[2] = Kp_Temp3;
    Kp_Value[3] = Kp_Temp4;

    Ki_Value[0] = Ki_Temp1;
    Ki_Value[1] = Ki_Temp2;
    Ki_Value[2] = Ki_Temp3;
    Ki_Value[3] = Ki_Temp4;

    Kd_Value[0] = Kd_Temp1;
    Kd_Value[1] = Kd_Temp2;
    Kd_Value[2] = Kd_Temp3;
    Kd_Value[3] = Kd_Temp4;

}
