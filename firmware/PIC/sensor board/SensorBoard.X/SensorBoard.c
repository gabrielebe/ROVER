/* 
 * File:   SensorBoard.c
 * Author: Harpal
 *
 * Created on 23 gennaio 2015, 12.10
 */

#include <stdio.h>
#include <stdlib.h>
#include "Analog.h"

// PIC16F886 Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1
#pragma config FOSC = EXTRC_NOCLKOUT// Oscillator Selection bits (RCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = ON       // RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Register Code Protection bit (Register memory code protection is disabled)
#pragma config BOREN = ON       // Brown Out Reset Selection bits (BOR enabled)
#pragma config IESO = ON        // Internal External Switchover bit (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#define _XTAL_FREQ  8000000
#define ADDR_BASE   0b01110000

#ifndef ADDR_PIN1
#define ADDR_PIN1           (PORTCbits.RC0)
#define ADDR_PIN1_DIR       (TRISCbits.TRISC0)
#endif

#ifndef ADDR_PIN2
#define ADDR_PIN2           (PORTCbits.RC1)
#define ADDR_PIN2_DIR       (TRISCbits.TRISC1)
#endif

#ifndef STATUS_LED
#define STATUS_LED          (PORTCbits.RC5)
#define STATUS_LED_DIR      (TRISCbits.TRISC5)
#endif



/* Function prototypes are declared here */
void config(void);
void initialize_i2c(void);
void interrupt ISR(void);
unsigned char sendMaster(void);
void receiveData(void);
void fillRegister(char);

/* Global variables are declared here */
volatile unsigned char junk, I2C_receivedByte, I2C_sendByte;
int counter = 1;
/* Communication variables */
enum {
    index,
    tempL,
    tempH,
    accXL,
    accXH,
    accYL,
    accYH,
    accZL,
    accZH,
    lightL,
    lightH,
    configuration,
    status,
    last
};

enum {
    Temprature,
    Light,
    Accelerometer
};

char Register[last] = {0};

int main(int argc, char** argv) {
    config();
    initialize_i2c();
    tempCalibration();
    STATUS_LED_DIR = 0;
    while (1) {
        STATUS_LED = 1;
        fillRegister(Temprature);
        fillRegister(Light);
        fillRegister(Accelerometer);
    }

    return (EXIT_SUCCESS);
}

void config(void) {
    //setting the oscillator to 8 MHz
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF2 = 1;

    //setting by default the internal clock source for the system
    OSCCONbits.SCS = 1;

    //Setting the port b as digital
    ANSELH = 0x00;

    STATUS_LED_DIR = 0;
}

void interrupt ISR(void) // Interrupt Service Routine
{
    if (SSPIF) // I2C interrupt
    {
        STATUS_LED = 0;
        if (!D_A && !R_W && BF) // An address from master: write to slave
        {
            junk = SSPBUF; // Empty buffer and clear BF bit

        } else if (D_A && !R_W && BF) // Register from master to slave
        {
            receiveData();   //Go to the function that handles the reception from master

        } else if (!D_A && R_W && !BF) // An address from master: read from slave
        {
            SSPBUF = Register[tempL]; // Register will be write to master
            CKP = 1; // Release clock (Register ready to be send to master)
        } else if (D_A && R_W && !BF) // Read a byte from slave (next byte)
        {
            SSPBUF = Register[tempL]; // Register will be write to master
            CKP = 1; // Release clock (Register ready to be send to master)
        } else if (D_A && !R_W && !BF) // NAK from master (complete transmission has occurred)
        {
            counter = 1;    //clear the counter
        }
    }
    SSPIF = 0; // Re-enable interrupt
}
void fillRegister(char array) {

    switch (array) {
        case Temprature:
            Register[tempL] = (0x00ff & (int)readTemp());
            Register[tempH] = ((int)readTemp() >> 8)&0x00ff;
            break;

        case Light:
            Register[lightL] = 0x00ff & (int)readLight();
            Register[lightH] = ((int)readLight() >> 8)&0x00ff;
            break;

        case Accelerometer:
            Register[accXL] = 0x00ff & (int)readAccelerometer(CHANNEL_X_AXIS);
            Register[accXH] = ((int)readAccelerometer(CHANNEL_X_AXIS) >> 8)&0x00ff;
            Register[accYL] = 0x00ff & (int)readAccelerometer(CHANNEL_Y_AXIS);
            Register[accYH] = ((int)readAccelerometer(CHANNEL_Y_AXIS) >> 8)&0x00ff;
            Register[accZL] = 0x00ff & (int)readAccelerometer(CHANNEL_Z_AXIS);
            Register[accZH] = ((int)readAccelerometer(CHANNEL_Z_AXIS) >> 8)&0x00ff;
    }
}

void receiveData(void)
{
    if(counter == 1)
    {
        Register[index] = SSPBUF;
        counter++;
    }
    else
    {
        Register[Register[index]] = SSPBUF;
        counter++;
    }
}

unsigned char sendMaster(void)
{
    static char indexCounter = 0;
    if(!(Register[index] < last && Register[index] > index))
    {
        Register[index] = 1;
        indexCounter = 0;
    }
    switch(indexCounter)
    {
        case 0:
            indexCounter++;
            return Register[tempL];
            
        case 1:
            indexCounter++;
            return Register[tempH];
            
        case 2:
            indexCounter++;
            return Register[lightL];
            
        case 3:
            indexCounter++;
            return Register[lightH];
            
        case 4:
            indexCounter++;
            return Register[accXL];
            
        case 5:
            indexCounter++;
            return Register[accXH];
            
        case 6:
            indexCounter++;
            return Register[accYL];
            
        case 7:
            indexCounter++;
            return Register[accYH];
            
        case 8:
            indexCounter++;
            return Register[accZL];
            
        case 9:
            indexCounter++;
            return Register[accZH];
            
        case 10:
            indexCounter = 0;
            return Register[status];
        default:
            return -10;
    }
}

void initialize_i2c(void) {
    //setting the SDA and SCL pin as input
    TRISCbits.TRISC3 = 1; //SCL pin
    TRISCbits.TRISC4 = 1; //SDA pin

    //enabling the slew rate mode to work at 400KHz
    SSPSTATbits.SMP = 0;

    //don't care about the write collision
    SSPCONbits.WCOL = 0;

    //dont' care about the oveerflow condition
    SSPCONbits.SSPOV = 0;


    //selecting the funztion mode as I2C slave with 7 bit address
    //without any interrupt on start and stop
    SSPCONbits.SSPM0 = 0;
    SSPCONbits.SSPM1 = 1;
    SSPCONbits.SSPM2 = 1;
    SSPCONbits.SSPM3 = 0;
    SSPCONbits.CKP = 1; //release the clock
    SSPCONbits.SSPEN = 1; //enable the module

    ADDR_PIN1_DIR = 1;
    ADDR_PIN2_DIR = 1;

    // Setting the adress of the devices
    SSPADD = ((ADDR_PIN1) | (ADDR_PIN2 << 1) | ADDR_BASE) << 1;

    //disable  the general call (?)
    SSPCON2bits.GCEN = 0;

    //enabling the intterrupt
    PIE1bits.SSPIE = 1;

    //clearing the intterrupt flag
    PIR1bits.SSPIF = 0;
}
