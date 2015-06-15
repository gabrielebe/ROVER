/* 
 * File:   SensorBoard.c
 * Author: Harpal
 *
 * Created on 23 gennaio 2015, 12.10
 */

#include <stdio.h>
#include <stdlib.h>


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
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown Out Reset Selection bits (BOR enabled)
#pragma config IESO = ON        // Internal External Switchover bit (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#define _XTAL_FREQ  8000000
#define ADDRESS 0x30

/*
 * 
 */
/*Function prototypes are declared here*/
void config(void);
void interrupt ISR(void);
void initialize_i2c(void);

/*Global variables are declared here*/

volatile unsigned char junk, I2C_receivedByte, I2C_sendByte;

int main(int argc, char** argv) {
    config();
    initialize_i2c();

    while (1) {

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
}

void interrupt ISR(void) // Interrupt Service Routine
{
    if (SSPIF) // I2C interrupt
    {
        if (!D_A && !R_W && BF) // An address from master: write to slave
        {
            junk = SSPBUF; // Empty buffer and clear BF bit

        } else if (D_A && !R_W && BF) // Data from master to slave
        {
            I2C_receivedByte = SSPBUF; // Output data to PORTC LEDs and clear BF bit

        } else if (!D_A && R_W && !BF) // An address from master: read from slave
        {
            SSPBUF = I2C_sendByte; // Data will be write to master
            CKP = 1; // Release clock (data ready to be send to master)
        } else if (D_A && R_W && !BF) // Read a byte from slave (next byte)
        {
            SSPBUF = I2C_sendByte; // Data will be write to master
            CKP = 1; // Release clock (data ready to be send to master)
        } else if (D_A && !R_W && !BF) // NAK from master (complete transmission has occurred)
        {
        }
    }
    SSPIF = 0; // Re-enable interrupt
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
    //setting the adress of the devices
    SSPADD = ADDRESS << 1;
    //disable  the general call (?)
    SSPCON2bits.GCEN = 0;
    //enabling the intterrupt
    PIE1bits.SSPIE = 1;
    //clearing the intterrupt flag
    PIR1bits.SSPIF = 0;
}
