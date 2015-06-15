/* 
 * File:   main.c
 * Author: Harpal
 *
 * Created on 9 gennaio 2015, 11.21
 */

#include <stdio.h>
#include <stdlib.h>
#include <pic16f88.h>
#include <xc.h>
#include "config.h"
//#include "Analog.h"
//#include "Communication.h"
#define _XTAL_FREQ 8000000
/*
 * 
 */

//volatile unsigned char junk = 0; // used to place unnecessary data
//unsigned char first = 1; // used to determine whether data address
//volatile unsigned char value = '0'; // used to determine whether data address
// location or actual data

int main(int argc, char** argv) {
    //initialize_i2c();
    //enabling the interrupt
    //INTCONbits.PEIE = 1;
    //INTCONbits.GIE = 1;

    OSCCONbits.IRCF0 = 1;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF2 = 1;
    
    TRISB = 0x00;

     PORTB = 0xff;

        for(int i = 0;  i < 500; i++)
        {
            __delay_ms(10);
        }
    while(1) {
        PORTB = 0xff;

        for(int i = 0;  i < 100; i++)
        {
            __delay_ms(10);
        }
        PORTB = 0x00;
        for(int i = 0;  i < 100; i++)
        {
            __delay_ms(10);
        }

    }
    return (EXIT_SUCCESS);
}
//execute this function if any interrupt has been fired

void interrupt ISR(void) // Interrupt Service Routine
{
    if (SSPIF) // I2C interrupt
    {
        PORTBbits.RB0 = 0;
        if (!D_A && !R_W && BF) // An address from master: write to slave
        {
            junk = SSPBUF; // Empty buffer and clear BF bit

        } else if (D_A && !R_W && BF) // Data from master to slave
        {
            value = SSPBUF; // Output data to PORTC LEDs and clear BF bit
            value += 2;
        } else if (!D_A && R_W && !BF) // An address from master: read from slave
        {
            SSPBUF = value; // Data will be write to master
            CKP = 1; // Release clock (data ready to be send to master)
        } else if (D_A && R_W && !BF) // Read a byte from slave (next byte)
        {
            SSPBUF = value; // Data will be write to master
            CKP = 1; // Release clock (data ready to be send to master)
        } else if (D_A && !R_W && !BF) // NAK from master (complete transmission has occurred)
        {
        }

    }
    SSPIF = 0; // Re-enable interrupt
    PORTBbits.RB0 = 1;
}
