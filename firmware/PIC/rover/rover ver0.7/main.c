/* 
 * File:   main.c
 * Author: Harpal
 *
 * Created on 25 settembre 2014, 10.24
 */

#include <stdio.h>
#include <stdlib.h>


#include "config.h"
#include "functions.h"
#include "lcd.h"
#include "motorEncoder.h"
#include "userInterface.h"
#include <pic18f47j53.h>
#include <xc.h>
/*
 * 
 */
void interrupt ISR(void);
void showTempLight(void);
void showAccelerometerVal(void);
void showDistance(void);
void config_init(void);
/***********************************/
/**********MAIN ROUTINE*************/
int main(int argc, char** argv)
{
    config_init();
    lcdInit();
    MOTOR_init();
    
    while (1) {
      
    }

    return (EXIT_SUCCESS);
}
/***********************************/
void config_init()
{
    TRISD = 0x00;
    TRISA = 0xff;
    TRISC = 0xff;

    EECON2 = 0x55;
    EECON2 = 0xAA;
    PPSCONbits.IOLOCK = 0;
    RPINR1 = 18;
    EECON2 = 0x55;
    EECON2 = 0xAA;
    PPSCONbits.IOLOCK = 1;

    INTCON2bits.INTEDG1 = 1;        //Interrupt on rising edge
    INTCON3bits.INT1IE = 1;         //enable external interrupt 1
    INTCONbits.GIE = 1;             //enabling all the interrupts

    ANCON0 = 0b11111111; //set All pin as digital
    ANCON1 |= 0b00001111;
}

void interrupt ISR(void) {
    if (INTCON3bits.INT1IF == 1) 
    {
        /*************************/
        /****USER-INTERFACE*******/

        /*
        lcdClear();
        lcdWriteStrC(readSwitch());
        __delay_ms(50);
        */
        INTCON3bits.INT1IF = 0;
    }
}
