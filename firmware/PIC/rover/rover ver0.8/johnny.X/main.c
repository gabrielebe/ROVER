/* 
 * File:   main.c
 * Author: Harpal
 *
 * Created on 24 dicembre 2014, 10.35
 */
#include <stdio.h>              //Standard c library for IO
#include <stdlib.h>             //Standard c library
#include <xc.h>                 //XC compiler library
#include <pic18f47j53.h>        //pic microcontroller library

#include "config.h"
#include "functions.h"          //library used for the sensors
#include "lcd.h"                //library used for the lcd control
#include "motorEncoder.h"       //library used for the motor and encoder control
#include "userInterface.h"      //library used for implementing the userinterface

/*
 *
 */
void interrupt ISR(void);
void config_init(void);
/***********************************/
/**********MAIN ROUTINE*************/
int main(int argc, char** argv)
{
    config_init();
    MOTOR_init();
    lcdInit();
    configSense();
    //MOTOR_testMotors();
    //lcdClean();

    while (1) {
        //encoder_move_degree(90,255,1);
        //encoder_move_degree(90,255,2);
        //encoder_move_degree(90,255,3);
        //encoder_move_degree(90,255,4);

        for( int j = 0;  j < 180; j+=10)
        {
            setServo(j);
            showTempLight();
            for(int i = 0; i < 20; i++) __delay_ms(10);
        }
        for( int j = 180;  j > 0; j-=10)
        {
            setServo(j);
            showAccelerometerVal();
            for(int i = 0; i < 20; i++) __delay_ms(10);
        }
    }

    return (EXIT_SUCCESS);
}
/***********************************/
void config_init()
{
    //TRISD = 0x00;
    //TRISA = 0xff;
    //TRISC = 0xff;

    EECON2 = 0x55;
    EECON2 = 0xAA;
    PPSCONbits.IOLOCK = 0;
    RPINR1 = 24;            //pin D7 is used as interrupt pin
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
