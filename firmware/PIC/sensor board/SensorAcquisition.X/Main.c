/* 
 * File:   Main.c
 * Author: Harpal
 *
 * Created on 30 dicembre 2014, 9.53
 */

#include <stdio.h>
#include <stdlib.h>
#include "config.h"
/*
 * 
 */
int main(int argc, char** argv) {

    
    //The continous loop
    while(1)
    {

    }

    return (EXIT_SUCCESS);
}

int analogRead(char channel)
{
    int value; //variable used to save the analog value
    //A/D configuration
    //Setting the conversion clock as Fosc/2
    ADCON0bits.ADCS0 = 0;
    ADCON0bits.ADCS1 = 0;
    ADCON1bits.ADCS2 = 0;   //A/D clock is not divided by 2
    //Setting the reference of the voltages as Vdd and Vss
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;
    //Formating the coversion as right justified
    ADCON1bits.ADFM = 1;
    //channel is selected by
    //the parameter given to this function
    switch(channel)
    {
        case 0:
            ANSEL &= 0b10000001;    //set the other channel as digitl input
            ADCON0bits.CHS0 = 0;
            ADCON0bits.CHS1 = 0;
            ADCON0bits.CHS2 = 0;
            break;
        case 1:
            ANSEL &= 0b10000010;
            ADCON0bits.CHS0 = 1;
            ADCON0bits.CHS1 = 0;
            ADCON0bits.CHS2 = 0;
            break;
        case 2:
            ANSEL &= 0b10000100;
            ADCON0bits.CHS0 = 0;
            ADCON0bits.CHS1 = 1;
            ADCON0bits.CHS2 = 0;
            break;
        case 3:
            ANSEL &= 0b10001000;
            ADCON0bits.CHS0 = 1;
            ADCON0bits.CHS1 = 1;
            ADCON0bits.CHS2 = 0;
            break;
        case 4:
            ANSEL &= 0b10010000;
            ADCON0bits.CHS0 = 0;
            ADCON0bits.CHS1 = 0;
            ADCON0bits.CHS2 = 1;
            break;
        case 5:
            ANSEL &= 0b10100000;
            ADCON0bits.CHS0 = 1;
            ADCON0bits.CHS1 = 0;
            ADCON0bits.CHS2 = 1;
            break;
        case 6:
            ANSEL &= 0b11000000;
            ADCON0bits.CHS0 = 0;
            ADCON0bits.CHS1 = 1;
            ADCON0bits.CHS2 = 1;
            break;
        default:
            return -10;
    }
    //turning on the A/D module
    ADCON0bits.ADON = 1;
    GO = 1;
    while(DONE);
    //save the value once the acquisition has finished
    value =(((ADRESH << 8)&0x3) + ADRESL);
    //turn off the module
    ADCON0bits.ADON = 0;
    return value;   //return the value just read
}