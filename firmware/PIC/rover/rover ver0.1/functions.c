/* 
 * File:   functions.c
 * Author: Harpal
 *
 * Created on 25 settembre 2014, 10.50
 */

#include <stdio.h>
#include <stdlib.h>
#include "functions.h"
#include <pic18f47j13.h>

//***********************//
//***********************//
//ADC functions
int read(int channel)
{
    int value = 0;
    switch(channel)
    {
        case 0: ADCON0bits.CHS0 = 0;           //select channel A0
                ADCON0bits.CHS1 = 0;
                ADCON0bits.CHS2 = 0;
                ADCON0bits.CHS3 = 0;

                ANCON0 &= 0b11111110;            //A0-RA0 as analog pin
                ANCON1 |= 0b00001111;
                break;

        case 1: ADCON0bits.CHS0 = 1;           //select channel A1
                ADCON0bits.CHS1 = 0;
                ADCON0bits.CHS2 = 0;
                ADCON0bits.CHS3 = 0;

                ANCON0 &= 0b11111101;            //A1-RA1 as analog pin
                ANCON1 |= 0b00001111;
                break;

        case 2: ADCON0bits.CHS0 = 0;           //select channel A1
                ADCON0bits.CHS1 = 1;
                ADCON0bits.CHS2 = 0;
                ADCON0bits.CHS3 = 0;

                ANCON0 &= 0b11111011;            //A2-RA2 as analog pin
                ANCON1 |= 0b00001111;
                break;
        case 3: ADCON0bits.CHS0 = 1;           //select channel A1
                ADCON0bits.CHS1 = 1;
                ADCON0bits.CHS2 = 0;
                ADCON0bits.CHS3 = 0;

                ANCON0 &= 0b11110111;            //A3-RA3 as analog pin
                ANCON1 |= 0b00001111;
                break;
        case 4: ADCON0bits.CHS0 = 1;           //select channel A7
                ADCON0bits.CHS1 = 1;
                ADCON0bits.CHS2 = 1;
                ADCON0bits.CHS3 = 0;

                ANCON0 &= 0b01111111;            //A7-RE2 as analog pin
                ANCON1 |= 0b00001111;
                break;

    }
    ADCON0bits.VCFG0 = 0;            // Vref+ = AVdd
    ADCON0bits.VCFG1 = 0;            // Vref- = AVss

    ADCON1bits.ADFM = 1;            //Right justification

    ADCON1bits.ACQT0 = 0;           // 16TAD is selected
    ADCON1bits.ACQT1 = 1;
    ADCON1bits.ACQT2 = 1;

    ADCON1bits.ADCS0 = 0;           //Fosc/2 is the A/D conversion clock
    ADCON1bits.ADCS1 = 0;
    ADCON1bits.ADCS2 = 0;

    ADCON0bits.ADON = 1;            //Enabling the A/D module
    GO_DONE = 1;
    while(GO_DONE);
    value = ((ADRESH << 8) + ADRESL);

    ADCON0bits.ADON = 0;            //turning off the adc module

    return value;
}
//***********************//
//***********************//

//***********************//
//***********************//
//Temperature functions
float readTempF(void)
{
    float temp;
    float j = 0;
    for(int i = 0; i < 5; i++)
    j = (float)read(0)+j;               //read five times
    j = j/5;                            //finde the average
    j = ((j*3.3)/4096);                 //scale it on the voltage
    temp = (18.181818*j) - 10;          //find the temp in celsius degree

    return temp;
}
//***********************//
//***********************//

//***********************//
//***********************//
//Light functions
int readLight(void)
{
    long j = read(1);
    j = ((j*100)/4096);
    return j;
}
//***********************//
//***********************//

//***********************//
//***********************//
//Accelerometer functions
float single_axis_measure(unsigned int axis, const float iteration)
{
    float axis_sum = 0.0;
    for(unsigned int i = 0; i < iteration; i++)
    {
        const float axis_read = read_accelerometer(axis);
        axis_sum = axis_sum + axis_read;
    }

    const float medium_value = axis_sum / iteration;
    axis_sum = 0;
    return voltageToG(medium_value);
}


float read_accelerometer(const unsigned int axis)
{
    const unsigned int axis_point = read(axis);
    return pointToVoltage(axis_point);
}


float pointToVoltage(const unsigned int point)
{
    const float fPoint = (float)point;
    return (3.3 * fPoint) / 4096;
}


float voltageToG(const float voltage)
{
    return (voltage - zero_g_ref_voltage) / sensor_sensitivity;
}