/* 
 * File:   analog.c
 * Author: Harpal
 *
 * Created on 9 gennaio 2015, 11.36
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <pic16f88.h>
#include "Analog.h"

int read(int channel, char justification)
{
    int analogValue;
    //channnel selection
    ANSEL = 0x00;
    switch(channel)
    {
        case 0:
            TRISAbits.TRISA0 = 1;
            ANSELbits.ANS0 = 1;
            ADCON0bits.CHS0 = 0;
            ADCON0bits.CHS1 = 0;
            ADCON0bits.CHS2 = 0;
            break;
        case 1:
            TRISAbits.TRISA1 = 1;
            ANSELbits.ANS1 = 1;
            ADCON0bits.CHS0 = 1;
            ADCON0bits.CHS1 = 0;
            ADCON0bits.CHS2 = 0;
            break;
        case 2:
            TRISAbits.TRISA2 = 1;
            ANSELbits.ANS2 = 1;
            ADCON0bits.CHS0 = 0;
            ADCON0bits.CHS1 = 1;
            ADCON0bits.CHS2 = 0;
            break;
        case 3:
            TRISAbits.TRISA3 = 1;
            ANSELbits.ANS3 = 1;
            ADCON0bits.CHS0 = 1;
            ADCON0bits.CHS1 = 1;
            ADCON0bits.CHS2 = 0;
            break;
        case 4:
            TRISAbits.TRISA4 = 1;
            ANSELbits.ANS4 = 1;
            ADCON0bits.CHS0 = 0;
            ADCON0bits.CHS1 = 0;
            ADCON0bits.CHS2 = 1;
            break;
        case 5:
            TRISBbits.TRISB5 = 1;
            ANSELbits.ANS5 = 1;
            ADCON0bits.CHS0 = 1;
            ADCON0bits.CHS1 = 0;
            ADCON0bits.CHS2 = 1;
            break;
        case 6:
            TRISBbits.TRISB6 = 1;
            ANSELbits.ANS6 = 1;
            ADCON0bits.CHS0 = 0;
            ADCON0bits.CHS1 = 1;
            ADCON0bits.CHS2 = 1;
            break;
        default:
            return ERROR_WRONG_CHANNEL;
    }

    //setting the justification
    if(justification == 1)  ADCON1bits.ADFM = 0;
    else ADCON1bits.ADFM = 1;

    //setting up the converting frequancy
    ADCON0bits.ADCS0 = 1;       //Internal RC converting frequancy is used
    ADCON0bits.ADCS1 = 1;

    //selecting the refrence voltages as Vdd and Vss
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;

    //turn on the analog module
    ADCON0bits.ADON = 1;
    //read the analog value
    GO_DONE = 1;
    while (GO_DONE);
    //save it in a variable
    if(justification == 1) analogValue = ((ADRESL << 8) + ADRESH);
    else analogValue = ((ADRESH << 8) + ADRESL);
    //trun off the module
    ADCON0bits.ADON = 0;

    return analogValue;
}

float readLight(void)
{
    double lightVal;
    lightVal = (pointToVoltage((const double)read(CHANNEL_LIGHT,0)) * M_COSTANT);
    lightVal = lightVal/GAIN_LIGHT + C_COSTANT;
    //transforming the value in lux
    if(lightVal != 0)
        lightVal = pow((A_COSTANT/(double)lightVal),(1/(double)ALPHA_COSTANT));
    else
        return ERROR_DIVIDE_BY_ZERO;
    //return the value back
    return (float)lightVal;
}

//function to calibrate the gain of the temprature
void tempCalibration(void)
{

}

float readTemp(void)
{
    float temp = 0;                     //variable used to store the temprature value
    for (int i = 0; i < 5; i++)         //read the temprature value 5 times
        temp = (float) read(CHANNEL_TEMPRATURE,0) + temp; //add the read value to the previous one
    temp = temp / 5;                    //find the average
    temp = pointToVoltage(temp);        //scale it to the voltage level
    temp = (18.181818 * temp) + TEMP_COSTANT;     //and find the temprature in degree celsius

    return temp;                        //return the just read value back
}

float single_axis_measure(unsigned int axis,int iteration) {
    float axis_sum = 0.0;
    float axis_read;
    float medium_value;
    for (unsigned int i = 0; i < iteration; i++) {
        axis_read = read_accelerometer(axis);
        axis_sum = axis_sum + axis_read;
    }
    medium_value = axis_sum / iteration;
    axis_sum = 0;
    return voltageToG(medium_value);
}

// function used to read the accelerometer axis
float read_accelerometer(const unsigned int axis) {
    return pointToVoltage((int)read(axis,0));
}

//function used to scale the value to the correct voltage
double pointToVoltage(double point) {
    return (POWER_SUPPLY * point) / 1023;
}

float voltageToG(const float voltage) {
    return (voltage - ZERO_G_VOLTAGE_REF) / SENSOR_SENSITIVITY;
}