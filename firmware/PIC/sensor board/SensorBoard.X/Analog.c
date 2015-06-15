/*
 * File:   analog.c
 * Author: Harpal
 *
 * Created on 9 gennaio 2015, 11.36
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <pic16f886.h>
#include "Analog.h"

float gainTemp = 18.181818;

int read(int channel, char justification) {
    int analogValue;

    //channnel selection
    ANSEL = 0x00;
    switch (channel) {
        case 0:
            ANSELbits.ANS0 = 1;
            ADCON0bits.CHS0 = 0;
            ADCON0bits.CHS1 = 0;
            ADCON0bits.CHS2 = 0;
            ADCON0bits.CHS3 = 0;
            break;

        case 1:
            ANSELbits.ANS1 = 1;
            ADCON0bits.CHS0 = 1;
            ADCON0bits.CHS1 = 0;
            ADCON0bits.CHS2 = 0;
            ADCON0bits.CHS3 = 0;
            break;

        case 2:
            ANSELbits.ANS2 = 1;
            ADCON0bits.CHS0 = 0;
            ADCON0bits.CHS1 = 1;
            ADCON0bits.CHS2 = 0;
            ADCON0bits.CHS3 = 0;
            break;

        case 3:
            ANSELbits.ANS3 = 1;
            ADCON0bits.CHS0 = 1;
            ADCON0bits.CHS1 = 1;
            ADCON0bits.CHS2 = 0;
            ADCON0bits.CHS3 = 0;
            break;

        case 4:
            ANSELbits.ANS4 = 1;
            ADCON0bits.CHS0 = 0;
            ADCON0bits.CHS1 = 0;
            ADCON0bits.CHS2 = 1;
            ADCON0bits.CHS3 = 0;
            break;

        case 5:
            ADCON0bits.CHS0 = 1;
            ADCON0bits.CHS1 = 0;
            ADCON0bits.CHS2 = 1;
            ADCON0bits.CHS3 = 0;
            break;

        case 6:
            ADCON0bits.CHS0 = 0;
            ADCON0bits.CHS1 = 1;
            ADCON0bits.CHS2 = 1;
            ADCON0bits.CHS3 = 0;
            break;

        case 7:
            ADCON0bits.CHS0 = 1;
            ADCON0bits.CHS1 = 1;
            ADCON0bits.CHS2 = 1;
            ADCON0bits.CHS3 = 0;
            break;

        case 8:
            ADCON0bits.CHS0 = 0;
            ADCON0bits.CHS1 = 0;
            ADCON0bits.CHS2 = 0;
            ADCON0bits.CHS3 = 1;
            break;

        case 9:
            ADCON0bits.CHS0 = 1;
            ADCON0bits.CHS1 = 0;
            ADCON0bits.CHS2 = 0;
            ADCON0bits.CHS3 = 1;
            break;

        case 10:
            ADCON0bits.CHS0 = 0;
            ADCON0bits.CHS1 = 1;
            ADCON0bits.CHS2 = 0;
            ADCON0bits.CHS3 = 1;
            break;

        case 11:
            ADCON0bits.CHS0 = 1;
            ADCON0bits.CHS1 = 1;
            ADCON0bits.CHS2 = 0;
            ADCON0bits.CHS3 = 1;
            break;

        case 12:
            ADCON0bits.CHS0 = 0;
            ADCON0bits.CHS1 = 0;
            ADCON0bits.CHS2 = 1;
            ADCON0bits.CHS3 = 1;
            break;

        case 13:
            ADCON0bits.CHS0 = 1;
            ADCON0bits.CHS1 = 0;
            ADCON0bits.CHS2 = 1;
            ADCON0bits.CHS3 = 1;
            break;

        default:
            ERROR_LED_DIR = 0;
            ERROR_LED = 1;
            return ERROR_WRONG_CHANNEL;
    }

    //setting the justification
    if (justification == 1) ADCON1bits.ADFM = 0;
    else ADCON1bits.ADFM = 1;

    //setting up the converting frequancy
    ADCON0bits.ADCS0 = 1; //Internal RC converting frequancy is used
    ADCON0bits.ADCS1 = 1; //In this case it is 500KHz

    //selecting the refrence voltages as Vdd and Vss
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;

    //turn on the analog module
    ADCON0bits.ADON = 1;

    //read the analog value
    GO_DONE = 1;
    while (GO_DONE);

    //save it in a variable
    if (justification == 1) analogValue = ((ADRESL << 8) + ADRESH);
    else analogValue = ((ADRESH << 8) + ADRESL);

    //trun off the module
    ADCON0bits.ADON = 0;

    return analogValue;
}


float readLight(void) {
    double lightVal;
    lightVal = (pointToVoltage((const double) read(CHANNEL_LIGHT, 0)) * M_COSTANT);
    lightVal = lightVal / GAIN_LIGHT + C_COSTANT;

    //transforming the value in lux
    if (lightVal != 0)
        lightVal = pow((A_COSTANT / (double) lightVal), (1 / (double) ALPHA_COSTANT));
    else
    {
        ERROR_LED_DIR = 0;
        ERROR_LED = 1;
        return ERROR_DIVIDE_BY_ZERO;
    }

    //return the value back
    return (float) lightVal;
}


// Function to calibrate the gain of the temprature
float tempCalibration(void) {
    float vOut = read(CHANNEL_TEMPRATURE, 0);
    float vIn = (read(CHANNEL_CALIBRATION, 0)) - 2.5;

    if (vIn != 0)
        gainTemp = vOut / vIn;
    else
    {
        ERROR_LED_DIR = 0;
        ERROR_LED = 1;
        return ERROR_DIVIDE_BY_ZERO;
    }

    return gainTemp;
}


float readTemp(void) {
    float temp = 0; //variable used to store the temprature value

    for (int i = 0; i < 5; i++) //read the temprature value 5 times
        temp = (float) read(CHANNEL_TEMPRATURE, 0) + temp; //add the read value to the previous one

    temp = temp / 5; //find the average
    temp = pointToVoltage(temp); //scale it to the voltage level
    temp = (gainTemp * temp) + TEMP_COSTANT; //and find the temprature in degree celsius

    return temp; //return the just read value back
}


float singleAxisMeasure(int axis, int iteration) {
    float sum = 0.0;
    float read;
    float mediumValue;
    
    for (unsigned int i = 0; i < iteration; i++)
    {
        read = readAccelerometer(axis);
        sum = sum + read;
    }
    
    mediumValue = sum / iteration;
    return ((mediumValue - ZERO_G_VOLTAGE_REF) / SENSOR_SENSITIVITY);
}


// Function used to read the accelerometer axis
float readAccelerometer(int axis) {
    if(axis == CHANNEL_X_AXIS ||axis == CHANNEL_Y_AXIS ||axis == CHANNEL_Z_AXIS)
        return pointToVoltage((int) read(axis, 0));
    else
        return ERROR_WRONG_CHANNEL;
}

// Function used to scale the value to the correct voltage
double pointToVoltage(double point) {
    return (POWER_SUPPLY * point) / 1023;
}
