/* 
 * File:   functions.c
 * Author: Harpal
 *
 * Created on 25 settembre 2014, 10.50
 */

#include <stdio.h>
#include <stdlib.h>
#include "functions.h"
#include "config.h"
#include <math.h>
//***********************//
//***********************//
//ADC functions

int read(int channel) {
    int value = 0;
    switch (channel) {
        case 0: 
            ADCON0bits.CHS0 = 0; //select channel A0
            ADCON0bits.CHS1 = 0;
            ADCON0bits.CHS2 = 0;
            ADCON0bits.CHS3 = 0;

            ANCON0 &= 0b11111110; //A0-RA0 as analog pin
            ANCON1 |= 0b00001111;
            break;

        case 1:
            ADCON0bits.CHS0 = 1; //select channel A1
            ADCON0bits.CHS1 = 0;
            ADCON0bits.CHS2 = 0;
            ADCON0bits.CHS3 = 0;

            ANCON0 &= 0b11111101; //A1-RA1 as analog pin
            ANCON1 |= 0b00001111;
            break;

        case 2:
            ADCON0bits.CHS0 = 0; //select channel A1
            ADCON0bits.CHS1 = 1;
            ADCON0bits.CHS2 = 0;
            ADCON0bits.CHS3 = 0;

            ANCON0 &= 0b11111011; //A2-RA2 as analog pin
            ANCON1 |= 0b00001111;
            break;
        case 3:
            ADCON0bits.CHS0 = 1; //select channel A1
            ADCON0bits.CHS1 = 1;
            ADCON0bits.CHS2 = 0;
            ADCON0bits.CHS3 = 0;

            ANCON0 &= 0b11110111; //A3-RA3 as analog pin
            ANCON1 |= 0b00001111;
            break;
        case 4:
            ADCON0bits.CHS0 = 1; //select channel A7
            ADCON0bits.CHS1 = 1;
            ADCON0bits.CHS2 = 1;
            ADCON0bits.CHS3 = 0;

            ANCON0 &= 0b01111111; //A7-RE2 as analog pin
            ANCON1 |= 0b00001111;
            break;

    }
    ADCON0bits.VCFG0 = 0; // Vref+ = AVdd
    ADCON0bits.VCFG1 = 0; // Vref- = AVss

    ADCON1bits.ADFM = 1; //Right justification

    ADCON1bits.ACQT0 = 0; // 16TAD is selected
    ADCON1bits.ACQT1 = 1;
    ADCON1bits.ACQT2 = 1;

    ADCON1bits.ADCS0 = 0; //Fosc/2 is the A/D conversion clock
    ADCON1bits.ADCS1 = 0;
    ADCON1bits.ADCS2 = 0;

    ADCON0bits.ADON = 1; //Enabling the A/D module
    GO_DONE = 1;
    while (GO_DONE);
    value = ((ADRESH << 8) + ADRESL);

    ADCON0bits.ADON = 0; //turning off the adc module

    return value;
}
//***********************//
//***********************//

//***********************//
//Servo control

void setServo(int degree) {
        for (int j = 0; j < 20; j++) {


            LATBbits.LATB0 = 1;
            for (int i = 0; i < 10; i++)__delay_us(25);
            for (int i = 0; i < degree; i++)__delay_us(4);

            LATBbits.LATB0 = 0;
            for (int i = 0; i < 180 - degree; i++)
            {
               __delay_us(3);
            }

            for (int i = 0; i < 1775; i++)
            {
                __delay_us(10);
            }
        }

}
//*************************//
//*************************//
//Ultrasonic distance sensor
// send sensor

double readDistance() {
    double a;
    TMR1H = 0; //Sets the Initial Value of Timer
    TMR1L = 0; //Sets the Initial Value of Timer

    T1CONbits.TMR1CS0 = 0;          //Setting the clock to Fosc/4
    T1CONbits.TMR1CS0 = 0;

    T1CONbits.T1CKPS0 = 1;          //Setting the prescaler to 1:2
    T1CONbits.T1CKPS1 = 0;

    LATBbits.LATB1 = 1; //TRIGGER HIGH
    __delay_us(10); //10uS Delay
    LATBbits.LATB1 = 0; //TRIGGER LOW

    while (!RB2); //Waiting for Echo
    T1CONbits.TMR1ON = 1; //Timer Starts
    while (RB2); //Waiting for Echo goes LOW
    T1CONbits.TMR1ON = 0; //Timer Stops

    a = (TMR1L | (TMR1H << 8)); //Reads Timer Value
    a = (a*43.6)/0xffff;        //convert in mS
    a = ((a * distanceCalibration) / 2)*1000; //Converts Time to Distance
    a += 1.3;               //Calibration;
    //a = a/58.82;        //converter time to distance
    //a = a+1;            //for calibration
    return a;
}

//***********************//
//***********************//
//Temperature functions

float readTempF(void) {
    float temp;
    float j = 0;
    for (int i = 0; i < 5; i++)
        j = (float) read(0) + j; //read five times
    j = j / 5; //finde the average
    j = ((j * 3.3) / 4096); //scale it on the voltage
    temp = (18.181818 * j) - 10; //find the temp in celsius degree

    return temp;
}
//***********************//
//***********************//

//***********************//
//***********************//
//Light functions

int readLight(void) {
    long j = read(1);
    j = ((j * 100) / 4096);
    return j;
}
//***********************//
//***********************//

//***********************//
//***********************//
//Accelerometer functions

float single_axis_measure(unsigned int axis, const float iteration) {
    float axis_sum = 0.0;
    for (unsigned int i = 0; i < iteration; i++) {
        const float axis_read = read_accelerometer(axis);
        axis_sum = axis_sum + axis_read;
    }

    const float medium_value = axis_sum / iteration;
    axis_sum = 0;
    return voltageToG(medium_value);
}

float read_accelerometer(const unsigned int axis) {
    const unsigned int axis_point = read(axis);
    return pointToVoltage(axis_point);
}

float pointToVoltage(const unsigned int point) {
    const float fPoint = (float) point;
    return (3.3 * fPoint) / 4096;
}

float voltageToG(const float voltage) {
    return (voltage - zero_g_ref_voltage) / sensor_sensitivity;
}

//******************************************//
//New functions
