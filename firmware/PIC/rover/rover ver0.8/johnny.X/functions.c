/*
 * File:   lcd.c
 * Author: Harpal
 *
 * Created on 24 dicembre 2014, 10.48
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


//*********************//
//****ADC functions****//
int read(int channel) {
    int value = 0;
    //select the channel
    analogChannelPin0_DIR = 0;
    analogChannelPin1_DIR = 0;
    analogChannelPin2_DIR = 0;
    analogChannelPin3_DIR = 0;
    switch (channel) {
        case 0:
            analogChannelPin0 = 0;
            analogChannelPin1 = 0;
            analogChannelPin2 = 0;
            analogChannelPin3 = 0;
            break;
        case 1:
            analogChannelPin0 = 1;
            analogChannelPin1 = 0;
            analogChannelPin2 = 0;
            analogChannelPin3 = 0;
            break;
        case 2:
            analogChannelPin0 = 0;
            analogChannelPin1 = 1;
            analogChannelPin2 = 0;
            analogChannelPin3 = 0;
            break;
        case 3:
            analogChannelPin0 = 1;
            analogChannelPin1 = 1;
            analogChannelPin2 = 0;
            analogChannelPin3 = 0;
            break;
        case 4:
            analogChannelPin0 = 0;
            analogChannelPin1 = 0;
            analogChannelPin2 = 1;
            analogChannelPin3 = 0;
            break;
        case 5:
            analogChannelPin0 = 1;
            analogChannelPin1 = 0;
            analogChannelPin2 = 1;
            analogChannelPin3 = 0;
            break;
        case 6:
            analogChannelPin0 = 0;
            analogChannelPin1 = 1;
            analogChannelPin2 = 1;
            analogChannelPin3 = 0;
            break;
        case 7:
            analogChannelPin0 = 1;
            analogChannelPin1 = 1;
            analogChannelPin2 = 1;
            analogChannelPin3 = 0;
            break;
        case 8:
            analogChannelPin0 = 0;
            analogChannelPin1 = 0;
            analogChannelPin2 = 0;
            analogChannelPin3 = 1;
            break;
        case 9:
            analogChannelPin0 = 1;
            analogChannelPin1 = 0;
            analogChannelPin2 = 0;
            analogChannelPin3 = 1;
            break;
        case 10:
            analogChannelPin0 = 0;
            analogChannelPin1 = 1;
            analogChannelPin2 = 0;
            analogChannelPin3 = 1;
            break;
        case 11:
            analogChannelPin0 = 1;
            analogChannelPin1 = 1;
            analogChannelPin2 = 0;
            analogChannelPin3 = 1;
            break;
        case 12:
            analogChannelPin0 = 0;
            analogChannelPin1 = 0;
            analogChannelPin2 = 1;
            analogChannelPin3 = 1;
            break;
        case 13:
            analogChannelPin0 = 1;
            analogChannelPin1 = 0;
            analogChannelPin2 = 1;
            analogChannelPin3 = 1;
            break;
        case 14:
            analogChannelPin0 = 0;
            analogChannelPin1 = 1;
            analogChannelPin2 = 1;
            analogChannelPin3 = 1;
            break;
        case 15:
            analogChannelPin0 = 1;
            analogChannelPin1 = 1;
            analogChannelPin2 = 1;
            analogChannelPin3 = 1;
            break;
    }
    ADCON0bits.CHS0 = 0; //select the A/D channel A0
    ADCON0bits.CHS1 = 0;
    ADCON0bits.CHS2 = 1;
    ADCON0bits.CHS3 = 0;
    TRISAbits.TRISA5 = 1;
    ANCON0 &= 0b11101111; //A0-RA0 as analog pin
    ANCON1 |= 0b00001111;

    //******************************************//
    //*********A/D module configurations********//

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

    //************************************************//
    //**********Begin reading the analog value********//

    GO_DONE = 1;
    while (GO_DONE);
    value = ((ADRESH << 8) + ADRESL);
    ADCON0bits.ADON = 0; //turning off the adc module

    //************************************************//
    return value; //returning the analog value just read
}

//***********************//
//*****Servo control*****//
void setServo(int degree) {
    servoPin_DIR = 0;
    //instead of using PWM signal only 20 pulses are used to control the servo
    for (int j = 0; j < 50; j++) {
        servoPin = 1; //Pulling high the servo signal pin
        for (int i = 0; i < 100; i++)__delay_us(2);                  //waiting for the offset time

        for (int i = 0; i < degree; i++)__delay_us(5); //waiting for the time needed to move to a certain  degree

        servoPin = 0; //pulling low the servo signal pin
        __delay_ms(20);
        
    }
}

//***********************************//
//****Ultrasonic distance sensor****//
double readDistance() {
    double a; //variable used to store the time
    
    __delay_ms(40);
    TMR1H = 0; //Sets the Initial Value of Timer
    TMR1L = 0; //Sets the Initial Value of Timer

    T1CONbits.TMR1CS0 = 0; //Setting the clock to Fosc/4
    T1CONbits.TMR1CS0 = 0;

    T1CONbits.T1CKPS0 = 1; //Setting the prescaler to 1:2
    T1CONbits.T1CKPS1 = 0;
    //Sending the Trigger signal to the sensor
    distanceTriggerPin = 1; //TRIGGER pin goes high
    __delay_us(10); //Delaying for 10uS as described in the technical document
    distanceTriggerPin = 0; //TRIGGER pin goes low

    while (!echoPin); //Waiting for the echo signal to go HIGH
    T1CONbits.TMR1ON = 1; //Start counting the time
    while (echoPin); //Waiting for the echo signal to return to the sensor
    T1CONbits.TMR1ON = 0; //Stop the timer

    a = (TMR1L | (TMR1H << 8)); //Reads the time from the timer registers
    a = (a * 43.6) / 0xffff; //and then convert in mS
    a = ((a * distanceCalibration) / 2)*1000; //now convert the time in distance
    a += 1.3; //calibration of the distance is done her

    return a; //return the distance in CM back
}

//*****************************//
//****Temperature functions****//
float readTempF(void) {
    float temp = 0; //variable used to store the temprature value
    for (int i = 0; i < 5; i++) //read the temprature value 5 times
        temp = (float) read(0) + temp; //add the read value to the previous one
    temp = temp / 5; //find the average
    temp = ((temp * 3.3) / 4096); //scale it to the voltage level
    temp = (18.181818 * temp) - 10; //and find the temprature in degree celsius

    return temp; //return the just read value back
}

//**********************//
//****Light functions****//
int readLight(void) {
    long j = read(1);
    j = ((j * 100) / 4096);
    return j;
}

//*******************************//
//****Accelerometer functions****//
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

// function used to read the accelerometer axis
float read_accelerometer(const unsigned int axis) {
    const unsigned int axis_point = read(axis);
    return pointToVoltage(axis_point);
}

//function used to scale the value to the correct voltage
float pointToVoltage(const unsigned int point) {
    const float fPoint = (float) point;
    return (3.3 * fPoint) / 4096;
}

float voltageToG(const float voltage) {
    return (voltage - zero_g_ref_voltage) / sensor_sensitivity;
}

//***********************//
//****reading button*****//
/*
char *readSwitch(void) {

    char array[8] = {0}; //array used to store the buttons state
    __delay_ms(10); //gice a little delay after the interrupt has fired

    switchDataPin_DIR = 0;      //DATA PIN Direction
    switchClockPin_DIR = 0;     //CLOCK PIN Direction
    switchLatchPin_DIR = 0;     //LATCH PIN Direction
    switchInputPin_DIR = 1;     //pinused to read the input

    switchDataPin = 0; //DATA PIN
    switchClockPin = 0; //CLOCK PIN
    switchLatchPin = 0; //LATCH PIN

    //pull low all the inputs
    for (int i = 0; i < 8; i++) {
        switchClockPin = 1;
        __delay_us(100);
        switchClockPin = 0;
        __delay_us(100);
    }

    //update the output register
    switchLatchPin = 1;
    __delay_us(100);
    switchLatchPin = 0;
    __delay_us(100);

    //scan the single buttons to determine whether it is pressed or not
    //and then save it in the array
    for (int i = 0; i < 8; i++) {
        if (i == 0) switchDataPin = 1; //DATA PIN
        else switchDataPin = 0; //DATA PIN
        switchClockPin = 1; //CLOCK PIN
        __delay_us(100);
        switchClockPin = 0;
        __delay_us(100);
        //update the output register
        switchLatchPin = 1;
        __delay_ms(10);
        switchLatchPin = 0;

        if (switchInputPin == 0)
            array[i] = 'L'; //save L if it is low
        else array[i] = 'H'; //save H if it is high
    }

    //pull high all the inputs for future readings
    switchDataPin = 1; //DATA PIN
    for (int i = 0; i < 8; i++) {
        switchClockPin = 1;
        __delay_us(100);
        switchClockPin = 0;
        __delay_us(100);
    }

    //update the output register
    switchLatchPin = 1;
    __delay_us(100);
    switchLatchPin = 0;
    __delay_us(100);

    while (switchInputPin); //wait until the button is unpressed
    return array; //return back the values as array
}
*/

void configSense(void)
{
    distanceTriggerPin_DIR = 0;
    echoPin_DIR = 1;
}
