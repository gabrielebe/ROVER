#include <stdio.h>              //Standard c library for IO
#include <stdlib.h>             //Standard c library
#include <xc.h>                 //XC compiler library
#include <pic16f886.h>          //pic microcontroller library
#include "Motor.h"              //library used for the motor and encoder control

#ifdef LCD
#include "lcd.h"                //library used for the lcd control
#endif

#define _XTAL_FREQ 8000000
//this function is used to initialize the motor functions



void MOTOR_init(void) 
{
    pinInit();
    pwmInit();
}


void pinInit(void)
{
    //Setting the pin directions to initialize the motor
    inputAM1_DIR = 0;
    inputBM1_DIR = 0;
    inputAM2_DIR = 0;
    inputBM2_DIR = 0;
    inputAM3_DIR = 0;
    inputBM3_DIR = 0;
    inputAM4_DIR = 0;
    inputBM4_DIR = 0;

    //pulling down all the input for security reasons
    inputAM1 = 0;
    inputBM1 = 0;
    inputAM2 = 0;
    inputBM2 = 0;
    inputAM3 = 0;
    inputBM3 = 0;
    inputAM4 = 0;
    inputBM4 = 0;
}


void pwmInit(void)
{
    /* Setting up the PWM module */

    //disable PWM module
    TRISCbits.TRISC1 = 1;
    TRISCbits.TRISC2 = 1;

    //setting the PR2 value: PR2 is the compare value
    PR2 = 0x41;   //setting for 8 bit resolution

    //configure CCP1 in PWM mode
    CCP1CONbits.P1M0 = 0;
    CCP1CONbits.P1M1 = 0;
    CCP1CONbits.CCP1M0 = 0;
    CCP1CONbits.CCP1M1 = 0;
    CCP1CONbits.CCP1M2 = 1;
    CCP1CONbits.CCP1M3 = 1;
    PWM1CONbits.PRSEN = 1;

    //configure CCP2 in PWM mode
    CCP2CONbits.CCP2M3 = 1;
    CCP2CONbits.CCP2M2 = 1;
    CCP2CONbits.CCP2M1 = 1;
    CCP2CONbits.CCP2M0 = 1;

    //set firs PWM value
    setPwm2(128);
    setPwm1(128);

    //enable PWM module
    T2CONbits.TMR2ON = 1;   //abilitation of timer2


    /*pwmM1 = 0x0f; //the value is set in the middle initially
    //doing all the setting for the pwm module
    CCP1CONbits.DC1B0 = 1;
    CCP1CONbits.DC1B1 = 1;
    TRISCbits.TRISC2 = 0;
    T2CONbits.T2CKPS0 = 0;
    T2CONbits.T2CKPS1 = 0;
    T2CONbits.TMR2ON = 1;
    CCP4CONbits.CCP4M2 = 1;
    CCP4CONbits.CCP4M3 = 1;

    //Setting up the second PWM module
    CCPTMRS1bits.C5TSEL0 = 1; //selecting TMR4 as time counter
    PR4 = 0xff; //(12M/(4*1*10K))-1 = 256
    pwmM2 = 0x0f; //the value is set in the middle initially
    //doing all the setting for the pwm module
    CCP5CONbits.DC5B0 = 1;
    CCP5CONbits.DC5B1 = 1;
    TRISBbits.TRISB5 = 0;
    T4CONbits.T4CKPS0 = 0;
    T4CONbits.T4CKPS1 = 0;
    T4CONbits.TMR4ON = 1;
    CCP5CONbits.CCP5M2 = 1;
    CCP5CONbits.CCP5M3 = 1;
    //finished all the configurtion and turned on both PWM modules
    pwmM1 = 0; //Set both pwm signals to 0 this is done
    pwmM2 = 0; //because we don't want any movement*/
}


void setPwm1(int pwm)
{
    if(pwm <= 255)
    {
        CCP1CONbits.DC1B0 =  pwm & 0b00000001;
        CCP1CONbits.DC1B1 = (pwm >> 1) & 0b00000001;
        CCPR1L = pwm >> 2;
    }
}


void setPwm2(int pwm)
{
    if(pwm <= 255)
    {
        CCP2CONbits.DC2B0 =  pwm & 0b00000001;
        CCP2CONbits.DC2B1 = (pwm >> 1) & 0b00000001;
        CCPR2L = pwm >> 2;
    }
}


// This function is usedd to stop the motors
void MOTOR_stop(void) {
    setPwm1(0);
    setPwm2(0);

    inputAM1 = 0;
    inputBM1 = 0;
    inputAM2 = 0;
    inputBM2 = 0;
    inputAM3 = 0;
    inputBM3 = 0;
    inputAM4 = 0;
    inputBM4 = 0;
}


// This function is used to stop the motors fast
void MOTOR_fastStop(void) {
    setPwm1(255);
    setPwm2(255);

    inputAM1 = 1;
    inputBM1 = 1;
    inputAM2 = 1;
    inputBM2 = 1;
    inputAM3 = 1;
    inputBM3 = 1;
    inputAM4 = 1;
    inputBM4 = 1;
}


// This function is used to move the robot forward for a certain time and at a certain velocity
void MOTOR_forwardT(int pwm, int time) {
    MOTOR_forward(pwm); //move the robot forward with the give value of the PWM signal
    
    for (int i = 0; i < time; i++) //maintain the velocity for certain time
        for (int j = 0; j < 100; j++) __delay_ms(10);

    MOTOR_stop(); //and finally stop the motors
}


// This function is used to move forward the robot at a certain velocity
void MOTOR_forward(int pwm) {
#ifdef LCD
    //the following part is used only to display on the lcd the speed and the
    //movement that the robot is doing
    char buffer [10];
    lcdClean();
    lcdWriteStrC(" Forward speed: ");
    lcdSetPos(0, 1);
    lcdWriteStrC("     ");
    sprintf(buffer, "%05d", pwm);
    lcdWriteStrC(buffer);
#endif

    //setting the inputs in order to go forward
    inputAM1 = 1;
    inputBM1 = 0;
    inputAM2 = 1;
    inputBM2 = 0;
    inputAM3 = 1;
    inputBM3 = 0;
    inputAM4 = 1;
    inputBM4 = 0;

    //setting the pwm signal given as a parameter to this function
    setPwm1(pwm);
    setPwm2(pwm);
}


// This function is the same as the previous one except it moves the robot backward
void MOTOR_backwardT(int pwm, int time) {
    MOTOR_backward(pwm);

    for (int i = 0; i < time; i++)
        for (int j = 0; j < 100; j++) __delay_ms(10);

    MOTOR_stop();
}


// This function is used to move backward the robot at a certain velocity
void MOTOR_backward(int pwm) {
#ifdef LCD
    char buffer [10];
    lcdClear();
    lcdWriteStrC(" Forward speed: ");
    lcdSetPos(0, 1);
    lcdWriteStrC("     ");
    sprintf(buffer, "%05d", pwm);
    lcdWriteStrC(buffer);
#endif

    inputAM1 = 0;
    inputBM1 = 1;
    inputAM2 = 0;
    inputBM2 = 1;
    inputAM3 = 0;
    inputBM3 = 1;
    inputAM4 = 0;
    inputBM4 = 1;

    setPwm1(pwm);
    setPwm2(pwm);
}


// This function is used to turn right the robot for a given time and velocity
void MOTOR_turnRightT(int pwm, int time) {
    MOTOR_turnRight(pwm);

    for (int i = 0; i < time; i++)
        for (int j = 0; j < 100; j++) __delay_ms(10);
    
    MOTOR_stop();
}


// This function is used to turn right the robot at a certain velocity
void MOTOR_turnRight(int pwm) {
#ifdef LCD
    char buffer [10];
    lcdClear();
    lcdWriteStrC("turn right speed");
    lcdSetPos(0, 1);
    lcdWriteStrC("     ");
    sprintf(buffer, "%05d", pwm);
    lcdWriteStrC(buffer);
#endif
    
    setPwm1(0);
    setPwm2(0);

    inputAM1 = 1;
    inputBM1 = 0;
    inputAM2 = 0;
    inputBM2 = 1;
    inputAM3 = 1;
    inputBM3 = 0;
    inputAM4 = 0;
    inputBM4 = 1;

    setPwm1(pwm);
    setPwm2(pwm);
}


// This function is used to turn left the robot for a given time and velocity
void MOTOR_turnLeftT(int pwm, int time) {
    MOTOR_turnLeft(pwm);

    for (int i = 0; i < time; i++)
        for (int j = 0; j < 100; j++) __delay_ms(10);

    MOTOR_stop();
}


// This function is used to turn left the robot at a certain velocity
void MOTOR_turnLeft(int pwm) {
#ifdef LCD
    char buffer [10];
    lcdClear();
    lcdWriteStrC("turn left speed");
    lcdSetPos(0, 1);
    lcdWriteStrC("     ");
    sprintf(buffer, "%05d", pwm);
    lcdWriteStrC(buffer);
#endif

    setPwm1(0);
    setPwm2(0);

    inputAM1 = 0;
    inputBM1 = 1;
    inputAM2 = 1;
    inputBM2 = 0;
    inputAM3 = 0;
    inputBM3 = 1;
    inputAM4 = 1;
    inputBM4 = 0;

    setPwm1(pwm);
    setPwm2(pwm);
}
