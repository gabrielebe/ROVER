#include <stdio.h>              //Standard c library for IO
#include <stdlib.h>             //Standard c library
#include <xc.h>                 //XC compiler library
#include <pic18f47j53.h>        //pic microcontroller library

#include "lcd.h"                //library used for the lcd control
#include "motorEncoder.h"       //library used for the motor and encoder control

//this function is used to initialize the motor functions
void MOTOR_init(void) {
    //Setting the pin directions to initialize the motor
    encoderInput1_DIR = 1;
    encoderInput2_DIR = 1;
    inputAM1_DIR = 0;
    inputBM1_DIR = 0;
    inputAM2_DIR = 0;
    inputBM2_DIR = 0;
    //pulling down all the input for security reasons
    inputAM1 = 0;
    inputBM1 = 0;
    inputAM2 = 0;
    inputBM2 = 0;

    //Setting up the first PWM module
    //calculating the PR2 value: PR2 is the compare value
    PR2 = 0xff; //(12M/(4*1*10K))-1 = 256
    pwmM1 = 0x0f;               //the value is set in the middle initially
    //doing all the setting for the pwm module
    CCP4CONbits.DC4B0 = 1;
    CCP4CONbits.DC4B1 = 1;
    TRISBbits.TRISB4 = 0;
    T2CONbits.T2CKPS0 = 0;
    T2CONbits.T2CKPS1 = 0;
    T2CONbits.TMR2ON = 1;
    CCP4CONbits.CCP4M2 = 1;
    CCP4CONbits.CCP4M3 = 1;

    //Setting up the second PWM module
    CCPTMRS1bits.C5TSEL0 = 1;   //selecting TMR4 as time counter
    PR4 = 0xff;                 //(12M/(4*1*10K))-1 = 256
    pwmM2 = 0x0f;               //the value is set in the middle initially
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
    pwmM1 = 0;                  //Set both pwm signals to 0 this is done
    pwmM2 = 0;                  //because we don't want any movement
    //calculation of these value is done only once at the beginning
    //and are necessary for the control of the movement angle
    encoder_signal_to_turn_reduction = MOTOR_REDUCTION * ENCODER_SIGNAL_TO_TURN;
    degree_per_signal = 360 / encoder_signal_to_turn_reduction;
}
//function is used initially for testing the motors but it won't be implemented
//in the main file
void MOTOR_testMotors(void) {
    lcdClear();                     //clearing the lcd
    lcdWriteStrC("Test Clockwise ");//displaying on the lcd what does the program is doing
    //setting the inputs correctly in order to go foreward
    inputAM1 = 1;
    inputBM1 = 0;

    inputAM2 = 1;
    inputBM2 = 0;
    //display on the lcd that the motors are being accelerating
    lcdSetPos(0, 1);
    lcdWriteStrC("  Accelerating  ");
    //slowly increasing the pwm signal to accelerate the motor speed
    for (int i = 0; i < 255; i++) {
        pwmM1 = i;
        pwmM2 = i;
        __delay_ms(20);
    }
    //maintaining the speed and displaying it on the lcd
    lcdSetPos(0, 1);
    lcdWriteStrC("    Maintain    ");
    for (int i = 0; i < 50; i++) __delay_ms(20);
    //display on the lcd that the motors are being decelerating
    lcdSetPos(0, 1);
    lcdWriteStrC("  Decelerating  ");
    //slowly decreasing the pwm signal to decelerate the motor speed
    for (int i = 255; i > 0; i--) {
        pwmM1 = i;
        pwmM2 = i;
        __delay_ms(20);
    }

    //doing the same test again but with opposite direction
    lcdClear();
    lcdWriteStrC("Test Anticlockwise ");

    inputAM1 = 0;
    inputBM1 = 1;

    inputAM2 = 0;
    inputBM2 = 1;

    lcdSetPos(0, 1);
    lcdWriteStrC("  Accelerating  ");
    for (int i = 0; i < 255; i++) {
        pwmM1 = i;
        pwmM2 = i;
        __delay_ms(20);
    }

    lcdSetPos(0, 1);
    lcdWriteStrC("    Maintain    ");
    for (int i = 0; i < 50; i++) __delay_ms(20);

    lcdSetPos(0, 1);
    lcdWriteStrC("  Decelerating  ");
    for (int i = 255; i > 0; i--) {
        pwmM1 = i;
        pwmM2 = i;
        __delay_ms(20);
    }
    //once finished the test stop the motors
    MOTOR_stop();
    //then clear the lcd and display that the test has been finished
    lcdClear();
    lcdWriteStrC("   Motor TEST   ");
    lcdSetPos(0, 1);
    lcdWriteStrC("     passed     ");
    for (int i = 0; i < 100; i++)__delay_ms(10);//maintain the display for some time
    lcdClean();                                 //and then clear it
}

//This test is actually used when the motors are being installed on the mecchanical structur
void MOTOR_moveTest(void) {
    lcdClear();
    lcdWriteStrC("   Motor move   ");
    lcdSetPos(0, 1);
    lcdWriteStrC("     testing    ");

    MOTOR_forwardT(100, 5);
    for (int i = 0; i < 5000; i++)
        for (int j = 0; j < 100; j++) __delay_ms(10);

    MOTOR_forward(100);
    for (int i = 0; i < 5000; i++)
        for (int j = 0; j < 100; j++) __delay_ms(10);
    MOTOR_stop();

    MOTOR_backwardT(100, 5);
    for (int i = 0; i < 5000; i++)
        for (int j = 0; j < 100; j++) __delay_ms(10);

    MOTOR_backward(100);
    for (int i = 0; i < 5000; i++)
        for (int j = 0; j < 100; j++) __delay_ms(10);
    MOTOR_stop();

    MOTOR_turnRightT(100, 5);
    for (int i = 0; i < 5000; i++)
        for (int j = 0; j < 100; j++) __delay_ms(10);

    MOTOR_turnLeftT(100, 5);
    for (int i = 0; i < 5000; i++)
        for (int j = 0; j < 100; j++) __delay_ms(10);

    MOTOR_backward(100);
    for (int i = 0; i < 5000; i++)
        for (int j = 0; j < 100; j++) __delay_ms(10);
    MOTOR_stop();
}

//this function is usedd to stop the motors
void MOTOR_stop(void) {
    pwmM1 = 0;
    pwmM2 = 0;
    inputAM1 = 0;
    inputBM1 = 0;
    inputAM2 = 0;
    inputBM2 = 0;
}

//this function is used to stop the motors fast
void MOTOR_fastStop(void) {
    pwmM1 = 255;
    pwmM2 = 255;
    inputAM1 = 0;
    inputBM1 = 0;
    inputAM2 = 0;
    inputBM2 = 0;
}

//this function is used to move the robot forward for a certain time and at a certain velocity
void MOTOR_forwardT(int pwm, int time) {
    MOTOR_forward(pwm);             //move the robot forward with the give value of the PWM signal
    for (int i = 0; i < time; i++)  //maintain the velocity for certain time
        for (int j = 0; j < 100; j++) __delay_ms(10);
    MOTOR_stop();                   //and finally stop the motors
}

//this function is used to move forward the robot at a certain velocity
void MOTOR_forward(int pwm) {
    //the following part is used only to display on the lcd the speed and the
    //movement that the robot is doing
    char buffer [10];
    lcdClean();
    lcdWriteStrC(" Forward speed: ");
    lcdSetPos(0, 1);
    lcdWriteStrC("     ");
    sprintf(buffer, "%05d", pwm);
    lcdWriteStrC(buffer);

    //setting the inputs in order to go forward
    inputAM1 = 1;
    inputBM1 = 0;
    inputAM2 = 1;
    inputBM2 = 0;
    //setting the pwm signal given as a parameter to this function
    pwmM1 = pwm;
    pwmM2 = pwm;
}
//this function is the same as the previous one except it moves the robot backward
void MOTOR_backwardT(int pwm, int time) {
    MOTOR_backward(pwm);
    for (int i = 0; i < time; i++)
        for (int j = 0; j < 100; j++) __delay_ms(10);
    MOTOR_stop();
}
//this function is used to move backward the robot at a certain velocity
void MOTOR_backward(int pwm) {
    char buffer [10];
    lcdClear();
    lcdWriteStrC(" Forward speed: ");
    lcdSetPos(0, 1);
    lcdWriteStrC("     ");
    sprintf(buffer, "%05d", pwm);
    lcdWriteStrC(buffer);

    inputAM1 = 0;
    inputBM1 = 1;
    inputAM2 = 0;
    inputBM2 = 1;

    pwmM1 = pwm;
    pwmM2 = pwm;

}

//this function is used to turn right the robot for a given time and velocity
void MOTOR_turnRightT(int pwm, int time) {
    MOTOR_turnRight(pwm);
    for (int i = 0; i < time; i++)
        for (int j = 0; j < 100; j++) __delay_ms(10);
    MOTOR_stop();

}
//this function is used to turn left the robot for a given time and velocity
void MOTOR_turnLeftT(int pwm, int time) {
    MOTOR_turnLeft(pwm);
    for (int i = 0; i < time; i++)
        for (int j = 0; j < 100; j++) __delay_ms(10);
    MOTOR_stop();
}
//this function is used to turn right the robot at a certain velocity
void MOTOR_turnRight(int pwm) {
    char buffer [10];
    lcdClear();
    lcdWriteStrC("turn right speed");
    lcdSetPos(0, 1);
    lcdWriteStrC("     ");
    sprintf(buffer, "%05d", pwm);
    lcdWriteStrC(buffer);

    pwmM1 = 0;
    pwmM2 = 0;
    inputAM1 = 1;
    inputBM1 = 0;
    pwmM1 = pwm;
}
//this function is used to turn left the robot at a certain velocity
void MOTOR_turnLeft(int pwm) {
    char buffer [10];
    lcdClear();
    lcdWriteStrC("turn left speed");
    lcdSetPos(0, 1);
    lcdWriteStrC("     ");
    sprintf(buffer, "%05d", pwm);
    lcdWriteStrC(buffer);

    pwmM1 = 0;
    pwmM2 = 0;
    inputAM2 = 0;
    inputBM2 = 1;
    pwmM2 = pwm;

}

//*************************//
//****encoder function*****//
//this function is used to move the motor at certain degree
float encoder_move_degree(float degree_to_do, int pwm, char direction) {
    float degree_done = 0;      //this variable is used to store the value of how many degree has the motor done
    switch (direction) {        //depending on the direction move in the right way
        case 1: MOTOR_forward(pwm);
            while (degree_done <= degree_to_do) {
                while (!encoderInput1);
                degree_done += degree_per_signal;
                while (encoderInput1);
            }
            break;
        case 2: MOTOR_backward(pwm);
            while (degree_done <= degree_to_do) {
                while (!encoderInput1);
                degree_done += degree_per_signal;
                while (encoderInput1);
            }
            break;
        case 3: MOTOR_turnLeft(pwm);
            while (degree_done <= degree_to_do) {
                while (!encoderInput1);
                degree_done += degree_per_signal;
                while (encoderInput1);
            }
            break;
        case 4: MOTOR_turnRight(pwm);
            while (degree_done <= degree_to_do) {
                while (!encoderInput2);
                degree_done += degree_per_signal;
                while (encoderInput2);
            }
            break;
    }
    MOTOR_fastStop();           //once finished stop the motors
    const float error_degree = degree_done - degree_to_do;  //calculate the error of movement
    return error_degree;        //send back the error that has been committed
}
