#include"motorEncoder.h"

#include <stdio.h>
#include <stdlib.h>
#include "functions.h"
#include "lcd.h"
#include "config.h"
#include <pic18f47j53.h>
#include <xc.h>

void MOTOR_init(void) {
    // Initializing the OUTPUT control array
    // Optional pulling down all the inputs for security
    TRISB = 0b01000000;
    LATB = 0x00;
    PR2 = 0xff; //(12M/(4*1*10K))-1 = 256
    //Setting up PWM module
    pwmM1 = 0x0f;
    CCP4CONbits.DC4B0 = 1;
    CCP4CONbits.DC4B1 = 1;
    LATBbits.LATB4 = 0;
    T2CONbits.T2CKPS0 = 0;
    T2CONbits.T2CKPS1 = 0;
    T2CONbits.TMR2ON = 1;
    CCP4CONbits.CCP4M2 = 1;
    CCP4CONbits.CCP4M3 = 1;

    CCPTMRS1bits.C5TSEL0 = 1; //selecting TMR4
    PR4 = 0xff; //(12M/(4*1*10K))-1 = 256
    //Setting up PWM module
    pwmM2 = 0x0f;
    CCP5CONbits.DC5B0 = 1;
    CCP5CONbits.DC5B1 = 1;
    LATBbits.LATB5 = 0;
    T4CONbits.T4CKPS0 = 0;
    T4CONbits.T4CKPS1 = 0;
    T4CONbits.TMR4ON = 1;
    CCP5CONbits.CCP5M2 = 1;
    CCP5CONbits.CCP5M3 = 1;
    pwmM1 = 0;
    pwmM2 = 0;
    init_encoder_variab();
    encoder_signal_to_turn_reduction = MOTOR_REDUCTION * ENCODER_SIGNAL_TO_TURN;
    degree_per_signal = 360 / encoder_signal_to_turn_reduction;
}

void MOTOR_testMotors(void) {
    lcdClear();
    lcdWriteStrC("Test Clockwise ");

    pwmM1 = 1;
    inputAM1 = 1;
    inputBM1 = 0;

    pwmM2 = 1;
    inputAM2 = 1;
    inputBM2 = 0;

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

    for (int i = 0; i < 50; i++) __delay_ms(20);

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

    pwmM1 = 0;
    pwmM2 = 0;
    lcdClear();
    lcdWriteStrC("   Motor TEST   ");
    lcdSetPos(0, 1);
    lcdWriteStrC("     passed     ");
    for (int i = 0; i < 100; i++)__delay_ms(10);
    lcdClean();
}

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

void MOTOR_stop(void) {
    pwmM1 = 0;
    pwmM2 = 0;
    inputAM1 = 0;
    inputBM1 = 0;
    inputAM2 = 0;
    inputBM2 = 0;
    //lcdClear();

}

void MOTOR_fastStop(void) {
    pwmM1 = 255;
    pwmM2 = 255;
    inputAM1 = 0;
    inputBM1 = 0;
    inputAM2 = 0;
    inputBM2 = 0;
    //lcdClear();

}

void MOTOR_forwardT(int pwm, int time) {
    MOTOR_forward(pwm);
    for (int i = 0; i < time; i++)
        for (int j = 0; j < 100; j++) __delay_ms(10);
    MOTOR_stop();
}

void MOTOR_forward(int pwm) {
    char buffer [10];
    lcdClear();
    lcdWriteStrC(" Forward speed: ");
    lcdSetPos(0, 1);
    lcdWriteStrC("     ");
    sprintf(buffer, "%05d", pwm);
    lcdWriteStrC(buffer);

    inputAM1 = 1;
    inputBM1 = 0;
    inputAM2 = 1;
    inputBM2 = 0;

    pwmM1 = pwm;
    pwmM2 = pwm;
}

void MOTOR_backwardT(int pwm, int time) {
    MOTOR_backward(pwm);
    for (int i = 0; i < time; i++)
        for (int j = 0; j < 100; j++) __delay_ms(10);
    MOTOR_stop();

}

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

void MOTOR_turnRightT(int pwm, int time) {

    MOTOR_turnRight(pwm);
    for (int i = 0; i < time; i++)
        for (int j = 0; j < 100; j++) __delay_ms(10);
    MOTOR_stop();

}

void MOTOR_turnLeftT(int pwm, int time) {
    MOTOR_turnLeft(pwm);
    for (int i = 0; i < time; i++)
        for (int j = 0; j < 100; j++) __delay_ms(10);
    MOTOR_stop();
}

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
//************************************//
//************************************//
//************************************//

float encoder_move_degree(float degree_to_do, int pwm, char direction) {

    float degree_done = 0;
    switch (direction) {
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

    MOTOR_fastStop();
    const float error_degree = degree_done - degree_to_do;
    return error_degree;
}

void init_encoder_variab(void) {
    
}