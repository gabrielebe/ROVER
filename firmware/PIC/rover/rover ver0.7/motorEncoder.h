/* 
 * File:   motorEncoder.h
 * Author: Harpal
 *
 * Created on 14 ottobre 2014, 19.45
 */


#ifndef MOTORENCODER_H
#define	MOTORENCODER_H

#define encoderInput1 PORTBbits.RB6
#define encoderInput2 PORTBbits.RB5

#define inputAM1 LATBbits.LATB0
#define inputBM1 LATBbits.LATB1
#define inputAM2 LATBbits.LATB2
#define inputBM2 LATBbits.LATB3
#define pwmM1    CCPR4L
#define pwmM2    CCPR5L

#define ENCODER_SIGNAL_TO_TURN 3
#define MOTOR_REDUCTION 43.3
#define ENABLE_MOTOR 3

float encoder_signal_to_turn_reduction;
float degree_per_signal;

void MOTOR_init(void);
//void MOTOR_testMotors(void);

//void MOTOR_moveTest(void);
void MOTOR_stop(void);
void MOTOR_fastStop(void);
void MOTOR_forwardT(int pwm, int time);
void MOTOR_forward(int pwm);
void MOTOR_backwardT(int pwm,int time);
void MOTOR_backward(int pwm);
void MOTOR_turnRightT(int pwm, int time);
void MOTOR_turnLeftT(int pwm,int time);
void MOTOR_turnRight(int pwm);
void MOTOR_turnLeft(int pwm);

float encoder_move_degree(float degree_to_do, int pwm, char direction);
#endif	/* MOTORENCODER_H */

