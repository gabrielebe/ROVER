/* 
 * File:   Motor.h
 * Author: Harpal
 *
 * Created on 25 gennaio 2015, 19.19
 */

#ifndef MOTOR_H
#define	MOTOR_H

#ifndef inputAM1
#define inputAM1            (PORTBbits.RB0)
#define inputAM1_DIR        (TRISBbits.TRISB0)
#endif

#ifndef inputBM1
#define inputBM1            (PORTBbits.RB1)
#define inputBM1_DIR        TRISBbits.TRISB1
#endif

#ifndef inputAM2
#define inputAM2            PORTBbits.RB2
#define inputAM2_DIR        TRISBbits.TRISB2
#endif

#ifndef inputBM2
#define inputBM2            PORTBbits.RB3
#define inputBM2_DIR        TRISBbits.TRISB3
#endif

#ifndef inputAM3
#define inputAM3            (PORTBbits.RB0)
#define inputAM3_DIR        (TRISBbits.TRISB0)
#endif

#ifndef inputBM3
#define inputBM3            (PORTBbits.RB1)
#define inputBM3_DIR        TRISBbits.TRISB1
#endif

#ifndef inputAM4
#define inputAM4            PORTBbits.RB2
#define inputAM4_DIR        TRISBbits.TRISB2
#endif

#ifndef inputBM4
#define inputBM4            PORTBbits.RB3
#define inputBM4_DIR        TRISBbits.TRISB3
#endif

void MOTOR_init(void);
void pinInit(void);
void pwmInit(void);
void setPwm1(int pwm);
void setPwm2(int pwm);

void MOTOR_stop(void);
void MOTOR_fastStop(void);
void MOTOR_forwardT(int pwm, int time);
void MOTOR_forward(int pwm);
void MOTOR_backwardT(int pwm, int time);
void MOTOR_backward(int pwm);
void MOTOR_turnRightT(int pwm, int time);
void MOTOR_turnRight(int pwm);
void MOTOR_turnLeftT(int pwm, int time);
void MOTOR_turnLeft(int pwm);

#endif	/* MOTOR_H */
