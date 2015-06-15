// Ensure this library description is only included once
#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

#define VERSION "MOTOR ver1.2"      // Versione del firmware
#define PWM_RESOLUTION 256          // Resolution of 8 bit Arduino ADC
#define BAUD_RATE 9600              // Velocità comunicazione seriale espressa in baud
#define NUM_MOTOR_OUTPUT_PIN 6      // Numero di pin di output per l'interfacciamento con entrambi i motori
#define LCD_COL 16                  // Colonne dell'LCD
#define LCD_ROW 2                   // Righe dell'LCD

// Input pin on the L298 to control the motors
#define enableM1 3
#define inp1M1 2
#define inp2M1 4
#define enableM2 5
#define inp1M2 7
#define inp2M2 8

const unsigned int motorPinOutput[NUM_MOTOR_OUTPUT_PIN] = {enableM1,
                                                           inp1M1,
                                                           inp2M1,
                                                           enableM2,
                                                           inp1M2,
                                                           inp2M2
                                                          };

void LCD_init(void);
void MOTOR_init(void);
void MOTOR_testMotors(void);
void MOTOR_testMotor(const unsigned int input1Pin, const unsigned int input2Pin, const unsigned int enablePin);
void MOTOR_moveTest(void);
void MOTOR_forward(const unsigned int pwm);
void MOTOR_forwardT(const unsigned int pwm, const unsigned int time_s);
void MOTOR_backward(const unsigned int pwm);
void MOTOR_backwardT(const unsigned int pwm, const unsigned int time_s);
void MOTOR_stop(void);
void MOTOR_turnRightT(const unsigned int pwm, const unsigned int time_s);
void MOTOR_turnLeftT(const unsigned int pwm, const unsigned int time_s);

#endif

