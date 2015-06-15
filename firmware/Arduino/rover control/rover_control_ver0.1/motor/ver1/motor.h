#ifndef MOTOR_H_INCLUDED
#define MOTOR_H_INCLUDED

/* Include Arduino library and defines */
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Hardware constants */
#define DEBUG_SERIAL
//#define DEBUG_LCD
#define BAUD_RATE 115200

/* Compass constants */
#define COMPASS_ADDRESS                (0x1E)          /* 7 bit address of the HMC58X3 used with the Wire library */
#define COMPASS_GAUSS_LSB              (0.73F)         /* Varies with gain */
#define COMPASS_PERIODE_MEASURE        56

/* User constants */
#define HEADING_LIMIT 2
#define SX 0
#define DX 1

/* Motor direction command define */
typedef enum motorDirection
{
  NONE,
  TURN,
  FOREWARD,
  BACKWARD,
  STOP,
  RIGHT_ANGLE,
  RIGHT,
  LEFT_ANGLE,
  LEFT,
} MOTOR_COMMAND;

/* Motor pins defines */
typedef enum motorPins
{
  MOTOR_PIN_AM1  =  2,
  MOTOR_PIN_BM1  =  3,
  MOTOR_PIN_AM2  =  4,
  MOTOR_PIN_BM2  =  5,
  MOTOR_PIN_AM3  =  13,
  MOTOR_PIN_BM3  =  7,
  MOTOR_PIN_AM4  =  8,
  MOTOR_PIN_BM4  =  12,
  MOTOR_PIN_EN1  =  6,
  MOTOR_PIN_EN2  =  9,
  MOTOR_PIN_EN3  =  10,
  MOTOR_PIN_EN4  =  11,
} MOTOR_PIN;

/* Register address define */
typedef enum registerEnum
{
  COMPASS_REG_A             =  0x00,
  COMPASS_REG_B             =  0x01,
  COMPASS_MODE_REG          =  0x02,
  COMPASS_DATA_OUT_X_MSB    =  0x03,
  COMPASS_DATA_OUT_X_LSB    =  0x04,
  COMPASS_DATA_OUT_Z_MSB    =  0x05,
  COMPASS_DATA_OUT_Z_LSB    =  0x06,
  COMPASS_DATA_OUT_Y_MSB    =  0x07,
  COMPASS_DATA_OUT_Y_LSB    =  0x08,
  COMPASS_STATUS_REGISTER   =  0x09,
  COMPASS_IDENTIFICATION_A  =  0x0A,
  COMPASS_IDENTIFICATION_B  =  0x0B,
  COMPASS_IDENTIFICATION_C  =  0x0C,
} REGISTER;

unsigned long magnLastTime = 0;
int correctionDir = SX;

void controlTest(void);
void motorBaseTest(void);
void moveControl(motorDirection command, int pwm);

void motorInit(void);
void motorMove(motorDirection comand, int pwm, int angle);
void motorTurn(int angle, int pwm);
void motorForward(int pwm);
void motorBackward(int pwm);
void motorStop(void);
void motorRight(int pwm, int angle);
void motorTurnRight(int pwm);
void motorLeft(int pwm, int angle);
void motorTurnLeft(int pwm);

void compassInit(void);
float headingDetect(void);
void compassCompensate(float compassData[3]);
void readMagneticAxis(float rawData[3]);
void write8(byte address, registerEnum reg, byte value);
byte read8(int address, registerEnum reg);

#endif

