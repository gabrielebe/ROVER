#ifndef JHONNY_H_INCLUDED
#define JHONNY_H_INCLUDED

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define DEBUG
#define BAUD_RATE 9600

#define ANALOG_DELTA_MEASURE_TIME 10000
#define ANALOG_accelerometerX 0
#define ANALOG_accelerometerY 1
#define ANALOG_accelerometerZ 2
#define ANALOG_phi 0
#define ANALOG_teta 1

#define NONE 0
#define TURN 1
#define FOREWARD 2
#define BACKWARD 3
#define STOP 4
#define RIGHT 5
#define LEFT 6

#define MOTOR_PIN_AM1 1
#define MOTOR_PIN_BM1 2
#define MOTOR_PIN_AM2 3
#define MOTOR_PIN_BM2 4
#define MOTOR_PIN_AM3 5
#define MOTOR_PIN_BM3 6
#define MOTOR_PIN_AM4 7
#define MOTOR_PIN_BM4 8
#define MOTOR_PIN_EN1 9
#define MOTOR_PIN_EN2 10
#define MOTOR_PIN_EN3 11
#define MOTOR_PIN_EN4 12

#define PING_SERVO_PIN 9
#define PING_SENSOR_PIN 7
#define HEADING_LIMIT 5
#define DISTANCE_LIMIT 30
#define SENSOR_GAP 0.2

#define SX 0
#define DX 1


void ARDUINO_init(void);
void ARDUINO_comunicationInit(void);

void ANALOG_init(void);  /* */
float ANALOG_temperatureRead(void);  /* */
float ANALOG_luxRead(void);  /* */
float* ANALOG_accelerometerRead(void);  /* */
float* ANALOG_tiltSensor(void);  /* */

void COMPASS_init(void);  /* */
float COMPASS_headingDetect(void);
void COMPASS_readValue(void);  /* */

void MOTOR_init(void);
void MOTOR_move(int comand, int pwm, int angle);
void MOTOR_turn(int angle, int pwm);
void MOTOR_forward(int pwm);
void MOTOR_backward(int pwm);
void MOTOR_stop(void);
void MOTOR_right(int pwm, int angle);
void MOTOR_turnRight(int pwm);
void MOTOR_left(int pwm, int angle);
void MOTOR_turnLeft(int pwm);

float ULTRASONIC_readDistance(void);
void ROBOT_moveControl(void);
int ROBOT_dirToMove(void);


boolean ANALOG_tempMeasurated = 0;
unsigned long ANALOG_lastMeasurement = millis();
boolean COMPASS_newMeasure = 0;
float COMPASS_X, COMPASS_Y, COMPASS_Z;
int correctionDir = SX;
float previewHeading = 0.0;
float previewError = 0.0;

#endif // JHONNY_H_INCLUDED

