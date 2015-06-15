#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED


// Include Arduino library and defines
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define BAUD_RATE       38400
#define SERIAL_DEBUG
#define DEBUG_LCD
#define RAD_TO_DEG      (57.295779513082320876798154814105F)
#define GRAVITY_EARTH   (9.80665F)

// Accelerometer constant
#define ZERO_G_REF_VOLTAGE        (1.65F)
#define SENSOR_SENTITIVITY        (0.300F)
#define ITERATION_POINT           (5.0F)
#define ALPHA 				      (0.3F)		// Smoothing factor of low pass filter

// Compass constants
#define COMPASS_ADDRESS           (0x1E)      // 7 bit address of the HMC58X3 used with the Wire library
#define COMPASS_GAUSS_LSB         (0.73F)     // Varies with gain
#define COMPASS_PERIODE_MEASURE   56

// Pin constants define
#define X_ASS       A0
#define Y_ASS       A1
#define Z_ASS       A2
#define TEMP_PIN    A3
#define SERVO_PIN   22
#define PING_PIN    23
#define ECHO_PIN    24
#define TRIG_PIN    25

// Motor pins defines
#define MOTOR_PIN_AM1 26
#define MOTOR_PIN_BM1 27
#define MOTOR_PIN_AM2 28
#define MOTOR_PIN_BM2 29
#define MOTOR_PIN_AM3 30
#define MOTOR_PIN_BM3 31
#define MOTOR_PIN_AM4 32
#define MOTOR_PIN_BM4 33
#define MOTOR_PIN_EN1 2
#define MOTOR_PIN_EN2 3
#define MOTOR_PIN_EN3 4
#define MOTOR_PIN_EN4 5

// User constants
#define HEADING_LIMIT           2
#define ULTRASONIC_ITERATIONS   (3.0F)
#define SX                      0
#define DX                      1
#define FORE_SENSOR             0
#define BACK_SENSOR             1


// Compass register address define
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


// Motor direction command define
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


// Global variables
float x0gCurrent, y0gCurrent, z1gCurrent;
float temperature;
int correctionDir = SX;

unsigned long tempLastTime = 0;
unsigned long distLastTime = 0;
unsigned long magnLastTime = 0;

float xPreviewVelocity = 0.0;
float yPreviewVelocity = 0.0;
float xPreviewDistance = 0.0;
float yPreviewDistance = 0.0;
int tDelay = 500;
float dt = (float)tDelay / 1000000;


// functione prototipes
/*void cinematicSerialPrint(void);
void cinematicRead(float cinematicData[4]);*/
void printTilt(void);
void accelerationMeasuramentPrint(void);
void getTilt(float angles[2]);
void gAccelerationMeasuramentRead(float acc[3]);
void accelerationMediumRead(float gAcc[3], float acc[3]);
float singleAxesAccMediumRead(int axes);

/*
	Calibration function: read the axes when the device is
	assumed flat and the acceleretion's values are the
	offset using to compesating the acceleration's measures.
*/
void accelerometerCalibration(void);

void printMagneticAxis(void);
void printHeading(void);
float headingDetect(void);
void compassCompensate(float compassData[3]);
void readMagneticAxis(float rawData[3]);
void compassInit(void);
void write8(byte address, registerEnum reg, byte value);
byte read8(byte address, registerEnum reg);

int getObjectContinuos(int directions);
bool getErrorObject(int directions);
int dirToMove(int servoStep);
float readUltrasonic(int nSensor);
float readUltrasonicAvg(int nSensor);

void controlTest(void);
void motorBaseTest(void);
void moveControl(motorDirection command, int pwm);
void motorMove(motorDirection comand, int pwm, int angle);
void motorTurn(int angle, int pwm);
void motorForward(int pwm);
void motorBackward(int pwm);
void motorStop(void);
void motorRight(int pwm, int angle);
void motorTurnRight(int pwm);
void motorLeft(int pwm, int angle);
void motorTurnLeft(int pwm);
void motorInit(void);

float getTemperature(void);


#endif // ROBOT_H_INCLUDED

