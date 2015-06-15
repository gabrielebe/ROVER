#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED


#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


#define BAUD_RATE         115200
#define SERIAL_DEBUG
#define RAD_TO_DEG        (57.295779513082320876798154814105F)
#define GRAVITY_EARTH     (9.80665F)
#define BATTERY_LIMIT     (15.0F)
#define BATT_MAX_SUPPLY   (18.2F)

#define SX               0
#define DX               1
#define FORE_SENSOR      0
#define BACK_SENSOR      1

#define COMPASS_ADDRESS           0x1E
#define COMPASS_REG_A             0x00
#define COMPASS_REG_B             0x01
#define COMPASS_MODE_REG          0x02
#define COMPASS_DATA_OUT_BEGIN    0x03
#define COMPASS_GAUSS_LSB         (0.73F)
#define COMPASS_PERIODE_MEASURE   56
#define ACC_ZERO_G_REF_VOLTAGE    (1.65F)
#define ACC_SENTITIVITY           (0.300F)

#define X_ASS                 A0
#define Y_ASS       	      A1
#define Z_ASS      	      A2
#define TEMP_PIN              A3
#define BATTERY_PIN           A4
#define SERVO_PIN   	      44
#define PING_PIN    	      46
#define ECHO_PIN    	      34
#define TRIG_PIN    	      36
#define ARM_SUPPLAY_ENABLE    12
#define AUTO_LED	      45
#define BUZZER_PIN	      47

#define MOTOR_PIN_AM1 22
#define MOTOR_PIN_BM1 23
#define MOTOR_PIN_AM2 24
#define MOTOR_PIN_BM2 25
#define MOTOR_PIN_AM3 26
#define MOTOR_PIN_BM3 27
#define MOTOR_PIN_AM4 28
#define MOTOR_PIN_BM4 29
#define MOTOR_PIN_EN1 2
#define MOTOR_PIN_EN2 3
#define MOTOR_PIN_EN3 4
#define MOTOR_PIN_EN4 5

#define HEADING_LIMIT     10
#define COLLISION_LIMIT   50
#define PWM_MAX           170
#define AUTO_VELOCITY     100
#define TURN_VELOCITY     150

#define delayRF(numByte) (4 + numByte)


enum
{
  NONE,
  FOREWARD,
  BACKWARD,
  NORMAL_STOP,
  FAST_STOP,
  RIGHT_ANGLE,
  RIGHT,
  LEFT_ANGLE,
  LEFT,
} MOTOR_COMMAND;


unsigned long tempLastTime = 0;
unsigned long battLastTime = 0;
unsigned long distLastTime = 0;
unsigned long magnLastTime = 0;
unsigned long acceLastTime = 0;
unsigned long autoLastTime = 0;

boolean autoMoveMode = false;
boolean armOn = false;
boolean autoToggle = true;
int stopMode = NORMAL_STOP;
int correctionDir = SX;
float temperature = 20.0;
float x0gCurrent = 0.0;
float y0gCurrent = 0.0;
float z1gCurrent = 0.0;
float errorHeading = 0.0;
float previewHeading = 0.0;
float error = 0.0;
float previewError = 0.0;


void autoMove(void);

float headingDetect(void);
void compassCompensate(float compassData[3]);
void readMagneticAxis(float rawData[3]);
void compassInit(void);
void write8(byte address, int reg, byte value);
byte read8(byte address, int reg);

void accValuePrint(void);
void tiltSensorRead(float* roll, float* pitch);
void gAccelerationMeasuramentRead(float* xAssG, float* yAssG, float* zAssG);
void accelerometerCalibration(void);

boolean checkNearObject(int directions);
float readUltrasonic(int nSensor);

void motorMove(int comand, int pwm, int angle);
void motorForward(int pwm);
void motorBackward(int pwm);
void motorNormalStop(void);
void motorFastStop(void);
void motorRight(int pwm, int angle);
void motorTurnRight(int pwm);
void motorLeft(int pwm, int angle);
void motorTurnLeft(int pwm);
void motorInit(void);

void serialRead(void);
void send6charRF(char val0, char val1, char val2, char val3, char val4, char val5);

float getTemperature(void);

#endif

