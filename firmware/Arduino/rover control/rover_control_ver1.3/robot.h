#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED


// Include Arduino library and defines
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define BAUD_RATE         115200
#define SERIAL_DEBUG
#define RAD_TO_DEG        (57.295779513082320876798154814105F)
#define GRAVITY_EARTH     (9.80665F)
#define BATTERY_LIMIT     (7.8F)
#define BATT_MAX_SUPPLY   (15.0F)

// Compass constants
#define COMPASS_ADDRESS           0x1E      // 7 bit address of the HMC58X3 used with the Wire library
#define COMPASS_REG_A             0x00
#define COMPASS_REG_B             0x01
#define COMPASS_MODE_REG          0x02
#define COMPASS_DATA_OUT_BEGIN    0x03
#define COMPASS_GAUSS_LSB         (0.73F)   // Varies with gain
#define COMPASS_PERIODE_MEASURE   56


// Pin constants define
#define X_ASS       				 A0
#define Y_ASS       				 A1
#define Z_ASS      			 		 A2
#define TEMP_PIN    				 A3
#define BATTERY_PIN          A4
#define SERVO_PIN   				 22
#define PING_PIN    				 23
#define ECHO_PIN    				 24
#define TRIG_PIN    				 25
#define ARM_SUPPLAY_ENABLE   34
#define AUTO_LED						 35
#define BUZZER_PIN					 36

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
#define HEADING_LIMIT           3
#define ULTRASONIC_ITERATIONS   (3.0F)
#define SX                      0
#define DX                      1
#define FORE_SENSOR             0
#define BACK_SENSOR             1


//*************************************************//
/**                Delay table                     **
/**  k = 4.21 @9600bps                             **
/**  k = 3.35 @19200bps                            **
/**  k = 3.1  @38400bps and @57600bps              **
/**  k = 2.8  @115200bps                           **
 //*************************************************/
#define delayRF(numByte) (4 + numByte)
// The maximum number of byte is 240


// Motor direction command define
enum
{
  NONE,
  TURN,
  FOREWARD,
  BACKWARD,
  NORMAL_STOP,
  FAST_STOP,
  RIGHT_ANGLE,
  RIGHT,
  LEFT_ANGLE,
  LEFT,
} MOTOR_COMMAND;


// Global variables
unsigned long tempLastTime = 0;
unsigned long battLastTime = 0;
unsigned long distLastTime = 0;
unsigned long magnLastTime = 0;
unsigned long autoLastTime = 0;
boolean armOn = false;
boolean autoMoveMode = false;
boolean autoToggle = true;
int stopMode = FAST_STOP;
int correctionDir = SX;
int angleToMoveDistMin = 15;
int pwmAutoMode = 128;
int pwmMax = 255;
float temperature;
float errorPreviewHeading, errorHeading;
float previewError = 0;
float error = 0;


void autoMoveControl(void);
boolean getObjectError(int directions);
int dirToMove(int servoStep);
void moveControl(void);
boolean checkNearObject(int directions);
float readUltrasonic(int nSensor);
float getTemperature(void);

float headingDetect(void);
void compassCompensate(float compassData[3]);
void readMagneticAxis(float rawData[3]);
void compassInit(void);
void write8(byte address, int reg, byte value);
byte read8(byte address, int reg);

void motorMove(int comand, int pwm, int angle);
void motorTurn(int angle, int pwm);
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


#endif // ROBOT_H_INCLUDED

