#ifndef JHONNY_H_INCLUDED
#define JHONNY_H_INCLUDED

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define DEBUG
#define BAUD_RATE 9600

/* General purpose constants */
#define GRAVITY_EARTH         (9.80665F)       /* Earth's gravity in m/s^2 */
#define MAGFIELD_EARTH_MAX_G  (0.6F)           /* Maximum magnetic field on Earth's surface in gauss*/
#define MAGFIELD_EARTH_MIN_G  (0.3F)           /* Minimum magnetic field on Earth's surface in gauss*/
#define MAGFIELD_EARTH_MAX_T  (60.0F)          /* Maximum magnetic field on Earth's surface in micro-Tesla*/
#define MAGFIELD_EARTH_MIN_T  (30.0F)          /* Minimum magnetic field on Earth's surface in micro-Tesla*/
#define GAUSS_TO_MICROTESLA   100              /* Gauss to micro-Tesla multiplier */

/* General trigonometrical defines */
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)

/* Analog constant */
#define ANALOG_ADDRESS      0x0A
#define ANALOG_PERIODE_MEASURE      10000

/* Compass constants */
#define COMPASS_ADDRESS                (0x1E)          /* 7 bit address of the HMC58X3 used with the Wire library */
#define COMPASS_GAUSS_LSB              (0.73F)         /* Varies with gain */
#define COMPASS_DECLINATION_ANGLE      0.22            /* Error of the magnetic field in your location */
#define COMPASS_SELF_TEST_LL           243	       /* For gain of 5, like the self test configuration */
#define COMPASS_SELF_TEST_HL           575	       /* For gain of 5, like the self test configuration */
#define COMPASS_PERIODE_MEASURE        56

/*#define NONE 0
#define TURN 1
#define FOREWARD 2
#define BACKWARD 3
#define STOP 4
#define RIGHT 5
#define LEFT 6*/

/*#define MOTOR_PIN_AM1 1
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
#define MOTOR_PIN_EN4 12*/

/* Robot, ultrasonic e servo costants*/
#define PING_SERVO_PIN 9
#define PING_SENSOR_PIN 7
#define HEADING_LIMIT 5
#define DISTANCE_LIMIT 30
#define SENSOR_GAP 0.2
#define SX 0
#define DX 1


/* Register address define */
/*typedef enum compassRegisterEnum
{
    ANALOG_REG_A       =  0x00,
    ANALOG_MODE_REG    =  0x01,
    ANALOG_DATA_TEMP   =  0x02,
    ANALOG_DATA_LUX    =  0x03,
} ANALOG_REGISTER;*/

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

  ANALOG_REG_A       =  0x00,
  ANALOG_MODE_REG    =  0x01,
  ANALOG_DATA_TEMP   =  0x02,
  ANALOG_DATA_LUX    =  0x03,

} REGISTER;

/* Motor direction command define */
typedef enum motorDirection
{
  NONE,
  TURN,
  FOREWARD,
  BACKWARD,
  STOP,
  RIGHT,
  LEFT,
} MOTOR_COMMAND;

/* Motor pins defines */
typedef enum motorPins
{
  MOTOR_PIN_AM1  =  1,
  MOTOR_PIN_BM1  =  2,
  MOTOR_PIN_AM2  =  3,
  MOTOR_PIN_BM2  =  4,
  MOTOR_PIN_AM3  =  5,
  MOTOR_PIN_BM3  =  6,
  MOTOR_PIN_AM4  =  7,
  MOTOR_PIN_BM4  =  8,
  MOTOR_PIN_EN1  =  9,
  MOTOR_PIN_EN2  =  10,
  MOTOR_PIN_EN3  =  11,
  MOTOR_PIN_EN4  =  12,
} MOTOR_PIN;


/* Accelerometer axis data */
typedef struct
{
  float x;
  float y;
  float z;
} ANALOG_accelerometerAxis;

ANALOG_accelerometerAxis ANALOG_accelerometerAxisAcceleration;
ANALOG_accelerometerAxis ANALOG_accelerometerAxisTilt;

/* Internal magnetometer data type */
typedef struct
{
  float x;
  float y;
  float z;
} COMPASS_axisData;

COMPASS_axisData COMPASS_axisMagneticData;


void ARDUINO_init(void);
void ARDUINO_comunicationInit(void);
void write8(byte address, registerEnum reg, byte value);
byte read8(byte address, registerEnum reg);

void ANALOG_temperatureRead(void);
float ANALOG_luxRead(void);
void ANALOG_accelerometerRead(void);
void ANALOG_tiltSensor(void);

bool COMPASS_init(void);
bool COMPASS_selfTest(void);
void COMPASS_readMagneticAxis(void);
float COMPASS_headingDetect(void);
void COMPASS_getID(char id[3]);

void MOTOR_init(void);
void MOTOR_move(motorDirection comand, int pwm, int angle);
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
long ANALOG_previewTime = millis();
float temperatura;

long COMPASS_previewTime = millis();
float previewHeading, previewError;
float heading = 0.0;
float error = 0.0;
int correctionDir = SX;

#endif // JHONNY_H_INCLUDED

