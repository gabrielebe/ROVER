#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

#include <Arduino.h>

#define BAUD_RATE 9600

/* General trigonometrical defines */
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)

/* Compass constants */
#define COMPASS_ADDRESS                (0x1E)          /* 7 bit address of the HMC58X3 used with the Wire library */
#define COMPASS_GAUSS_LSB              (0.73F)         /* Varies with gain */
#define COMPASS_DECLINATION_ANGLE      0.22            /* Error of the magnetic field in your location */
#define COMPASS_PERIODE_MEASURE        56

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

void compassInit(void);
void printHeading(void);
void printMagneticAxis(void);;
void readMagneticAxis(float* x, float* y, float* z);
float headingDetect(void);
void write8(byte address, registerEnum reg, byte value);
byte read8(byte address, registerEnum reg);

#endif // MAIN_H_INCLUDED

