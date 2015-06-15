#ifndef MAIN_H
#define MAIN_H

#define BAUD_RATE 115200

#define X_ASS A0
#define Y_ASS A1
#define Z_ASS A2

#define ZERO_G_REF_VOLTAGE     (1.65F)
#define SENSOR_SENTITIVITY     (0.300F)
#define ITERATION_POINT        (3.0F)

void tiltSensorPrint(void);
void gAccelerationMeasuramentPrint(void);
void tiltSensorRead(float* roll, float* pitch);
void gAccelerationMeasuramentRead(float* xAssG, float* yAssG, float* zAssG);
float singlAassMeasure(int axes);

#endif

