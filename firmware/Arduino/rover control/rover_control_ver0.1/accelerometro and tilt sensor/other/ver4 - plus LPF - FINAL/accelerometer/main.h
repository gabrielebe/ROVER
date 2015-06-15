#ifndef MAIN_H
#define MAIN_H

#define BAUD_RATE 115200
#define DEBUG

#define X_ASS A0
#define Y_ASS A1
#define Z_ASS A2

#define ZERO_G_REF_VOLTAGE     (1.65F)
#define SENSOR_SENTITIVITY     (0.300F)
#define ITERATION_POINT        (3.0F)

/* Smoothing factor of low pass filter */
#define ALPHA (0.3F)

float x0gCurrent, y0gCurrent, z1gCurrent;

void getTilt(float angles[2]);
void accelerometerCalibration(void);

#endif

