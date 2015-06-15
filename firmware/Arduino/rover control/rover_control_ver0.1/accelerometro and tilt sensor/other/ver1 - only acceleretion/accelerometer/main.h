#ifndef MAIN_H
#define MAIN_H

#define BAUD_RATE 115200

#define X_ASS A0
#define Y_ASS A1
#define Z_ASS A2

#define GRAVITY_EARTH          (9.80665F)       /* Earth's gravity in m/s^2 */
#define ZERO_G_REF_VOLTAGE     (1.65F)
#define SENSOR_SENTITIVITY     (0.300F)
#define ITERATION_POINT        (3.0F)

float xAssG, yAssG, zAssG;
float xAss, yAss, zAss;

void gAccelerationMeasuramentPrint(void);
void accelerationMeasuramentPrint(void);
void gAccelerationMeasuramentRead(void);
float singlAassMeasure(int ass);

#endif

