#include "main.h"

void setup()
{
  Serial.begin(BAUD_RATE);

  /* Calibration function: read the axes when the device is
     assumed flat and the acceleretion's values are the offset
     using to compesating the acceleration's measures. */
  accelerometerCalibration();
}


void loop()
{
  gAccelerationMeasuramentPrint();
  tiltSensorPrint();
  delay(100);
}


void tiltSensorPrint(void)
{
  float roll, pitch;
  tiltSensorRead(&roll, &pitch);

  Serial.print("Pitch:  ");
  Serial.print(pitch);
  Serial.print("   Roll:  ");
  Serial.println(roll);
  Serial.println();
}


void gAccelerationMeasuramentPrint(void)
{
  float xAssG, yAssG, zAssG;
  gAccelerationMeasuramentRead(&xAssG, &yAssG, &zAssG);

  Serial.print("X: ");
  Serial.print(xAssG);
  Serial.print("  Y: ");
  Serial.print(yAssG);
  Serial.print("  Z: ");
  Serial.print(zAssG);
  Serial.println();
}


void tiltSensorRead(float* roll, float* pitch)
{
  float xAssG, yAssG, zAssG;
  gAccelerationMeasuramentRead(&xAssG, &yAssG, &zAssG);

  /* Roll & Pitch Equations */
  *roll  = (atan2(yAssG, zAssG) * 180.0) / M_PI;
  *pitch = (atan2(xAssG, sqrt(-yAssG * yAssG + zAssG * zAssG)) * 180.0) / M_PI;
}


/* Read the value from the sensor */
void gAccelerationMeasuramentRead(float* xAssG, float* yAssG, float* zAssG)
{
  int sample = analogRead(X_ASS);
  float voltage = (((float)sample) * 5.0) / 1023;
  float acceleration = (voltage - ZERO_G_REF_VOLTAGE) / SENSOR_SENTITIVITY;
  *xAssG = acceleration - x0gCurrent;

  sample = analogRead(Y_ASS);
  voltage = (((float)sample) * 5.0) / 1023;
  acceleration = (voltage - ZERO_G_REF_VOLTAGE) / SENSOR_SENTITIVITY;
  *yAssG = acceleration - y0gCurrent;

  sample = analogRead(Z_ASS);
  voltage = (((float)sample) * 5.0) / 1023;
  acceleration = (voltage - ZERO_G_REF_VOLTAGE) / SENSOR_SENTITIVITY;
  *zAssG = acceleration - z1gCurrent;
}


void accelerometerCalibration(void)
{
  int sample = analogRead(X_ASS);
  float voltage = (((float)sample) * 5.0) / 1023;
  x0gCurrent = (voltage - ZERO_G_REF_VOLTAGE) / SENSOR_SENTITIVITY;

  sample = analogRead(Y_ASS);
  voltage = (((float)sample) * 5.0) / 1023;
  y0gCurrent = (voltage - ZERO_G_REF_VOLTAGE) / SENSOR_SENTITIVITY;

  sample = analogRead(Z_ASS);
  voltage = ((((float)sample) * 5.0) / 1023) - SENSOR_SENTITIVITY;
  z1gCurrent = (voltage - ZERO_G_REF_VOLTAGE) / SENSOR_SENTITIVITY;
}

