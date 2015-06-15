#include "main.h"

void setup()
{
  Serial.begin(BAUD_RATE);
}


void loop()
{
  gAccelerationMeasuramentPrint();
  accelerationMeasuramentPrint();
  Serial.println();
  Serial.println();
  delay(100);
}


void gAccelerationMeasuramentPrint(void)
{
  gAccelerationMeasuramentRead();
  Serial.print("X: ");
  Serial.print(xAssG);
  Serial.print("  Y: ");
  Serial.print(yAssG);
  Serial.print("  Z: ");
  Serial.print(zAssG);
  Serial.println();
}


void accelerationMeasuramentPrint(void)
{
  gAccelerationMeasuramentRead();
  Serial.print("X: ");
  Serial.print(xAss);
  Serial.print("  Y: ");
  Serial.print(yAss);
  Serial.print("  Z: ");
  Serial.print(zAss);
  Serial.println();
}


void gAccelerationMeasuramentRead(void)
{
  xAssG = singlAassMeasure(X_ASS);
  yAssG = singlAassMeasure(Y_ASS);
  zAssG = singlAassMeasure(Z_ASS);
  xAss = xAssG * GRAVITY_EARTH;
  yAss = yAssG * GRAVITY_EARTH;
  zAss = zAssG * GRAVITY_EARTH;
}


float singlAassMeasure(int ass)
{
  int assPoint;
  float assRead;
  float assSum = 0.0;

  for (int i = 0; i < ITERATION_POINT; i++)
  {
    assPoint = analogRead(ass);
    assRead = (((float)assPoint) * 5.0) / 1023;
    assSum = assSum + assRead;
  }

  float mediumValue = assSum / ITERATION_POINT;
  float voltageToG = (mediumValue - ZERO_G_REF_VOLTAGE) / SENSOR_SENTITIVITY;
  return voltageToG;
}

