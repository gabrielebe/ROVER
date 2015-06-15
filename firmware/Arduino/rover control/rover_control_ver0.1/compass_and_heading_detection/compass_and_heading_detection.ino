#include <Wire.h>
#include "main.h"

void setup()
{
  Serial.begin(BAUD_RATE);
  compassInit();
}


void loop()
{
  printMagneticAxis();
  printHeading();
  delay(COMPASS_PERIODE_MEASURE * 10);
}


void compassInit(void)
{
  /* Configure the magnetometer */
  write8(COMPASS_ADDRESS, COMPASS_REG_A, 0x50);
  write8(COMPASS_ADDRESS, COMPASS_REG_B, 0x00);
  write8(COMPASS_ADDRESS, COMPASS_MODE_REG, 0x00);

  delay(COMPASS_PERIODE_MEASURE);
}


void printHeading(void)
{
  float heading = headingDetect();
  Serial.println(heading);
  Serial.println();
}


void printMagneticAxis(void)
{
  float x, y, z;
  readMagneticAxis(&x, &y, &z);

  Serial.print("X:  ");
  Serial.print(x);
  Serial.print("   Y:  ");
  Serial.print(y);
  Serial.print("   Z:  ");
  Serial.print(z);
  Serial.println();
  Serial.println();
}


void readMagneticAxis(float* x, float* y, float* z)
{
  /* Read the magnetometer */
  Wire.beginTransmission((byte)COMPASS_ADDRESS);
  Wire.write(COMPASS_DATA_OUT_X_MSB);
  Wire.endTransmission();
  Wire.requestFrom((byte)COMPASS_ADDRESS, (byte)6);

  /* Wait around until enough data is available */
  while (Wire.available() < 6);

  uint8_t xhi = Wire.read();
  uint8_t xlo = Wire.read();
  uint8_t zhi = Wire.read();
  uint8_t zlo = Wire.read();
  uint8_t yhi = Wire.read();
  uint8_t ylo = Wire.read();

  /* Shift values to create properly formed integer (low byte first) */
  int16_t xTemp = (int16_t)(xlo | ((int16_t)xhi << 8));
  int16_t yTemp = (int16_t)(ylo | ((int16_t)yhi << 8));
  int16_t zTemp = (int16_t)(zlo | ((int16_t)zhi << 8));

  *x = (float)xTemp * COMPASS_GAUSS_LSB;
  *y = (float)yTemp * COMPASS_GAUSS_LSB;
  *z = (float)zTemp * COMPASS_GAUSS_LSB;
}


float headingDetect(void)
{
  float compassX, compassY, compassZ;
  float headingDegrees;
  float heading = 0;

  readMagneticAxis(&compassX, &compassY, &compassZ);

  if (compassX && compassY)
    heading = atan2(compassY, compassX);

  /* Once you have your heading, you must then add your 'Declination Angle',
   which is the 'Error' of the magnetic field in your location.
   Find yours here: http://www.magnetic-declination.com/
   Mine is: +2° 10' , which is ~2 Degrees, or (which we need) 0.035 radians */
  heading = heading + COMPASS_DECLINATION_ANGLE;

  /* Correct for when signs are reversed */
  if (heading < 0)
    heading = heading + 2 * M_PI;

  /* Convert radians to degrees for readability */
  headingDegrees = degrees(heading);

  return headingDegrees;
}


void write8(byte address, registerEnum reg, byte value)
{
  Wire.beginTransmission(address);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
  Wire.endTransmission();
}


byte read8(byte address, registerEnum reg)
{
  byte value;
  Wire.beginTransmission(address);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)1);
  value = Wire.read();
  Wire.endTransmission();
  return value;
}

