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
  float tiltAngles[2];
  getTilt(tiltAngles);
  float roll = tiltAngles[0];
  float pitch = tiltAngles[1];

#ifdef DEBUG
  Serial.print("Roll:  ");
  Serial.print(roll);
  Serial.print("   Pitch:  ");
  Serial.println(pitch);
  Serial.println();
#endif

  delay(100);
}


/* angles[0] = roll
   angles[1] = pitch */
void getTilt(float angles[2])
{
  int sample;
  float voltage;
  float Xg, Yg, Zg;
  float fXg, fYg, fZg;

  /* Read and convert value from analog accelerometer */
  sample = analogRead(X_ASS);
  voltage = (((float)sample) * 5.0) / 1023;
  Xg = ((voltage - ZERO_G_REF_VOLTAGE) / SENSOR_SENTITIVITY) - x0gCurrent;

  /* Low Pass Filter: the value rappresente a filtered acceleration component */
  fXg = Xg * ALPHA + (fXg * (1.0 - ALPHA));

  sample = analogRead(Y_ASS);
  voltage = (((float)sample) * 5.0) / 1023;
  Yg = ((voltage - ZERO_G_REF_VOLTAGE) / SENSOR_SENTITIVITY) - y0gCurrent;
  fYg = Yg * ALPHA + (fYg * (1.0 - ALPHA));

  sample = analogRead(Z_ASS);
  voltage = (((float)sample) * 5.0) / 1023;
  Zg = ((voltage - ZERO_G_REF_VOLTAGE) / SENSOR_SENTITIVITY) - z1gCurrent;
  fZg = Zg * ALPHA + (fZg * (1.0 - ALPHA));

  /* Estimate roll and pitch angles and return */
  angles[0]  = (atan2(fYg, fZg) * 180.0) / M_PI;
  angles[1] = (atan2(fXg, sqrt(-fYg * fYg + fZg * fZg)) * 180.0) / M_PI;
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

