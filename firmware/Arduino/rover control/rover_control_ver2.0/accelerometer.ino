#define X_ASS                  A0
#define Y_ASS                  A1
#define Z_ASS                  A2
#define ACC_ZERO_G_REF_VOLTAGE (1.65F)
#define ACC_SENTITIVITY        (0.300F)

float x0gCurrent = 0.0;
float y0gCurrent = 0.0;
float z1gCurrent = 0.0;


void accelerometerPrint(void)
{
  float xAssG = 0.0;
  float yAssG = 0.0;
  float zAssG = 0.0;
  float roll = 0.0;
  float pitch = 0.0;

  accelerationRead(&xAssG, &yAssG, &zAssG);
  tiltRead(&roll, &pitch);

  Serial.print("Acceleration:  x-> ");
  Serial.print(xAssG);
  Serial.print("  y-> ");
  Serial.print(yAssG);
  Serial.print("  z-> ");
  Serial.println(zAssG);
  Serial.print("Tilt angle: roll-> ");
  Serial.print(roll);
  Serial.println("  pitch-> ");
  Serial.println(roll);
  Serial.println();
}


void tiltRead(float* roll, float* pitch)
{
  float xAssG = 0.0;
  float yAssG = 0.0;
  float zAssG = 0.0;
  accelerationRead(&xAssG, &yAssG, &zAssG);
  *roll  = (atan2(yAssG, zAssG) * 180.0) / M_PI;
  *pitch = (atan2(xAssG, sqrt(-yAssG * yAssG + zAssG * zAssG)) * 180.0) / M_PI;
}


void accelerationRead(float* xAssG, float* yAssG, float* zAssG)
{
  int xSample = analogRead(X_ASS);
  int ySample = analogRead(Y_ASS);
  int zSample = analogRead(Z_ASS);
  float xVoltage = (((float)xSample) * 5.0) / 1024;
  float yVoltage = (((float)ySample) * 5.0) / 1024;
  float zVoltage = (((float)zSample) * 5.0) / 1024;
  float xAcceleration = (xVoltage - ACC_ZERO_G_REF_VOLTAGE) / ACC_SENTITIVITY;
  float yAcceleration = (yVoltage - ACC_ZERO_G_REF_VOLTAGE) / ACC_SENTITIVITY;
  float zAcceleration = (zVoltage - ACC_ZERO_G_REF_VOLTAGE) / ACC_SENTITIVITY;

  *xAssG = xAcceleration - x0gCurrent;
  *yAssG = yAcceleration - y0gCurrent;
  *zAssG = zAcceleration - z1gCurrent;
}


void accelerometerCalibration(void)
{
  int xSample = analogRead(X_ASS);
  int ySample = analogRead(Y_ASS);
  int zSample = analogRead(Z_ASS);
  float xVoltage = (((float)xSample) * 5.0) / 1024;
  float yVoltage = (((float)ySample) * 5.0) / 1024;
  float zVoltage = ((((float)zSample) * 5.0) / 1024) - ACC_SENTITIVITY;

  x0gCurrent = (xVoltage - ACC_ZERO_G_REF_VOLTAGE) / ACC_SENTITIVITY;
  y0gCurrent = (yVoltage - ACC_ZERO_G_REF_VOLTAGE) / ACC_SENTITIVITY;
  z1gCurrent = (zVoltage - ACC_ZERO_G_REF_VOLTAGE) / ACC_SENTITIVITY;
}

