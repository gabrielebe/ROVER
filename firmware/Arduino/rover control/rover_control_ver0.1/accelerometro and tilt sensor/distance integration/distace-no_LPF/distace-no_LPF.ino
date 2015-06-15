#define BAUD_RATE 115200

#define X_ASS A0
#define Y_ASS A1
#define Z_ASS A2

#define ZERO_G_REF_VOLTAGE     (1.65F)
#define SENSOR_SENTITIVITY     (0.300F)
#define ALPHA                  (0.3F)

float x0gCurrent, y0gCurrent;
int xSample, ySample;
float xVoltage, yVoltage;
float gxAcceleration, gyAcceleration;
float xAcceleration, yAcceleration;
float xVelocity, yVelocity;
float xDistance, yDistance;
float xPreviewVelocity = 0.0;
float yPreviewVelocity = 0.0;
float xPreviewDistance = 0.0;
float yPreviewDistance = 0.0;
int tDelay = 500;
float dt = (float)tDelay / 1000000;

void cinematicSerialPrint(void);
void cinematicRead(void);
void accelerometerCalibration(void);


void setup()
{
  Serial.begin(BAUD_RATE);
  accelerometerCalibration();
}


void loop()
{
  cinematicRead();
  cinematicSerialPrint();
  delayMicroseconds(tDelay);
}


void cinematicSerialPrint(void)
{
  Serial.print("Accelerazione m/s^2 asse X:  ");
  Serial.print(xAcceleration);
  Serial.print("     Velocity m/s asse X:  ");
  Serial.print(xVelocity);
  Serial.print("     Distance m asse X:  ");
  Serial.println(xDistance);
  Serial.print("Accelerazione m/s^2 asse Y:  ");
  Serial.print(yAcceleration);
  Serial.print("     Velocity m/s asse Y:  ");
  Serial.print(yVelocity);
  Serial.print("     Distance m asse Y:  ");
  Serial.println(yDistance);
  Serial.println();
}


void cinematicRead(void)
{
  xSample = analogRead(X_ASS);
  xVoltage = (((float)xSample) * 5.0) / 1023;
  gxAcceleration = ((xVoltage - ZERO_G_REF_VOLTAGE) / SENSOR_SENTITIVITY) - x0gCurrent;
  xAcceleration = abs(gxAcceleration) * 9.81;
  xVelocity = xPreviewVelocity + xAcceleration * dt;
  xDistance = xPreviewDistance + xVelocity * dt;
  xPreviewVelocity = xVelocity;
  xPreviewDistance = xDistance;

  ySample = analogRead(Y_ASS);
  yVoltage = (((float)ySample) * 5.0) / 1023;
  gyAcceleration = ((yVoltage - ZERO_G_REF_VOLTAGE) / SENSOR_SENTITIVITY) - y0gCurrent;
  yAcceleration = abs(gyAcceleration) * 9.81;
  yVelocity = yPreviewVelocity + yAcceleration * dt;
  yDistance = yPreviewVelocity + yVelocity * dt;
  yPreviewVelocity = yVelocity;
  yPreviewDistance = yDistance;
}


void accelerometerCalibration(void)
{
  int sample;
  float voltage;

  sample = analogRead(X_ASS);
  voltage = (((float)sample) * 5.0) / 1023;
  x0gCurrent = (voltage - ZERO_G_REF_VOLTAGE) / SENSOR_SENTITIVITY;

  sample = analogRead(Y_ASS);
  voltage = (((float)sample) * 5.0) / 1023;
  y0gCurrent = (voltage - ZERO_G_REF_VOLTAGE) / SENSOR_SENTITIVITY;
}

