/******************************************************
* The Author: Singh Harpal - Bè Gabriele - Rosa Alice *
* Date: xx/05/2015                                    *
* Version: V1.0                                       *
* Project name: Johnny Robot Control                  *
*******************************************************/

#include "robot.h"
#include <Wire.h>
#include <Servo.h>

Servo towerServo;

void setup()
{
#ifdef SERIAL_DEBUG
  Serial.begin(BAUD_RATE);
#endif

  Serial1.begin(38400);
  Wire.begin();
  compassInit();
  accelerometerCalibration();
  temperature = getTemperature();
  towerServo.attach(SERVO_PIN);
  motorInit();
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ARM_SUPPLAY_ENABLE, OUTPUT);
  digitalWrite(ARM_SUPPLAY_ENABLE, LOW);
  pinMode(AUTO_LED, OUTPUT);
}


void loop()
{
  /*if(autoMoveMode)
  {
    if((millis() - autoLastTime) > 500)
    {
      if(autoToggle)
      {
        digitalWrite(AUTO_LED, HIGH);
        tone(BUZZER_PIN, 440, 300);
        autoToggle = false;
      }
      else
      {
        digitalWrite(AUTO_LED, LOW);
        autoToggle = true;
      }
      autoLastTime = millis();
    }
  }*/

  serialRead();

  if (autoMoveMode)
  {
    autoMove();
  }

  //delayMicroseconds(1000);
}



/* Automovement functiones controll */

void autoMove(void)
{
  int forDist = 0;
  int sxDist = 0;
  int dxDist = 0;

  towerServo.write(90);
  motorMove(FOREWARD, AUTO_VELOCITY, NONE);

  if (checkNearObject(FORE_SENSOR))
  {
    delay(200);
    motorMove(FAST_STOP, NONE, NONE);
    forDist = readUltrasonic(FORE_SENSOR);
    towerServo.write(10);
    delay(200);
    dxDist = readUltrasonic(FORE_SENSOR);
    towerServo.write(170);
    delay(200);
    sxDist = readUltrasonic(FORE_SENSOR);

    if ((sxDist > COLLISION_LIMIT) || (dxDist > COLLISION_LIMIT))
    {
      if (sxDist >= dxDist)
        motorMove(LEFT_ANGLE, TURN_VELOCITY, 80);
      else
        motorMove(RIGHT_ANGLE, TURN_VELOCITY, 80);
    }
    else
    {
      if (readUltrasonic(BACK_SENSOR > 100))
      {
        motorMove(BACKWARD, AUTO_VELOCITY, NONE);
        delay(1500);
        motorMove(FAST_STOP, NONE, NONE);
        towerServo.write(10);
        delay(200);
        dxDist = readUltrasonic(FORE_SENSOR);
        towerServo.write(170);
        delay(200);
        sxDist = readUltrasonic(FORE_SENSOR);

        if (sxDist > dxDist)
          motorMove(LEFT_ANGLE, TURN_VELOCITY, 80);
        else
          motorMove(RIGHT_ANGLE, TURN_VELOCITY, 80);
      }
    }

    /*Serial.print("Forward distance: ");
    Serial.print(forDist);
    Serial.print("  Right distance: ");
    Serial.print(dxDist);
    Serial.print("  Left distance: ");
    Serial.println(sxDist);*/
  }
}


void autoMoveControl(void)
{
  int forDist = 0;
  int sxDist = 0;
  int dxDist = 0;

  if (getObjectError())
  {
    delay(200);
    motorMove(FAST_STOP, NONE, NONE);
    forDist = readUltrasonic(FORE_SENSOR);
    towerServo.write(10);
    delay(200);
    dxDist = readUltrasonic(FORE_SENSOR);
    towerServo.write(170);
    delay(200);
    sxDist = readUltrasonic(FORE_SENSOR);

    if ((sxDist > COLLISION_LIMIT) || (dxDist > COLLISION_LIMIT))
    {
      if (sxDist >= dxDist)
        motorMove(LEFT_ANGLE, TURN_VELOCITY, 80);
      else
        motorMove(RIGHT_ANGLE, TURN_VELOCITY, 80);
    }
    else
    {
      if (readUltrasonic(BACK_SENSOR > 70))
      {
        motorMove(BACKWARD, AUTO_VELOCITY, NONE);
        delay(1500);
        motorMove(FAST_STOP, NONE, NONE);
        towerServo.write(10);
        delay(200);
        dxDist = readUltrasonic(FORE_SENSOR);
        towerServo.write(170);
        delay(200);
        sxDist = readUltrasonic(FORE_SENSOR);

        if (sxDist > dxDist)
          motorMove(LEFT_ANGLE, TURN_VELOCITY, 80);
        else
          motorMove(RIGHT_ANGLE, TURN_VELOCITY, 80);
      }
    }

    motorMove(FOREWARD, AUTO_VELOCITY, NONE);
  }
}


boolean getObjectError(void)
{
  towerServo.write(10);
  delay(200);

  if (checkNearObject(FORE_SENSOR))
    return true;

  towerServo.write(90);
  delay(200);

  if (checkNearObject(FORE_SENSOR))
    return true;

  towerServo.write(170);
  delay(200);

  if (checkNearObject(FORE_SENSOR))
    return true;

  return false;
}



/* Magnetometer and compass functiones */

float headingDetect(void)
{
  float compassData[3];
  float heading;

  readMagneticAxis(compassData);
  compassCompensate(compassData);

  if (compassData[1] && compassData[0])
    heading = atan2(compassData[1], compassData[0]);

  if (heading < 0)
    heading = heading + 2 * PI;

  if (heading > 2 * PI)
    heading = heading - 2 * PI;

  return (heading * RAD_TO_DEG);
}


void compassCompensate(float compassData[3])
{
  double calibrationMatrix[3][3] =
  {
    {1.136, -0.007, 0.018},
    { -0.003, 1.023, -0.002},
    {0.045, -0.074, 1.398}
  };

  double bias[3] =
  {
    -38.215,
    -43.315,
    -10.107
  };

  // Application of calibration and compensation formula
  for (int i = 0; i < 3; ++i)
    compassData[i] = compassData[i] - bias[i];

  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      compassData[i] += calibrationMatrix[i][j] * compassData[j];
}


void readMagneticAxis(float rawData[3])
{
  Wire.beginTransmission((byte)COMPASS_ADDRESS);
  Wire.write(COMPASS_DATA_OUT_BEGIN);
  Wire.requestFrom((byte)COMPASS_ADDRESS, (byte)6);

  // Wait around until enough data is available
  while (Wire.available() < 6);

  // Read the magnetometer
  uint8_t xhi = Wire.read();
  uint8_t xlo = Wire.read();
  uint8_t zhi = Wire.read();
  uint8_t zlo = Wire.read();
  uint8_t yhi = Wire.read();
  uint8_t ylo = Wire.read();
  Wire.endTransmission();

  rawData[0] = (float)((int16_t)(xlo | ((int16_t)xhi << 8))) * COMPASS_GAUSS_LSB;
  rawData[1] = (float)((int16_t)(ylo | ((int16_t)yhi << 8))) * COMPASS_GAUSS_LSB;
  rawData[2] = (float)((int16_t)(zlo | ((int16_t)zhi << 8))) * COMPASS_GAUSS_LSB;
}


void compassInit(void)
{
  write8(COMPASS_ADDRESS, COMPASS_REG_A, 0x50);
  write8(COMPASS_ADDRESS, COMPASS_REG_B, 0x00);
  write8(COMPASS_ADDRESS, COMPASS_MODE_REG, 0x00);
  delay(COMPASS_PERIODE_MEASURE);
}


void write8(byte address, int reg, byte value)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}


byte read8(int address, int reg)
{
  byte value;
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(address, 1);
  value = Wire.read();
  Wire.endTransmission();
  return value;
}



/* Accelerometer functiones */

void accValuePrint(void)
{
  float xAssG = 0.0;
  float yAssG = 0.0;
  float zAssG = 0.0;
  float roll = 0.0;
  float pitch = 0.0;

  gAccelerationMeasuramentRead(&xAssG, &yAssG, &zAssG);
  tiltSensorRead(&roll, &pitch);

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


void tiltSensorRead(float* roll, float* pitch)
{
  float xAssG, yAssG, zAssG;
  gAccelerationMeasuramentRead(&xAssG, &yAssG, &zAssG);
  *roll  = (atan2(yAssG, zAssG) * 180.0) / M_PI;
  *pitch = (atan2(xAssG, sqrt(-yAssG * yAssG + zAssG * zAssG)) * 180.0) / M_PI;
}


void gAccelerationMeasuramentRead(float* xAssG, float* yAssG, float* zAssG)
{
  int xSample = analogRead(X_ASS);
  int ySample = analogRead(Y_ASS);
  int zSample = analogRead(Z_ASS);
  float xVoltage = (((float)xSample) * 5.0) / 1023;
  float yVoltage = (((float)ySample) * 5.0) / 1023;
  float zVoltage = (((float)zSample) * 5.0) / 1023;
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
  float xVoltage = (((float)xSample) * 5.0) / 1023;
  float yVoltage = (((float)ySample) * 5.0) / 1023;
  float zVoltage = ((((float)zSample) * 5.0) / 1023) - ACC_SENTITIVITY;

  x0gCurrent = (xVoltage - ACC_ZERO_G_REF_VOLTAGE) / ACC_SENTITIVITY;
  y0gCurrent = (yVoltage - ACC_ZERO_G_REF_VOLTAGE) / ACC_SENTITIVITY;
  z1gCurrent = (zVoltage - ACC_ZERO_G_REF_VOLTAGE) / ACC_SENTITIVITY;
}



/* Ultrasonic sensor and servo control functiones */

boolean checkNearObject(int directions)
{
  int distance = readUltrasonic(directions);
  Serial.print("Collision distance: ");
  Serial.println(distance);

  if (distance <= COLLISION_LIMIT)
    return true;
  else
    return false;
}


float readUltrasonic(int nSensor)
{
  unsigned long timeUltrasonicResponse;
  float netDistance;
  float microsecondsPerCm = 1 / ((331.5 + (0.6 * temperature)) / 10000);
  float sensorOffset = 0.2 * microsecondsPerCm * 2;

  switch (nSensor)
  {
    case BACK_SENSOR:
      pinMode(PING_PIN, OUTPUT);
      digitalWrite(PING_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(PING_PIN, HIGH);
      delayMicroseconds(5);
      digitalWrite(PING_PIN, LOW);
      pinMode(PING_PIN, INPUT);
      timeUltrasonicResponse = pulseIn(PING_PIN, HIGH);
      break;

    case FORE_SENSOR:
      digitalWrite(TRIG_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(TRIG_PIN, HIGH);
      delayMicroseconds(5);
      digitalWrite(TRIG_PIN, LOW);
      timeUltrasonicResponse = pulseIn(ECHO_PIN, HIGH);
      break;
  }

  netDistance = max(0, timeUltrasonicResponse - sensorOffset);
  return (netDistance / microsecondsPerCm / 2);
}



/* Control robot and motor functiones */

void motorMove(int command, int pwm, int angle)
{
  if (pwm > PWM_MAX)
    pwm = PWM_MAX;

  switch (command)
  {
    case FOREWARD:
      motorForward(pwm);
      break;

    case BACKWARD:
      motorBackward(pwm);
      break;

    case NORMAL_STOP:
      motorNormalStop();
      break;

    case FAST_STOP:
      motorFastStop();
      break;

    case RIGHT_ANGLE:
      motorRight(pwm, angle);
      break;

    case RIGHT:
      motorTurnRight(pwm);
      break;

    case LEFT_ANGLE:
      motorLeft(pwm, angle);
      break;

    case LEFT:
      motorTurnLeft(pwm);
      break;
  }
}


void motorForward(int pwm)
{
  analogWrite(MOTOR_PIN_EN1, 0);
  analogWrite(MOTOR_PIN_EN2, 0);
  analogWrite(MOTOR_PIN_EN3, 0);
  analogWrite(MOTOR_PIN_EN4, 0);

  digitalWrite(MOTOR_PIN_AM1, LOW);
  digitalWrite(MOTOR_PIN_BM1, HIGH);
  digitalWrite(MOTOR_PIN_AM2, LOW);
  digitalWrite(MOTOR_PIN_BM2, HIGH);
  digitalWrite(MOTOR_PIN_AM3, LOW);
  digitalWrite(MOTOR_PIN_BM3, HIGH);
  digitalWrite(MOTOR_PIN_AM4, LOW);
  digitalWrite(MOTOR_PIN_BM4, HIGH);

  analogWrite(MOTOR_PIN_EN1, pwm);
  analogWrite(MOTOR_PIN_EN2, pwm);
  analogWrite(MOTOR_PIN_EN3, pwm);
  analogWrite(MOTOR_PIN_EN4, pwm);
}


void motorBackward(int pwm)
{
  analogWrite(MOTOR_PIN_EN1, 0);
  analogWrite(MOTOR_PIN_EN2, 0);
  analogWrite(MOTOR_PIN_EN3, 0);
  analogWrite(MOTOR_PIN_EN4, 0);

  digitalWrite(MOTOR_PIN_AM1, HIGH);
  digitalWrite(MOTOR_PIN_BM1, LOW);
  digitalWrite(MOTOR_PIN_AM2, HIGH);
  digitalWrite(MOTOR_PIN_BM2, LOW);
  digitalWrite(MOTOR_PIN_AM3, HIGH);
  digitalWrite(MOTOR_PIN_BM3, LOW);
  digitalWrite(MOTOR_PIN_AM4, HIGH);
  digitalWrite(MOTOR_PIN_BM4, LOW);

  analogWrite(MOTOR_PIN_EN1, pwm);
  analogWrite(MOTOR_PIN_EN2, pwm);
  analogWrite(MOTOR_PIN_EN3, pwm);
  analogWrite(MOTOR_PIN_EN4, pwm);
}


void motorNormalStop(void)
{
  analogWrite(MOTOR_PIN_EN1, 0);
  analogWrite(MOTOR_PIN_EN2, 0);
  analogWrite(MOTOR_PIN_EN3, 0);
  analogWrite(MOTOR_PIN_EN4, 0);

  digitalWrite(MOTOR_PIN_AM1, LOW);
  digitalWrite(MOTOR_PIN_BM1, LOW);
  digitalWrite(MOTOR_PIN_AM2, LOW);
  digitalWrite(MOTOR_PIN_BM2, LOW);
  digitalWrite(MOTOR_PIN_AM3, LOW);
  digitalWrite(MOTOR_PIN_BM3, LOW);
  digitalWrite(MOTOR_PIN_AM4, LOW);
  digitalWrite(MOTOR_PIN_BM4, LOW);
}


void motorFastStop(void)
{
  analogWrite(MOTOR_PIN_EN1, 0);
  analogWrite(MOTOR_PIN_EN2, 0);
  analogWrite(MOTOR_PIN_EN3, 0);
  analogWrite(MOTOR_PIN_EN4, 0);

  digitalWrite(MOTOR_PIN_AM1, HIGH);
  digitalWrite(MOTOR_PIN_BM1, HIGH);
  digitalWrite(MOTOR_PIN_AM2, HIGH);
  digitalWrite(MOTOR_PIN_BM2, HIGH);
  digitalWrite(MOTOR_PIN_AM3, HIGH);
  digitalWrite(MOTOR_PIN_BM3, HIGH);
  digitalWrite(MOTOR_PIN_AM4, HIGH);
  digitalWrite(MOTOR_PIN_BM4, HIGH);

  analogWrite(MOTOR_PIN_EN1, 255);
  analogWrite(MOTOR_PIN_EN2, 255);
  analogWrite(MOTOR_PIN_EN3, 255);
  analogWrite(MOTOR_PIN_EN4, 255);
}


void motorRight(int pwm, int angle)
{
  float heading = headingDetect();
  float previewHeading = heading;
  float headingDiff = 0;

  motorMove(RIGHT, pwm, NONE);

  while (!(headingDiff >= angle))
  {
    heading = headingDetect();
    headingDiff = abs(heading - previewHeading);
    delay(10);
  }

  motorMove(stopMode, NONE, NONE);
}


void motorTurnRight(int pwm)
{
  analogWrite(MOTOR_PIN_EN1, 0);
  analogWrite(MOTOR_PIN_EN2, 0);
  analogWrite(MOTOR_PIN_EN3, 0);
  analogWrite(MOTOR_PIN_EN4, 0);

  digitalWrite(MOTOR_PIN_AM1, HIGH);
  digitalWrite(MOTOR_PIN_BM1, LOW);
  digitalWrite(MOTOR_PIN_AM2, HIGH);
  digitalWrite(MOTOR_PIN_BM2, LOW);
  digitalWrite(MOTOR_PIN_AM3, LOW);
  digitalWrite(MOTOR_PIN_BM3, HIGH);
  digitalWrite(MOTOR_PIN_AM4, LOW);
  digitalWrite(MOTOR_PIN_BM4, HIGH);

  analogWrite(MOTOR_PIN_EN1, pwm);
  analogWrite(MOTOR_PIN_EN2, pwm);
  analogWrite(MOTOR_PIN_EN3, pwm);
  analogWrite(MOTOR_PIN_EN4, pwm);
}


void motorLeft(int pwm, int angle)
{
  float heading = headingDetect();
  float previewHeading = heading;
  float headingDiff = 0;

  motorMove(LEFT, pwm, NONE);

  while (!(headingDiff >= angle))
  {
    heading = headingDetect();
    headingDiff = abs(heading - previewHeading);
    delay(10);
  }

  motorMove(stopMode, NONE, NONE);
}


void motorTurnLeft(int pwm)
{
  analogWrite(MOTOR_PIN_EN1, 0);
  analogWrite(MOTOR_PIN_EN2, 0);
  analogWrite(MOTOR_PIN_EN3, 0);
  analogWrite(MOTOR_PIN_EN4, 0);

  digitalWrite(MOTOR_PIN_AM1, LOW);
  digitalWrite(MOTOR_PIN_BM1, HIGH);
  digitalWrite(MOTOR_PIN_AM2, LOW);
  digitalWrite(MOTOR_PIN_BM2, HIGH);
  digitalWrite(MOTOR_PIN_AM3, HIGH);
  digitalWrite(MOTOR_PIN_BM3, LOW);
  digitalWrite(MOTOR_PIN_AM4, HIGH);
  digitalWrite(MOTOR_PIN_BM4, LOW);

  analogWrite(MOTOR_PIN_EN1, pwm);
  analogWrite(MOTOR_PIN_EN2, pwm);
  analogWrite(MOTOR_PIN_EN3, pwm);
  analogWrite(MOTOR_PIN_EN4, pwm);
}


void motorInit(void)
{
  pinMode(MOTOR_PIN_AM1, OUTPUT);
  pinMode(MOTOR_PIN_BM1, OUTPUT);
  pinMode(MOTOR_PIN_AM2, OUTPUT);
  pinMode(MOTOR_PIN_BM2, OUTPUT);
  pinMode(MOTOR_PIN_AM3, OUTPUT);
  pinMode(MOTOR_PIN_BM3, OUTPUT);
  pinMode(MOTOR_PIN_AM4, OUTPUT);
  pinMode(MOTOR_PIN_BM4, OUTPUT);
  pinMode(MOTOR_PIN_EN1, OUTPUT);
  pinMode(MOTOR_PIN_EN2, OUTPUT);
  pinMode(MOTOR_PIN_EN3, OUTPUT);
  pinMode(MOTOR_PIN_EN4, OUTPUT);

  digitalWrite(MOTOR_PIN_AM1, LOW);
  digitalWrite(MOTOR_PIN_BM1, LOW);
  digitalWrite(MOTOR_PIN_AM2, LOW);
  digitalWrite(MOTOR_PIN_BM2, LOW);
  digitalWrite(MOTOR_PIN_AM3, LOW);
  digitalWrite(MOTOR_PIN_BM3, LOW);
  digitalWrite(MOTOR_PIN_AM4, LOW);
  digitalWrite(MOTOR_PIN_BM4, LOW);
  analogWrite(MOTOR_PIN_EN1, 0);
  analogWrite(MOTOR_PIN_EN2, 0);
  analogWrite(MOTOR_PIN_EN3, 0);
  analogWrite(MOTOR_PIN_EN4, 0);
}



/* RF functiones */

void serialRead(void)
{
  int index = 0;
  unsigned char dataArray[10] = {0};
  int xValue, yValue, zValue;

  while (Serial1.available() > 0)
  {
    dataArray[index] = Serial1.read();
    index++;
    delayMicroseconds(500);

    if (index > 10)
      break;
  }

  if (index == 1)
  {
    Serial.print("OneCommand: ");
    Serial.write(dataArray[0]);
    Serial.println();

    if (dataArray[0] == 'm')
    {
      autoMoveMode = false;
      digitalWrite(AUTO_LED, LOW);
      noTone(BUZZER_PIN);
      motorMove(FAST_STOP, NONE, NONE);
    }

    if (dataArray[0] == 'a')
    {
      autoMoveMode = true;
      towerServo.write(90);
      motorMove(FOREWARD, AUTO_VELOCITY, NONE);
      delay(100);
      previewHeading = headingDetect();
    }

    index = 0;
  }

  if (index == 2)
  {
    switch (dataArray[0])
    {
      case '0':
        motorMove(stopMode, NONE, NONE);
        break;

      case '1':
        motorMove(FOREWARD, dataArray[1], NONE);
        break;

      case '2':
        motorMove(BACKWARD, dataArray[1], NONE);
        break;

      case '3':
        motorMove(RIGHT, dataArray[1], NONE);
        break;

      case '4':
        motorMove(LEFT, dataArray[1], NONE);
        break;
    }

    index = 0;
  }

  if (index == 3)
  {
    Serial.print("ThreeCommand: ");
    Serial.write(dataArray[0]);
    Serial.write(dataArray[1]);
    Serial.write(dataArray[2]);
    Serial.println();
    index = 0;
  }

  if (index == 10)
  {
    Serial.print("TenCommand: ");
    Serial.write(dataArray[0]);
    Serial.write(dataArray[1]);
    Serial.write(dataArray[2]);
    Serial.write(dataArray[3]);
    Serial.write(dataArray[4]);
    Serial.write(dataArray[5]);
    Serial.write(dataArray[6]);
    Serial.write(dataArray[7]);
    Serial.write(dataArray[8]);
    Serial.write(dataArray[9]);
    Serial.println();
    index = 0;
  }
}


void send6charRF(char val0, char val1, char val2, char val3, char val4, char val5)
{
  Serial.println("Printing back");
  char finalArray[7] = {val0, val1, val2, val3, val4, val5, '\n'};
  Serial.println(finalArray);
  Serial1.write(finalArray, 7);
  delayRF(7);
}



/* General pourpose functiones */

float getTemperature(void)
{
  int sensorVoltage = analogRead(TEMP_PIN);
  float voltage = (sensorVoltage * 5.0) / 1023;
  return (((voltage * 1000) - 500.0) / 10.0);
}

