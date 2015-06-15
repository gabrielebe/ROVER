/* robot.ino */

/******************************************************
* The Author: Singh Harpal - Bè Gabriele - Rosa Alice *
* Date: xx/04/2015                                    *
* Version: V1.0                                       *
* Project name: Johnny Robot Control                  *
*******************************************************/

#include "robot.h"
#include <Wire.h>
#include <Servo.h>

#ifdef DEBUG_LCD
#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
#endif

Servo towerServo;

void setup()
{
#ifdef SERIAL_DEBUG
  Serial.begin(BAUD_RATE);
#endif

  Serial1.begin(38400);

  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  motorInit();
  towerServo.attach(SERVO_PIN);
  accelerometerCalibration();
  compassInit();
  temperature = getTemperature();

#ifdef SERIAL_DEBUG
  Serial.print("Temperature: ");
  Serial.println(temperature);
#endif

  // Infinite loop
  void remoteTest(void);
}


void loop()
{
  if ((millis() - tempLastTime) > (10 * 1000))
  {
    temperature = getTemperature();
    tempLastTime = millis();

#ifdef SERIAL_DEBUG
    Serial.println();
    Serial.print("Temperature:  ");
    Serial.println(temperature);
#endif
  }

  /*if((millis() - distLastTime) > 500)
  {
      float distFore = readUltrasonic(FORE_SENSOR);
      float distBack = readUltrasonic(BACK_SENSOR);
      distLastTime = millis();

      #ifdef SERIAL_DEBUG
      Serial.print("Distance foreward: ");
      Serial.print(distFore);
      Serial.print("  Distance backward: ");
      Serial.println(distBack);
      #endif
  }*/

  /*if((millis() - magnLastTime) > 100)
  {
      float heading = headingDetect();
      magnLastTime = millis();

      #ifdef DEBUG_SERIAL
      Serial.print("Heading: ");
      Serial.println(heading);
      #endif
  }*/

  delayMicroseconds(1000);
}



/* Accelerometer functiones */

/*void cinematicSerialPrint(void)
{
    float cinematicData[6];
    cinematicRead(cinematicData);

    #ifdef SERIAL_DEBUG
    Serial.println();
    Serial.print("Distance [m] asse X: ");
    Serial.print(cinematicData[0]);
    Serial.print("  Velocity [m/s] asse X: ");
    Serial.print(cinematicData[1]);
    Serial.print("  Accelerazione [m/s^2] asse X: ");
    Serial.println(cinematicData[2]);
    Serial.print("Distance m asse Y: ");
    Serial.print(cinematicData[3]);
    Serial.print("  Velocity m/s asse Y: ");
    Serial.print(cinematicData[4]);
    Serial.print("  Accelerazione [m/s^2] asse Y: ");
    Serial.println(cinematicData[5]);
    #endif
}*/


/*
    cinematicData[0] = xDistance;
    cinematicData[1] = xVelocity;
    cinematicData[2] = xAcceleration;
    cinematicData[3] = yDistance;
    cinematicData[4] = yVelocity;
    cinematicData[5] = yAcceleration;
*/
/*void cinematicRead(float cinematicData[6])
{
    float acc[3];
    gAccelerationMeasuramentRead(acc);
    float xAcceleration = acc[0] * GRAVITY_EARTH;
    float yAcceleration = acc[1] * GRAVITY_EARTH;

    float xVelocity = xPreviewVelocity + xAcceleration * dt;
    float xDistance = xPreviewDistance + xVelocity * dt;
    xPreviewVelocity = xVelocity;
    xPreviewDistance = xDistance;
    float yVelocity = yPreviewVelocity + yAcceleration * dt;
    float yDistance = yPreviewVelocity + yVelocity * dt;
    yPreviewVelocity = yVelocity;
    yPreviewDistance = yDistance;

    cinematicData[0] = xDistance;
    cinematicData[1] = xVelocity;
    cinematicData[2] = xAcceleration;
    cinematicData[3] = yDistance;
    cinematicData[4] = yVelocity;
    cinematicData[5] = yAcceleration;
    delay(tDelay);
}*/


void printTilt(void)
{
  float tiltAngles[2];
  getTilt(tiltAngles);

#ifdef SERIAL_DEBUG
  Serial.println();
  Serial.print("Roll:  ");
  Serial.print(tiltAngles[0]);
  Serial.print("   Pitch:  ");
  Serial.println(tiltAngles[1]);
  Serial.println();
#endif
}


void accelerationMeasuramentPrint(void)
{
  float acc[3], gAcc[3];
  gAccelerationMeasuramentRead(gAcc);

  for (int i = 0; i < 3; i++)
    acc[i] = GRAVITY_EARTH * gAcc[i];

#ifdef SERIAL_DEBUG
  Serial.println();
  Serial.print("Acceleration [m/s^2]:\tX: ");
  Serial.print(acc[0]);
  Serial.print("  Y: ");
  Serial.print(acc[1]);
  Serial.print("  Z: ");
  Serial.println(acc[2]);
  Serial.print("Acceleration [g]:\tX: ");
  Serial.print(gAcc[0]);
  Serial.print("  Y: ");
  Serial.print(gAcc[1]);
  Serial.print("  Z: ");
  Serial.print(gAcc[2]);
  Serial.println();
#endif
}


/*
	angles[0] = roll
    angles[1] = pitch
*/
void getTilt(float angles[2])
{
  int sample;
  float voltage;
  float Xg, Yg, Zg;
  float fXg, fYg, fZg;

  // Read and convert value from analog accelerometer
  sample = analogRead(X_ASS);
  voltage = (((float)sample) * 5.0) / 1023;
  Xg = ((voltage - ZERO_G_REF_VOLTAGE) / SENSOR_SENTITIVITY) - x0gCurrent;

  // Low Pass Filter: the value rappresente a filtered acceleration component
  fXg = Xg * ALPHA + (fXg * (1.0 - ALPHA));

  sample = analogRead(Y_ASS);
  voltage = (((float)sample) * 5.0) / 1023;
  Yg = ((voltage - ZERO_G_REF_VOLTAGE) / SENSOR_SENTITIVITY) - y0gCurrent;
  fYg = Yg * ALPHA + (fYg * (1.0 - ALPHA));

  sample = analogRead(Z_ASS);
  voltage = (((float)sample) * 5.0) / 1023;
  Zg = ((voltage - ZERO_G_REF_VOLTAGE) / SENSOR_SENTITIVITY) - z1gCurrent;
  fZg = Zg * ALPHA + (fZg * (1.0 - ALPHA));

  // Estimate roll and pitch angles and return
  angles[0]  = (atan2(fYg, fZg) * 180.0) / M_PI;
  angles[1] = (atan2(fXg, sqrt(-fYg * fYg + fZg * fZg)) * 180.0) / M_PI;
}


/*
	acc[0] = xAcc
    acc[1] = yAcc
    acc[2] = zAcc
*/
void gAccelerationMeasuramentRead(float acc[3])
{
  int sample = analogRead(X_ASS);
  float voltage = (((float)sample) * 5.0) / 1023;
  float acceleration = (voltage - ZERO_G_REF_VOLTAGE) / SENSOR_SENTITIVITY;
  acc[0] = acceleration - x0gCurrent;

  sample = analogRead(Y_ASS);
  voltage = (((float)sample) * 5.0) / 1023;
  acceleration = (voltage - ZERO_G_REF_VOLTAGE) / SENSOR_SENTITIVITY;
  acc[1] = acceleration - y0gCurrent;

  sample = analogRead(Z_ASS);
  voltage = (((float)sample) * 5.0) / 1023;
  acceleration = (voltage - ZERO_G_REF_VOLTAGE) / SENSOR_SENTITIVITY;
  acc[2] = acceleration - z1gCurrent;
}


void accelerationMediumRead(float gAcc[3], float acc[3])
{
  gAcc[0] = singleAxesAccMediumRead(X_ASS);
  gAcc[1] = singleAxesAccMediumRead(Y_ASS);
  gAcc[2] = singleAxesAccMediumRead(Z_ASS);

  for (int i = 0; i < 3; i++)
    acc[i] = GRAVITY_EARTH * gAcc[i];
}


float singleAxesAccMediumRead(int axes)
{
  int axPoint;
  float axRead;
  float axSum = 0.0;

  for (int i = 0; i < ITERATION_POINT; i++)
  {
    axPoint = analogRead(axes);
    axRead = (((float)axPoint) * 5.0) / 1023;
    axSum = axSum + axRead;
  }

  float mediumValue = axSum / ITERATION_POINT;
  float voltageToG = (mediumValue - ZERO_G_REF_VOLTAGE) / SENSOR_SENTITIVITY;
  return voltageToG;
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



/* Magnetometer and compass functiones */

void printMagneticAxis(void)
{
  float gaussData[3];
  readMagneticAxis(gaussData);

  Serial.println();
  Serial.println("Megnetometer data [gauss]:");
  Serial.print("X: ");
  Serial.print(gaussData[0]);
  Serial.print("  Y: ");
  Serial.print(gaussData[1]);
  Serial.print("  Z: ");
  Serial.print(gaussData[2]);
  Serial.println();
}


void printHeading(void)
{
  float heading = headingDetect();

#ifdef SERIAL_DEBUG
  Serial.println();
  Serial.print("Heading:  ");
  Serial.println(heading);
#endif
}


float headingDetect(void)
{
  float compassData[3];
  float heading;

  readMagneticAxis(compassData);
  compassCompensate(compassData);

#ifdef SERIAL_DEBUG
  Serial.println();
  Serial.print("X magnetometer: ");
  Serial.print(compassData[0]);
  Serial.print(",  ");
  Serial.print("  Y magnetometer: ");
  Serial.print(compassData[1]);
  Serial.print(",  ");
  Serial.print("  Z magnetometer: ");
  Serial.println(compassData[2]);
#endif

  if (compassData[1] && compassData[0])
    heading = atan2(compassData[1], compassData[0]);

  // Correct for when signs are reversed
  if (heading < 0)
    heading = heading + 2 * PI;

  // Check for wrap due to addition of declination
  if (heading > 2 * PI)
    heading = heading - 2 * PI;

  return heading * RAD_TO_DEG;
}


void compassCompensate(float compassData[3])
{
  // calibrationMatrix[3][3] is the transformation matrix
  double calibrationMatrix[3][3] =
  {
    {1.019, 0.001, 0.003},
    {0.032, 0.97, -0.048},
    { -0.02, -0.017, 1.274}
  };

  // bias[3] is the bias vector
  double bias[3] =
  {
    -34.311,
    -212.987,
    -122.717
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
  // Read the magnetometer
  Wire.beginTransmission((byte)COMPASS_ADDRESS);
  Wire.write(COMPASS_DATA_OUT_X_MSB);
  Wire.endTransmission();
  Wire.requestFrom((byte)COMPASS_ADDRESS, (byte)6);

  // Wait around until enough data is available
  while (Wire.available() < 6);

  uint8_t xhi = Wire.read();
  uint8_t xlo = Wire.read();
  uint8_t zhi = Wire.read();
  uint8_t zlo = Wire.read();
  uint8_t yhi = Wire.read();
  uint8_t ylo = Wire.read();

  // Shift values to create properly formed integer (low byte first)
  rawData[0] = (float)((int16_t)(xlo | ((int16_t)xhi << 8))) * COMPASS_GAUSS_LSB;
  rawData[1] = (float)((int16_t)(ylo | ((int16_t)yhi << 8))) * COMPASS_GAUSS_LSB;
  rawData[2] = (float)((int16_t)(zlo | ((int16_t)zhi << 8))) * COMPASS_GAUSS_LSB;
}


void compassInit(void)
{
  // Configure the magnetometer
  write8(COMPASS_ADDRESS, COMPASS_REG_A, 0x50);
  write8(COMPASS_ADDRESS, COMPASS_REG_B, 0x00);
  write8(COMPASS_ADDRESS, COMPASS_MODE_REG, 0x00);

  /*
      Normally we would delay the application by 66ms to allow the loop
      to run at 15Hz (default bandwidth for the HMC5883L).
  */
  delay(COMPASS_PERIODE_MEASURE);
}


void write8(byte address, registerEnum reg, byte value)
{
  Wire.beginTransmission(address);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
  Wire.endTransmission();
}


byte read8(int address, registerEnum reg)
{
  byte value;
  Wire.beginTransmission(address);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom(address, 1);
  value = Wire.read();
  Wire.endTransmission();
  return value;
}



/* Ultrasonic sensor and servo control functiones */

int getObjectContinuos(int directions)
{
  int angleToMove;
  int servoAngle = 60;
  bool errorObject;

  towerServo.write(servoAngle);
  delay(300);
  errorObject = getErrorObject(directions);

  while (!errorObject)
  {
    servoAngle = 90;

    for (int i = 0; i < 2; i++)
    {
      towerServo.write(servoAngle);
      delay(500);
      errorObject = getErrorObject(directions);

      if (errorObject)
        break;

      servoAngle += 30;
    }

    servoAngle = 90;

    for (int i = 0; i < 2; i++)
    {
      towerServo.write(servoAngle);
      delay(500);
      errorObject = getErrorObject(directions);

      if (errorObject)
        break;

      servoAngle -= 30;
    }
  }

  angleToMove = dirToMove(45);
  return angleToMove;
}


bool getErrorObject(int directions)
{
  float distance = readUltrasonic(directions);

  if (distance > 30)
    return false;
  else
    return true;
}


int dirToMove(int servoStep)
{
  int previewDist = 0;
  int angleToMove = 0;
  int previewAngle, distanceToMove;
  int servoAngle = 22;
  float foreDistance, backDistance;
  float mDistance[2][4];

  for (int i = 0; i < 4; i++)
  {
    towerServo.write(servoAngle);
    delay(500);
    foreDistance = readUltrasonic(FORE_SENSOR);
    backDistance = readUltrasonic(BACK_SENSOR);
    mDistance[0][i] = foreDistance;
    mDistance[1][i] = backDistance;

#ifdef SERIAL_DEBUG
    Serial.println();
    Serial.print("  Distance foreward: ");
    Serial.print(foreDistance);
    Serial.print("  Distance backward: ");
    Serial.println(backDistance);
#endif

    previewAngle = angleToMove;
    previewDist = distanceToMove;

    if (foreDistance > backDistance)
    {
      angleToMove = servoAngle;
      distanceToMove = foreDistance;
    }
    else
    {
      angleToMove = servoAngle + 180;
      distanceToMove = backDistance;
    }

    if (previewDist > distanceToMove)
    {
      angleToMove = previewAngle;
      distanceToMove = previewDist;
    }

    servoAngle += servoStep;
  }

#ifdef SERIAL_DEBUG
  Serial.println();
  Serial.println(angleToMove);
#endif

  return angleToMove;
}


float readUltrasonic(int nSensor)
{
  unsigned long timeUltrasonicResponse;
  float netDistance;
  float microsecondsPerCm = 1 / ((331.5 + (0.6 * temperature)) / 10000);
  float sensorOffset = 0.2 * microsecondsPerCm * 2;

  switch (nSensor)
  {
    case FORE_SENSOR:
      pinMode(PING_PIN, OUTPUT);
      digitalWrite(PING_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(PING_PIN, HIGH);
      delayMicroseconds(5);
      digitalWrite(PING_PIN, LOW);
      pinMode(PING_PIN, INPUT);
      timeUltrasonicResponse = pulseIn(PING_PIN, HIGH);
      break;

    case BACK_SENSOR:
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


float readUltrasonicAvg(int nSensor)
{
  float distanceAvg;
  float sum = 0.0;

  for (int i = 0; i < ULTRASONIC_ITERATIONS; i++)
  {
    float distance = readUltrasonic(nSensor);
    sum += distance;
  }

  return (sum / ULTRASONIC_ITERATIONS);
}



/* Control robot and motor functiones */

void controlTest(void)
{
  motorBaseTest();

  motorMove(TURN, 128, 60);
  motorMove(FOREWARD, 128, NONE);
  delay(2000);
  motorMove(BACKWARD, 128, NONE);
  delay(2000);
  motorMove(STOP, NONE, NONE);
  delay(2000);
  motorMove(RIGHT_ANGLE, 128, 60);
  delay(2000);
  motorMove(RIGHT, 128, NONE);
  delay(2000);
  motorMove(LEFT_ANGLE, 128, 60);
  delay(2000);
  motorMove(LEFT, 128, NONE);
  delay(2000);
  motorMove(STOP, NONE, NONE);
  delay(2000);

  moveControl(FOREWARD, 150);
  //moveControl(BACKWARD, 150);
}


void motorBaseTest(void)
{
  for (int i = 0; i < 255; i += 100)
  {
#ifdef SERIAL_DEBUG
    Serial.println();
    Serial.print("Velocità pwm avanti: ");
    Serial.println(i);
#endif

    motorForward(i);
    delay(2000);
    motorStop();

#ifdef SERIAL_DEBUG
    Serial.println("Stop!");
#endif

    delay(2000);
  }

  for (int i = 0; i < 255; i += 100)
  {
#ifdef SERIAL_DEBUG
    Serial.print("Pwm backward speed: ");
    Serial.println(i);
#endif

    motorBackward(i);
    delay(2000);
    motorStop();

#ifdef SERIAL_DEBUG
    Serial.println("Stop!");
#endif

    delay(2000);
  }

  for (int i = 0; i < 255; i += 100)
  {
#ifdef SERIAL_DEBUG
    Serial.print("Pwm right speed: ");
    Serial.println(i);
#endif

    motorTurnRight(i);
    delay(2000);
    motorStop();

#ifdef SERIAL_DEBUG
    Serial.println("Stop!");
#endif

    delay(2000);
  }

  for (int i = 0; i < 255; i += 100)
  {
#ifdef SERIAL_DEBUG
    Serial.print("Pwm left speed: ");
    Serial.println(i);
#endif

    motorTurnLeft(i);
    delay(2000);
    motorStop();

#ifdef SERIAL_DEBUG
    Serial.println("Stop!");
#endif

    delay(2000);
  }
}


void moveControl(motorDirection command, int pwm)
{
  float errorPreviewHeading = headingDetect();
  float errorHeading = errorPreviewHeading;
  float previewError = 0;
  float error = 0;

  motorMove(command, pwm, NONE);

  while (1) {

    if (error >= HEADING_LIMIT)
    {
      if (error > previewError)
      {
        if (correctionDir == SX)
          correctionDir = DX;
        else
          correctionDir = SX;
      }

      if (correctionDir == SX)
      {
        motorMove(LEFT, 100, NONE);
        delay(100);
        motorMove(command, pwm, NONE);
      }
      else
      {
        motorMove(RIGHT, 100, NONE);
        delay(100);
        motorMove(command, pwm, NONE);
      }
    }

    delay(50);
    errorHeading = headingDetect();
    previewError = error;
    error = abs(errorHeading - errorPreviewHeading);

#ifdef SERIAL_DEBUG
    Serial.println();
    Serial.print("Error: ");
    Serial.print(error);
    Serial.print("  Error heading: ");
    Serial.print(errorHeading);
    Serial.print("  Direction: ");
    Serial.println(correctionDir);
#endif

    delay(100);
  }
}


void motorMove(motorDirection command, int pwm, int angle)
{
  switch (command)
  {
    case TURN:
      motorTurn(pwm, angle);
      break;

    case FOREWARD:
      motorForward(pwm);
      break;

    case BACKWARD:
      motorBackward(pwm);
      break;

    case STOP:
      motorStop();
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


void motorTurn(int angle, int pwm)
{
}


// This function is used to move forward the robot at a certain velocity
void motorForward(int pwm)
{
#ifdef DEBUG_LCD
  /*
      The following part is used only to display on the LCD the speed and the
      movement that the robot is doing
  */
  lcd.clear();
  lcd.print(" Forward speed: ");
  lcd.setCursor(0, 1);
  lcd.print("    ");
  lcd.print(pwm);
#endif

  analogWrite(MOTOR_PIN_EN1, 0);
  analogWrite(MOTOR_PIN_EN2, 0);
  analogWrite(MOTOR_PIN_EN3, 0);
  analogWrite(MOTOR_PIN_EN4, 0);

  // Setting the inputs in order to go forward
  digitalWrite(MOTOR_PIN_AM1, LOW);
  digitalWrite(MOTOR_PIN_BM1, HIGH);
  digitalWrite(MOTOR_PIN_AM2, LOW);
  digitalWrite(MOTOR_PIN_BM2, HIGH);
  digitalWrite(MOTOR_PIN_AM3, LOW);
  digitalWrite(MOTOR_PIN_BM3, HIGH);
  digitalWrite(MOTOR_PIN_AM4, LOW);
  digitalWrite(MOTOR_PIN_BM4, HIGH);

  // Setting the pwm signal given as a parameter to this function
  analogWrite(MOTOR_PIN_EN1, pwm);
  analogWrite(MOTOR_PIN_EN2, pwm);
  analogWrite(MOTOR_PIN_EN3, pwm);
  analogWrite(MOTOR_PIN_EN4, pwm);
}


// This function is used to move backward the robot at a certain velocity
void motorBackward(int pwm)
{
#ifdef DEBUG_LCD
  /*
      The following part is used only to display on the LCD the speed and the
      movement that the robot is doing
  */
  lcd.clear();
  lcd.print(" Backward speed: ");
  lcd.setCursor(0, 1);
  lcd.print("    ");
  lcd.print(pwm);
#endif

  analogWrite(MOTOR_PIN_EN1, 0);
  analogWrite(MOTOR_PIN_EN2, 0);
  analogWrite(MOTOR_PIN_EN3, 0);
  analogWrite(MOTOR_PIN_EN4, 0);

  // Setting the inputs in order to go backward
  digitalWrite(MOTOR_PIN_AM1, HIGH);
  digitalWrite(MOTOR_PIN_BM1, LOW);
  digitalWrite(MOTOR_PIN_AM2, HIGH);
  digitalWrite(MOTOR_PIN_BM2, LOW);
  digitalWrite(MOTOR_PIN_AM3, HIGH);
  digitalWrite(MOTOR_PIN_BM3, LOW);
  digitalWrite(MOTOR_PIN_AM4, HIGH);
  digitalWrite(MOTOR_PIN_BM4, LOW);

  // Setting the pwm signal given as a parameter to this function
  analogWrite(MOTOR_PIN_EN1, pwm);
  analogWrite(MOTOR_PIN_EN2, pwm);
  analogWrite(MOTOR_PIN_EN3, pwm);
  analogWrite(MOTOR_PIN_EN4, pwm);
}


// This function is used to stop the motors fast
void motorStop(void)
{
  analogWrite(MOTOR_PIN_EN1, 0);
  analogWrite(MOTOR_PIN_EN2, 0);
  analogWrite(MOTOR_PIN_EN3, 0);
  analogWrite(MOTOR_PIN_EN4, 0);

  // Setting the inputs in order to stop
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

  Serial.println("motorRight");

  while (!(headingDiff >= (angle - HEADING_LIMIT)))
  {
    Serial.println("while");

    motorTurnRight(pwm);
    delay(100);
    motorStop();
    delay(50);
    heading = headingDetect();
    headingDiff = abs(heading - previewHeading);

#ifdef SERIAL_DEBUG
    Serial.println();
    Serial.print("Heading: ");
    Serial.print(heading);
    Serial.print("  Preview heading: ");
    Serial.print(previewHeading);
    Serial.print("  Heading diff: ");
    Serial.println(headingDiff);
#endif
  }
}


// This function is used to turn right the robot at a certain velocity
void motorTurnRight(int pwm)
{
#ifdef DEBUG_LCD
  /*
      The following part is used only to display on the LCD the speed and the
      movement that the robot is doing
  */
  lcd.clear();
  lcd.print("Turn right speed");
  lcd.setCursor(0, 1);
  lcd.print("    ");
  lcd.print(pwm);
#endif

  // Setting the pwm signal given as a parameter to this function
  analogWrite(MOTOR_PIN_EN1, 0);
  analogWrite(MOTOR_PIN_EN2, 0);
  analogWrite(MOTOR_PIN_EN3, 0);
  analogWrite(MOTOR_PIN_EN4, 0);

  // Setting the inputs in order to go forward
  digitalWrite(MOTOR_PIN_AM1, HIGH);
  digitalWrite(MOTOR_PIN_BM1, LOW);
  digitalWrite(MOTOR_PIN_AM2, LOW);
  digitalWrite(MOTOR_PIN_BM2, HIGH);
  digitalWrite(MOTOR_PIN_AM3, HIGH);
  digitalWrite(MOTOR_PIN_BM3, LOW);
  digitalWrite(MOTOR_PIN_AM4, LOW);
  digitalWrite(MOTOR_PIN_BM4, HIGH);

  // Setting the pwm signal given as a parameter to this function
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

  while (!(headingDiff >= (angle - HEADING_LIMIT)))
  {
    motorTurnLeft(pwm);
    delay(100);
    motorStop();
    delay(50);
    heading = headingDetect();
    headingDiff = abs(heading - previewHeading);

#ifdef DEBUG_SERIAL
    Serial.println();
    Serial.print("Heading: ");
    Serial.print(heading);
    Serial.print("  Preview heading: ");
    Serial.print(previewHeading);
    Serial.print("  Heading diff: ");
    Serial.println(headingDiff);
#endif
  }
}


// This function is used to turn left the robot at a certain velocity
void motorTurnLeft(int pwm)
{
#ifdef DEBUG_LCD
  /*
      The following part is used only to display on the LCD the speed and the
      movement that the robot is doing
  */
  lcd.clear();
  lcd.print("Turn left speed");
  lcd.setCursor(0, 1);
  lcd.print("    ");
  lcd.print(pwm);
#endif

  // Setting the pwm signal given as a parameter to this function
  analogWrite(MOTOR_PIN_EN1, 0);
  analogWrite(MOTOR_PIN_EN2, 0);
  analogWrite(MOTOR_PIN_EN3, 0);
  analogWrite(MOTOR_PIN_EN4, 0);

  // Setting the inputs in order to go forward
  digitalWrite(MOTOR_PIN_AM1, LOW);
  digitalWrite(MOTOR_PIN_BM1, HIGH);
  digitalWrite(MOTOR_PIN_AM2, HIGH);
  digitalWrite(MOTOR_PIN_BM2, LOW);
  digitalWrite(MOTOR_PIN_AM3, LOW);
  digitalWrite(MOTOR_PIN_BM3, HIGH);
  digitalWrite(MOTOR_PIN_AM4, HIGH);
  digitalWrite(MOTOR_PIN_BM4, LOW);

  // Setting the pwm signal given as a parameter to this function
  analogWrite(MOTOR_PIN_EN1, pwm);
  analogWrite(MOTOR_PIN_EN2, pwm);
  analogWrite(MOTOR_PIN_EN3, pwm);
  analogWrite(MOTOR_PIN_EN4, pwm);
}


void motorInit(void)
{
  // Setting the pin directions to initialize the motor
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

  // Pulling down all the input for security reasons
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

void remoteTest(void)
{
  char index = 0;
  unsigned char dataArray[10] = {0};
  int xValue, yValue, zValue;

  while (1)
  {

    if (Serial1.available() > 0)
    {
      while (Serial1.available() > 0)
      {
        dataArray[index++] = Serial1.read();
      }

      if (index == 0 || index == 1)
      {
        Serial.println("NoCommand");
        index = 0;
      }

      else if (index == 2)
      {
        /*
            In case which receive only one character
            the data array 0 is the command char received
        */

#ifdef SERIAL_DEBUG
        Serial.println();
        Serial.println("One Command");
#endif

        if (dataArray[0] == 'A')
        {
          xValue = analogRead(X_ASS);
          yValue = analogRead(Y_ASS);
          zValue = analogRead(Z_ASS);

#ifdef SERIAL_DEBUG
          Serial.println();
          Serial.print("x value: ");
          Serial.print(xValue);
          Serial.print("  y value: ");
          Serial.print(yValue);
          Serial.print("  z value: ");
          Serial.println(zValue);
#endif

          send6charRF(xValue & 0xff, (xValue >> 8) & 0xff,
                      yValue & 0xff, (yValue >> 8) & 0xff,
                      zValue & 0xff, (zValue >> 8) & 0xff);
        }

        index = 0;
      }

      else if (index == 3)
      {

        /*
            Serial.println("Receiving 3 char");
            in case which receive only two characters
            the data array 0 is the first received character
            the data array 1 is the second received chracter

            The data array 0 represent the instruction
            The data array 1 represent the first parameter
            The data array 2 represent the second parameter
        */
        switch (dataArray[0])
        {
          case '0':
            motorStop();

#ifdef SERIAL_DEBUG
            Serial.println("Stopping the motor");
#endif

            break;

          case '1':
            motorForward(dataArray[1]);

#ifdef SERIAL_DEBUG
            Serial.print("Moving the motor forward by: ");
            Serial.write(dataArray[1]);
            Serial.println();
#endif

            break;

          case '2':
            motorBackward(dataArray[1]);

#ifdef SERIAL_DEBUG
            Serial.print("Moving the motor backward by: ");
            Serial.write(dataArray[1]);
            Serial.println();
#endif

            break;

          case '3':
            motorTurnRight(dataArray[1]);

#ifdef SERIAL_DEBUG
            Serial.print("turning the motor right with: ");
            Serial.write(dataArray[1]);
            //Serial.print("of degree");
            //Serial.write(secondParameter);
            Serial.println();
#endif

            break;

          case '4':
            motorTurnLeft(dataArray[1]);

#ifdef SERIAL_DEBUG
            Serial.print("turning the motor left with: ");
            Serial.write(dataArray[1]);
            //Serial.print("of degree");
            //Serial.write(secondParameter);
            Serial.println();
#endif

            break;

          default:
            motorStop();

#ifdef SERIAL_DEBUG
            Serial.println("Incorrect command");
#endif
        }

        delayRF(5);
        index = 0;
      }

      else if (index == 4)
      {
#ifdef SERIAL_DEBUG
        Serial.println("Receiving 5 char");
#endif

        index = 0;
      }

      else if (index == 6)
      {
#ifdef SERIAL_DEBUG
        Serial.println("Receiving 6 char");
#endif

        index = 0;
      }

      else
      {
#ifdef SERIAL_DEBUG
        Serial.println("Receiving more than 6 char");
#endif

        /*
            Add here new if else statements if needed when the
            received characters are more than those above declared
        */

        index = 0;
      }
    }
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

