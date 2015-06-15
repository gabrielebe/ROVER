/******************************************************
* The Author: Singh Harpal - Bè Gabriele - Rosa Alice *
* Date: xx/04/2015                                    *
* Version: V1.1                                       *
* Project name: Johnny Robot Control V1.0             *
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

  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  towerServo.attach(SERVO_PIN);
  motorInit();
  errorPreviewHeading = headingDetect();
  errorHeading = errorPreviewHeading;
  temperature = getTemperature();

#ifdef SERIAL_DEBUG
  Serial.print("Temperature: ");
  Serial.println(temperature);
  Serial.println();
#endif
}


void loop()
{
  if ((millis() - tempLastTime) > 7000)
  {
    temperature = getTemperature();
    tempLastTime = millis();

#ifdef SERIAL_DEBUG
    Serial.print("Temperature:  ");
    Serial.println(temperature);
#endif
  }

  if ((millis() - battLastTime) > 10000)
  {
    int battSample = analogRead(BATTERY_PIN);
    float battVoltage = (battSample * 5.0) / 1023;

    if (battVoltage > 15)
    {
      // (255 * 12) / vSupply
      float fpwmMax = float(3825) / battVoltage;
      pwmMax = int(fpwmMax);
    }

    if (battVoltage < BATTERY_LIMIT)
    {
      //battLow TO DO
    }
  }

  /*if((millis() - distLastTime) > 1000)
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

  /*if((millis() - magnLastTime) > 5000)
  {
    float heading = headingDetect();
    magnLastTime = millis();

    #ifdef SERIAL_DEBUG
    Serial.print("Heading: ");
    Serial.println(heading);
    #endif
  }*/

  if (autoMoveMode && ((millis() - autoLastTime) > 700))
  {
    if (autoToggle)
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
  }

  serialRead();

  if (autoMoveMode)
  {
#ifdef SERIAL_DEBUG
    Serial.println("Auto move function");
#endif

    autoMoveControl();
  }

  /*#ifdef SERIAL_DEBUG
  Serial.println();
  Serial.println();
  #endif*/

  delayMicroseconds(1000);
}



/* Automovement functiones controll */

void autoMoveControl(void)
{
  boolean error = getObjectError(FORE_SENSOR);

  if (error)
  {
    int angleToMove = dirToMove(45);

    while (angleToMove == -1)
    {
      motorMove(BACKWARD, pwmAutoMode, NONE);
      delay(500);
      motorMove(stopMode, NONE, NONE);
      angleToMove = getObjectError(FORE_SENSOR);

#ifdef SERIAL_DEBUG
      Serial.print("New test... angle to move: ");
      Serial.println(angleToMove);
#endif
    }

    motorMove(TURN, pwmAutoMode, angleToMove);
    motorMove(FOREWARD, pwmAutoMode, NONE);
  }

  else
  {
#ifdef SERIAL_DEBUG
    Serial.println("Move control function");
#endif

    moveControl();
  }
}


boolean getObjectError(int directions)
{
  int angleToMove;
  int servoAngle = 60;
  boolean errorObject;

  towerServo.write(servoAngle);
  delay(300);
  errorObject = checkNearObject(directions);
  servoAngle = 90;

  for (int i = 0; i < 2; i++)
  {
    towerServo.write(servoAngle);
    delay(300);
    errorObject = checkNearObject(directions);

    if (errorObject)
      return true;

    servoAngle += 30;
  }

  servoAngle = 90;

  for (int i = 0; i < 2; i++)
  {
    towerServo.write(servoAngle);
    delay(500);
    errorObject = checkNearObject(directions);

    if (errorObject)
      return true;

    servoAngle -= 30;
  }
}


int dirToMove(int servoStep)
{
  int previewDist = 0;
  int angleToMove = 0;
  int previewAngle, distanceToMove;
  int servoAngle = 22;
  float foreDistance, backDistance;

  for (int i = 0; i < 4; i++)
  {
    towerServo.write(servoAngle);
    delay(300);
    foreDistance = readUltrasonic(FORE_SENSOR);
    backDistance = readUltrasonic(BACK_SENSOR);

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

  if (angleToMove < angleToMoveDistMin)
    angleToMove = -1;

#ifdef SERIAL_DEBUG
  Serial.print("Object error, angle to move: ");
  Serial.println(angleToMove);
#endif

  return angleToMove;
}


void moveControl(void)
{
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
      motorMove(LEFT, pwmAutoMode, NONE);
      delay(100);
      motorMove(FOREWARD, pwmAutoMode, NONE);
    }

    else
    {
      motorMove(RIGHT, pwmAutoMode, NONE);
      delay(100);
      motorMove(FOREWARD, pwmAutoMode, NONE);
    }
  }

  delay(200);
  errorPreviewHeading = errorHeading;
  errorHeading = headingDetect();
  previewError = error;
  error = abs(errorHeading - errorPreviewHeading);

#ifdef SERIAL_DEBUG
  Serial.print("Error: ");
  Serial.print(error);
  Serial.print("  Direction (SX = 0, DX = 1):");
  Serial.println(correctionDir);
#endif
}



/* Magnetometer and compass functiones */

float headingDetect(void)
{
  float compassData[3];
  float heading;

  readMagneticAxis(compassData);
  compassCompensate(compassData);

#ifdef SERIAL_DEBUG
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

  return (heading * RAD_TO_DEG);
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



/* Ultrasonic sensor and servo control functiones */

boolean checkNearObject(int directions)
{
  float distance = readUltrasonic(directions);

  if (distance > 30)
    return false;
  else
    return true;
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



/* Control robot and motor functiones */

void motorMove(int command, int pwm, int angle)
{
  if (pwm > pwmMax)
    pwm = pwmMax;

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


void motorTurn(int angle, int pwm)
{
  if (angle == 90)
    motorMove(FOREWARD, pwm, NONE);

  else if ((angle < 90) && (angle > 0))
    motorMove(RIGHT_ANGLE, pwm, angle);

  else if ((angle < 180) && (angle > 90))
  {
    angle -= 90;
    motorMove(LEFT_ANGLE, pwm, angle);
  }

  if (angle == 270)
  {
    motorMove(LEFT_ANGLE, pwm, 270);
    motorMove(FOREWARD, pwm, NONE);
  }

  else if ((angle > 180) && (angle < 360))
  {
    motorMove(LEFT_ANGLE, pwm, 270);

    if ((angle < 270) && (angle > 180))
    {
      angle -= 180;
      motorMove(LEFT_ANGLE, pwm, angle);
    }

    if ((angle < 360) && (angle > 270))
    {
      angle -= 270;
      motorMove(RIGHT_ANGLE, pwm, angle);
    }
  }
}


// This function is used to move forward the robot at a certain velocity
void motorForward(int pwm)
{
#ifdef SERIAL_DEBUG
  Serial.print(" Forward speed: ");
  Serial.print("    ");
  Serial.println(pwm);
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
#ifdef SERIAL_DEBUG
  Serial.print(" Backward speed: ");
  Serial.print("    ");
  Serial.println(pwm);
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


void motorNormalStop(void)
{
#ifdef SERIAL_DEBUG
  Serial.println("Robot Stop");
#endif

  analogWrite(MOTOR_PIN_EN1, 0);
  analogWrite(MOTOR_PIN_EN2, 0);
  analogWrite(MOTOR_PIN_EN3, 0);
  analogWrite(MOTOR_PIN_EN4, 0);

  // Setting the inputs in order to stop
  digitalWrite(MOTOR_PIN_AM1, LOW);
  digitalWrite(MOTOR_PIN_BM1, LOW);
  digitalWrite(MOTOR_PIN_AM2, LOW);
  digitalWrite(MOTOR_PIN_BM2, LOW);
  digitalWrite(MOTOR_PIN_AM3, LOW);
  digitalWrite(MOTOR_PIN_BM3, LOW);
  digitalWrite(MOTOR_PIN_AM4, LOW);
  digitalWrite(MOTOR_PIN_BM4, LOW);
}


// This function is used to stop the motors fast
void motorFastStop(void)
{
#ifdef SERIAL_DEBUG
  Serial.println("Robot Stop");
#endif

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

#ifdef SERIAL_DEBUG
  Serial.println("motorRight");
#endif

  motorMove(RIGHT, pwm, NONE);

  while (!(headingDiff >= angle))
  {
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

    delay(10);
  }

  motorMove(stopMode, NONE, NONE);
}


// This function is used to turn right the robot at a certain velocity
void motorTurnRight(int pwm)
{
#ifdef SERIAL_DEBUG
  Serial.print("Turn right speed: ");
  Serial.println(pwm);
#endif

  analogWrite(MOTOR_PIN_EN1, 0);
  analogWrite(MOTOR_PIN_EN2, 0);
  analogWrite(MOTOR_PIN_EN3, 0);
  analogWrite(MOTOR_PIN_EN4, 0);

  // Setting the inputs in order to turn right
  digitalWrite(MOTOR_PIN_AM1, HIGH);
  digitalWrite(MOTOR_PIN_BM1, LOW);
  digitalWrite(MOTOR_PIN_AM2, HIGH);
  digitalWrite(MOTOR_PIN_BM2, LOW);
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


void motorLeft(int pwm, int angle)
{
  float heading = headingDetect();
  float previewHeading = heading;
  float headingDiff = 0;

  motorMove(RIGHT, pwm, NONE);

  while (!(headingDiff >= angle))
  {
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

    delay(10);
  }

  motorMove(stopMode, NONE, NONE);
}


// This function is used to turn left the robot at a certain velocity
void motorTurnLeft(int pwm)
{
#ifdef SERIAL_DEBUG
  Serial.print("Turn left speed:  ");
  Serial.println(pwm);
#endif

  analogWrite(MOTOR_PIN_EN1, 0);
  analogWrite(MOTOR_PIN_EN2, 0);
  analogWrite(MOTOR_PIN_EN3, 0);
  analogWrite(MOTOR_PIN_EN4, 0);

  // Setting the inputs in order to turn left
  digitalWrite(MOTOR_PIN_AM1, LOW);
  digitalWrite(MOTOR_PIN_BM1, HIGH);
  digitalWrite(MOTOR_PIN_AM2, LOW);
  digitalWrite(MOTOR_PIN_BM2, HIGH);
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

#ifdef SERIAL_DEBUG
  Serial.println("Motor initialized correctly");
#endif
}



/* RF functiones */

void serialRead(void)
{
  int index = 0;
  unsigned char dataArray[10] = {0};
  int xValue, yValue, zValue;

  /*while(Serial1.available() > 0)
  {
    dataArray[index] = Serial1.read();
    index++;
    delayMicroseconds(500);

    if(index > 10)
      break;
  }*/

  if (index == 1)
  {
    Serial.print("OneCommand: ");
    Serial.write(dataArray[0]);
    Serial.println();

    /*

    Punto in cui setti il flag per la movimentazione automatica
    autoMoveMode = true;
    motorMove(FOREWARD, 128, NONE);
    Quando autoMoveMode si spegne, spegnere il LED di movimento
    e il buffer con noTone();

    Settare i seguenti parametri:
    -> pwmAutoMode (default = 128)
    -> pwmMax (default = 255)

    Se ricevo il comando di attivazione del braccio:
    armOn = true;
    digitalWrite(ARM_SUPPLAY_ENABLE, HIGH)

    stopMode = ... (default = FAST_STOP)

    */

    index = 0;
  }

  if (index == 2)
  {
    switch (dataArray[0])
    {
      case '0':
        motorMove(stopMode, NONE, NONE);

#ifdef SERIAL_DEBUG
        Serial.println("Stopping the motor");
#endif

        break;

      case '1':
        motorMove(FOREWARD, dataArray[1], NONE);

#ifdef SERIAL_DEBUG
        Serial.print("Moving the motor forward by: ");
        Serial.print(dataArray[1]);
        Serial.println();
#endif

        break;

      case '2':
        motorMove(BACKWARD, dataArray[1], NONE);

#ifdef SERIAL_DEBUG
        Serial.print("Moving the motor backward by: ");
        Serial.print(dataArray[1]);
        Serial.println();
#endif

        break;

      case '3':
        motorMove(RIGHT, dataArray[1], NONE);

#ifdef SERIAL_DEBUG
        Serial.print("turning the motor right with: ");
        Serial.print(dataArray[1]);
        Serial.println();
#endif

        break;

      case '4':
        motorMove(LEFT, dataArray[1], NONE);

#ifdef SERIAL_DEBUG
        Serial.print("turning the motor left with: ");
        Serial.print(dataArray[1]);
        Serial.println();
#endif

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

