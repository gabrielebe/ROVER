#include "motor.h"
#include <Wire.h>

#ifdef DEBUG_LCD
#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
#endif


void setup()
{
#ifdef DEBUG_SERIAL
  Serial.begin(BAUD_RATE);
#endif

  motorInit();
  compassInit();
  controlTest();
}


void loop()
{
  /*if((millis() - magnLastTime) > 100)
  {
  previewHeading = heading;
      heading = headingDetect();
  magnLastTime = millis();

  #ifdef DEBUG_SERIAL
    Serial.println();
    Serial.print("Heading: ");
          Serial.println(heading);
      #endif
  }*/

  delay(1);
}


void controlTest(void)
{
  /*motorBaseTest();
  motorRight(50, 30);
  delay(5000);
  motorLeft(150, 60);
  delay(5000);*/

  /*motorTurn(90, 150);
  delay(2000);
  motorTurn(270, 50);
  delay(2000);
  motorTurn(30, 50);
  delay(2000);
  motorTurn(250, 50);
  delay(2000);*/

  /*motorMove(TURN, 128, 60);
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
  delay(2000);*/

  /*moveControl(FOREWARD, 150);
  moveControl(BACKWARD, 150);*/
}


void motorBaseTest(void)
{
  for (int i = 0; i < 255; i += 100)
  {
#ifdef DEBUG_SERIAL
    Serial.print("Velocità pwm avanti: ");
    Serial.println(i);
#endif

    motorForward(i);
    delay(2000);
    motorStop();

#ifdef DEBUG_SERIAL
    Serial.println("Stop!");
#endif

    delay(2000);
  }

  for (int i = 0; i < 255; i += 100)
  {
#ifdef DEBUG_SERIAL
    Serial.print("Pwm backward speed: ");
    Serial.println(i);
#endif

    motorBackward(i);
    delay(2000);
    motorStop();

#ifdef DEBUG_SERIAL
    Serial.println("Stop!");
#endif

    delay(2000);
  }

  for (int i = 0; i < 255; i += 100)
  {
#ifdef DEBUG_SERIAL
    Serial.print("Pwm right speed: ");
    Serial.println(i);
#endif

    motorTurnRight(i);
    delay(2000);
    motorStop();

#ifdef DEBUG_SERIAL
    Serial.println("Stop!");
#endif

    delay(2000);
  }

  for (int i = 0; i < 255; i += 100)
  {
#ifdef DEBUG_SERIAL
    Serial.print("Pwm left speed: ");
    Serial.println(i);
#endif

    motorTurnLeft(i);
    delay(2000);
    motorStop();

#ifdef DEBUG_SERIAL
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

#ifdef DEBUG_SERIAL
    Serial.println();
    Serial.print("Error: ");
    Serial.print(error);
    Serial.print("   Error heading: ");
    Serial.print(errorHeading);
    Serial.print("   Direction: ");
    Serial.println(correctionDir);
#endif

    delay(100);
  }
}


void motorInit(void)
{
  /* Setting the pin directions to initialize the motor */
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

  /* Pulling down all the input for security reasons */
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


/* This function is used to move forward the robot at a certain velocity */
void motorForward(int pwm)
{
#ifdef DEBUG_LCD
  /* The following part is used only to display on the LCD the speed and the
     movement that the robot is doing */
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

  /* Setting the inputs in order to go forward */
  digitalWrite(MOTOR_PIN_AM1, LOW);
  digitalWrite(MOTOR_PIN_BM1, HIGH);
  digitalWrite(MOTOR_PIN_AM2, LOW);
  digitalWrite(MOTOR_PIN_BM2, HIGH);
  digitalWrite(MOTOR_PIN_AM3, LOW);
  digitalWrite(MOTOR_PIN_BM3, HIGH);
  digitalWrite(MOTOR_PIN_AM4, LOW);
  digitalWrite(MOTOR_PIN_BM4, HIGH);

  /* Setting the pwm signal given as a parameter to this function */
  analogWrite(MOTOR_PIN_EN1, pwm);
  analogWrite(MOTOR_PIN_EN2, pwm);
  analogWrite(MOTOR_PIN_EN3, pwm);
  analogWrite(MOTOR_PIN_EN4, pwm);
}


/* This function is used to move backward the robot at a certain velocity */
void motorBackward(int pwm)
{
#ifdef DEBUG_LCD
  /* The following part is used only to display on the LCD the speed and the
     movement that the robot is doing */
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

  /* Setting the inputs in order to go backward */
  digitalWrite(MOTOR_PIN_AM1, HIGH);
  digitalWrite(MOTOR_PIN_BM1, LOW);
  digitalWrite(MOTOR_PIN_AM2, HIGH);
  digitalWrite(MOTOR_PIN_BM2, LOW);
  digitalWrite(MOTOR_PIN_AM3, HIGH);
  digitalWrite(MOTOR_PIN_BM3, LOW);
  digitalWrite(MOTOR_PIN_AM4, HIGH);
  digitalWrite(MOTOR_PIN_BM4, LOW);

  /* Setting the pwm signal given as a parameter to this function */
  analogWrite(MOTOR_PIN_EN1, pwm);
  analogWrite(MOTOR_PIN_EN2, pwm);
  analogWrite(MOTOR_PIN_EN3, pwm);
  analogWrite(MOTOR_PIN_EN4, pwm);
}


/* This function is used to stop the motors fast */
void motorStop(void)
{
  analogWrite(MOTOR_PIN_EN1, 0);
  analogWrite(MOTOR_PIN_EN2, 0);
  analogWrite(MOTOR_PIN_EN3, 0);
  analogWrite(MOTOR_PIN_EN4, 0);

  /* Setting the inputs in order to stop */
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

  while (!(headingDiff >= (angle - HEADING_LIMIT)))
  {
    motorTurnRight(pwm);
    delay(100);
    motorStop();
    delay(50);
    heading = headingDetect();
    headingDiff = abs(heading - previewHeading);

#ifdef DEBUG_SERIAL
    Serial.println();
    Serial.print("Heading: ");
    Serial.print(heading);
    Serial.print("   Preview heading: ");
    Serial.print(previewHeading);
    Serial.print("   Heading diff: ");
    Serial.println(headingDiff);
#endif
  }
}


/* This function is used to turn right the robot at a certain velocity */
void motorTurnRight(int pwm)
{
#ifdef DEBUG_LCD
  /* The following part is used only to display on the LCD the speed and the
     movement that the robot is doing */
  lcd.clear();
  lcd.print("Turn right speed");
  lcd.setCursor(0, 1);
  lcd.print("    ");
  lcd.print(pwm);
#endif

  /* Setting the pwm signal given as a parameter to this function */
  analogWrite(MOTOR_PIN_EN1, 0);
  analogWrite(MOTOR_PIN_EN2, 0);
  analogWrite(MOTOR_PIN_EN3, 0);
  analogWrite(MOTOR_PIN_EN4, 0);

  /* Setting the inputs in order to go forward */
  digitalWrite(MOTOR_PIN_AM1, HIGH);
  digitalWrite(MOTOR_PIN_BM1, LOW);
  digitalWrite(MOTOR_PIN_AM2, LOW);
  digitalWrite(MOTOR_PIN_BM2, HIGH);
  digitalWrite(MOTOR_PIN_AM3, HIGH);
  digitalWrite(MOTOR_PIN_BM3, LOW);
  digitalWrite(MOTOR_PIN_AM4, LOW);
  digitalWrite(MOTOR_PIN_BM4, HIGH);

  /* Setting the pwm signal given as a parameter to this function */
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
    Serial.print("   Preview heading: ");
    Serial.print(previewHeading);
    Serial.print("   Heading diff: ");
    Serial.println(headingDiff);
#endif
  }
}


/* This function is used to turn left the robot at a certain velocity */
void motorTurnLeft(int pwm)
{
#ifdef DEBUG_LCD
  /* The following part is used only to display on the LCD the speed and the
     movement that the robot is doing */
  lcd.clear();
  lcd.print("Turn left speed");
  lcd.setCursor(0, 1);
  lcd.print("    ");
  lcd.print(pwm);
#endif

  /* Setting the pwm signal given as a parameter to this function */
  analogWrite(MOTOR_PIN_EN1, 0);
  analogWrite(MOTOR_PIN_EN2, 0);
  analogWrite(MOTOR_PIN_EN3, 0);
  analogWrite(MOTOR_PIN_EN4, 0);

  /* Setting the inputs in order to go forward */
  digitalWrite(MOTOR_PIN_AM1, LOW);
  digitalWrite(MOTOR_PIN_BM1, HIGH);
  digitalWrite(MOTOR_PIN_AM2, HIGH);
  digitalWrite(MOTOR_PIN_BM2, LOW);
  digitalWrite(MOTOR_PIN_AM3, LOW);
  digitalWrite(MOTOR_PIN_BM3, HIGH);
  digitalWrite(MOTOR_PIN_AM4, HIGH);
  digitalWrite(MOTOR_PIN_BM4, LOW);

  /* Setting the pwm signal given as a parameter to this function */
  analogWrite(MOTOR_PIN_EN1, pwm);
  analogWrite(MOTOR_PIN_EN2, pwm);
  analogWrite(MOTOR_PIN_EN3, pwm);
  analogWrite(MOTOR_PIN_EN4, pwm);
}


void compassInit(void)
{
  /* Configure the magnetometer */
  write8(COMPASS_ADDRESS, COMPASS_REG_A, 0x50);
  write8(COMPASS_ADDRESS, COMPASS_REG_B, 0x00);
  write8(COMPASS_ADDRESS, COMPASS_MODE_REG, 0x00);
  delay(COMPASS_PERIODE_MEASURE);
}


float headingDetect(void)
{
  float compassData[3];
  float heading;

  readMagneticAxis(compassData);
  compassCompensate(compassData);

  /*#ifdef DEBUG_SERIAL
      Serial.println();
  Serial.print("X magnetometer: ");
  Serial.print(compassData[0]);
      Serial.print(",  ");
      Serial.print("  Y magnetometer: ");
  Serial.print(compassData[1]);
      Serial.print(",  ");
  Serial.print("  Z magnetometer: ");
      Serial.println(compassData[2]);
  #endif*/

  if (compassData[1] && compassData[0])
    heading = atan2(compassData[1], compassData[0]);

  /* Correct for when signs are reversed */
  if (heading < 0)
    heading = heading + 2 * PI;

  /* Check for wrap due to addition of declination */
  if (heading > 2 * PI)
    heading = heading - 2 * PI;

  return (heading * RAD_TO_DEG);
}


void compassCompensate(float compassData[3])
{
  /* calibrationMatrix[3][3] is the transformation matrix */
  double calibrationMatrix[3][3] =
  {
    {1.019, 0.001, 0.003},
    {0.032, 0.97, -0.048},
    { -0.02, -0.017, 1.274}
  };

  /* bias[3] is the bias vector */
  double bias[3] =
  {
    -34.311,
    -212.987,
    -122.717
  };

  /* Application of calibration and compensation formula */
  for (int i = 0; i < 3; ++i)
    compassData[i] = compassData[i] - bias[i];

  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      compassData[i] += calibrationMatrix[i][j] * compassData[j];
}


void readMagneticAxis(float rawData[3])
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
  rawData[0] = (float)((int16_t)(xlo | ((int16_t)xhi << 8))) * COMPASS_GAUSS_LSB;
  rawData[1] = (float)((int16_t)(ylo | ((int16_t)yhi << 8))) * COMPASS_GAUSS_LSB;
  rawData[2] = (float)((int16_t)(zlo | ((int16_t)zhi << 8))) * COMPASS_GAUSS_LSB;
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

