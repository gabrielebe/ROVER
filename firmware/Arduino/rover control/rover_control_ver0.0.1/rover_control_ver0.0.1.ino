#include <Wire.h>
#include <Servo.h>
#include <LiquidCrystal.h>
#include "Jhonny.h"

Servo pingServo;
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup()
{
  ARDUINO_init();
  ARDUINO_comunicationInit();
  MOTOR_init();

#ifdef DEBUG
  Serial.println("HMC5883 Magnetometer Initializzation");
  Serial.println();

  /* Initialise the sensor */
  if (!COMPASS_init())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while (1);
  }

#else
  COMPASS_init();
#endif
}


void loop()
{
  float distance = ULTRASONIC_readDistance();
  long time = millis();

  if ((time - COMPASS_previewTime) > COMPASS_PERIODE_MEASURE)
  {
    previewHeading = heading;
    heading = COMPASS_headingDetect();
    ROBOT_moveControl();
    COMPASS_previewTime = millis();
  }

  if ((time - ANALOG_previewTime) > ANALOG_PERIODE_MEASURE)
  {
    ANALOG_tiltSensor();
    ANALOG_temperatureRead();
    ANALOG_tempMeasurated = 1;
    ANALOG_previewTime = millis();
  }

  if (distance > DISTANCE_LIMIT)
  {
    int angleToMove = ROBOT_dirToMove();
    MOTOR_move(TURN, 128, angleToMove);
  }

  delay(10);
}


/* ARDUINO FUNCTIONES */

void ARDUINO_init(void)
{
  pingServo.attach(PING_SERVO_PIN);

#ifdef DEBUG
  lcd.begin(16, 2);
#endif
}


void ARDUINO_comunicationInit(void)
{
  Serial.begin(BAUD_RATE);
  Wire.begin();
}


/* ANALOG FUNCTIONS */

void ANALOG_temperatureRead(void)
{
  float dataSensor;
  ANALOG_tempMeasurated = 1;
  dataSensor = read8(ANALOG_ADDRESS, ANALOG_DATA_TEMP);

  /* Decodica dataSensor in degree */

}


float ANALOG_luxRead(void)
{
  float lux, dataSensor;
  dataSensor = read8(ANALOG_ADDRESS, ANALOG_DATA_LUX);

  /* Decodica dataSensor in lux */

  return lux;
}


void ANALOG_accelerometerRead(void)
{
  /* Add the request for data trasmission */

  /* Wait around until enough data is available */
  while (Wire.available() < 6);

  uint8_t xhi = Wire.read();
  uint8_t xlo = Wire.read();
  uint8_t zhi = Wire.read();
  uint8_t zlo = Wire.read();
  uint8_t yhi = Wire.read();
  uint8_t ylo = Wire.read();

  /* Shift values to create properly formed integer (low byte first) */
  int16_t x = (int16_t)(xlo | ((int16_t)xhi << 8));
  int16_t y = (int16_t)(ylo | ((int16_t)yhi << 8));
  int16_t z = (int16_t)(zlo | ((int16_t)zhi << 8));

  /* Convert the data into accelerometer value */

  ANALOG_accelerometerAxisAcceleration.x = x;
  ANALOG_accelerometerAxisAcceleration.y = y;
  ANALOG_accelerometerAxisAcceleration.z = z;
}


void ANALOG_tiltSensor(void)
{
  float theta, psi, phi;

  theta = atan2(ANALOG_accelerometerAxisAcceleration.x,
                sqrt(pow(ANALOG_accelerometerAxisAcceleration.y, 2) +
                     pow(ANALOG_accelerometerAxisAcceleration.z, 2)));


  psi = atan2(ANALOG_accelerometerAxisAcceleration.y,
              sqrt(pow(ANALOG_accelerometerAxisAcceleration.x, 2) +
                   pow(ANALOG_accelerometerAxisAcceleration.z, 2)));


  phi = atan2(sqrt(pow(ANALOG_accelerometerAxisAcceleration.x, 2) +
                   pow(ANALOG_accelerometerAxisAcceleration.y, 2)),
              ANALOG_accelerometerAxisAcceleration.z);

  ANALOG_accelerometerAxisTilt.x = theta;
  ANALOG_accelerometerAxisTilt.y = psi;
  ANALOG_accelerometerAxisTilt.z = phi;
}


/* COMPASS FUNCTIONES */

bool COMPASS_init(void)
{
  if (!COMPASS_selfTest())
    return false;

  /* Configure the magnetometer */
  write8(COMPASS_ADDRESS, COMPASS_REG_A, 0x50);
  write8(COMPASS_ADDRESS, COMPASS_REG_B, 0x00);
  write8(COMPASS_ADDRESS, COMPASS_MODE_REG, 0x00);

  delay(56);
  COMPASS_readMagneticAxis();
  return true;
}


bool COMPASS_selfTest(void)
{
  bool result;

  /* Configure the magnetometer for positive bias self
     test with gain = 5 and single measurement mode */
  write8(COMPASS_ADDRESS, COMPASS_REG_A, 0x70);
  write8(COMPASS_ADDRESS, COMPASS_REG_B, 0xA0);
  write8(COMPASS_ADDRESS, COMPASS_MODE_REG, 0x00);

  delay(6);
  COMPASS_readMagneticAxis();
  delay(67);
  COMPASS_readMagneticAxis();

  if ((COMPASS_axisMagneticData.x > COMPASS_SELF_TEST_LL) && (COMPASS_axisMagneticData.x < COMPASS_SELF_TEST_HL) &&
      (COMPASS_axisMagneticData.y > COMPASS_SELF_TEST_LL) && (COMPASS_axisMagneticData.y < COMPASS_SELF_TEST_HL) &&
      (COMPASS_axisMagneticData.z > COMPASS_SELF_TEST_LL) && (COMPASS_axisMagneticData.z < COMPASS_SELF_TEST_HL))
    result = true;
  else
    result = false;

  //COMPASS_init();
  return result;
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


void COMPASS_readMagneticAxis(void)
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
  int16_t x = (int16_t)(xlo | ((int16_t)xhi << 8));
  int16_t y = (int16_t)(ylo | ((int16_t)yhi << 8));
  int16_t z = (int16_t)(zlo | ((int16_t)zhi << 8));

  COMPASS_axisMagneticData.x = (float)x * COMPASS_GAUSS_LSB;
  COMPASS_axisMagneticData.y = (float)y * COMPASS_GAUSS_LSB;
  COMPASS_axisMagneticData.z = (float)z * COMPASS_GAUSS_LSB;
}


float COMPASS_headingDetect(void)
{
  float Xh, Yh, atanArg;
  float heading;

  /* -1 means error in heading determination */
  float headingDegrees = -1;

  COMPASS_readMagneticAxis();

  Xh = COMPASS_axisMagneticData.x * cos(ANALOG_accelerometerAxisTilt.z) +
       COMPASS_axisMagneticData.y * sin(ANALOG_accelerometerAxisTilt.x) -
       COMPASS_axisMagneticData.z * cos(ANALOG_accelerometerAxisTilt.z) *
       sin(ANALOG_accelerometerAxisTilt.x);

  Yh = COMPASS_axisMagneticData.y * cos(ANALOG_accelerometerAxisTilt.x) +
       COMPASS_axisMagneticData.z * sin(ANALOG_accelerometerAxisTilt.x);

  /*if(Xh == 0)
  {
      if(Yh > 0)
          heading = 270;
      else if(Yh < 0)
          heading = 90;
  }
  else
  {
      atanArg = Yh / Xh;
      heading = atan(atanArg);

      if(Xh < 0)
          heading = 180 - heading;
      else if((Xh > 0) && (Yh < 0))
          heading = - heading;
      else if((Xh > 0) && (Yh > 0))
          heading = 360 - heading;
  }*/

  if (Xh && Yh)
    heading = atan2(Yh, Xh);

  /* Once you have your heading, you must then add your 'Declination Angle',
   which is the 'Error' of the magnetic field in your location.
   Find yours here: http://www.magnetic-declination.com/
   Mine is: +2° 10' , which is ~2 Degrees, or (which we need) 0.035 radians */
  heading = heading + COMPASS_DECLINATION_ANGLE;

  /* Correct for when signs are reversed */
  if (heading < 0)
    heading = heading + 2 * PI;

  /* Check for wrap due to addition of declination */
  if (heading > 2 * PI)
    heading = heading - 2 * PI;

  /* Convert radians to degrees for readability */
  headingDegrees = degrees(heading);

  return headingDegrees;
}


/* Retrieve the value of the three ID registers.
   The HMC5883L has the same 'H43' identification register values */
void COMPASS_getID(char id[3])
{
  int i = 0;
  Wire.beginTransmission(COMPASS_ADDRESS);

  /* Will start reading registers starting from Identification Register A */
  Wire.write(COMPASS_IDENTIFICATION_A);

  Wire.endTransmission();
  Wire.beginTransmission(COMPASS_ADDRESS);
  Wire.requestFrom(COMPASS_ADDRESS, 3);

  while (Wire.available())
  {
    id[i] = Wire.read();
    i++;
  }

  Wire.endTransmission();
}


/* MOTOR FUNCTIONES */

void MOTOR_init(void)
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

  /* pulling down all the input for security reasons */
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


void MOTOR_move(motorDirection comand, int pwm, int angle)
{
  switch (comand)
  {
    case TURN:
      MOTOR_turn(pwm, angle);
      break;

    case FOREWARD:
      MOTOR_forward(pwm);
      break;

    case BACKWARD:
      MOTOR_backward(pwm);
      break;

    case STOP:
      MOTOR_stop();
      break;

    case RIGHT:
      MOTOR_right(pwm, angle);
      break;

    case LEFT:
      MOTOR_left(pwm, angle);
      break;
  }
}


void MOTOR_turn(int angle, int pwm)
{
  if ((angle >= 0) && (angle <= 360))
  {
    if (angle == 90)
      MOTOR_forward(pwm);
    else if (angle == 270)
      MOTOR_backward(pwm);
    else if (angle > 90)
      MOTOR_right(pwm, angle);
    else if ((angle < 180) && (angle > 90))
    {
      angle = 90 - angle;
      MOTOR_left(pwm, angle);
    }
  }
}


/* This function is used to move forward the robot at a certain velocity */
void MOTOR_forward(int pwm)
{
#ifdef DEBUG
  /* The following part is used only to display on the LCD the speed and the
  movement that the robot is doing */
  lcd.clear();
  lcd.print(" Forward speed: ");
  lcd.setCursor(0, 1);
  lcd.print("    ");
  lcd.print(pwm);
#endif

  /* setting the inputs in order to go forward */
  digitalWrite(MOTOR_PIN_AM1, LOW);
  digitalWrite(MOTOR_PIN_BM1, HIGH);
  digitalWrite(MOTOR_PIN_AM2, LOW);
  digitalWrite(MOTOR_PIN_BM2, HIGH);
  digitalWrite(MOTOR_PIN_AM3, LOW);
  digitalWrite(MOTOR_PIN_BM3, HIGH);
  digitalWrite(MOTOR_PIN_AM4, LOW);
  digitalWrite(MOTOR_PIN_BM4, HIGH);

  /* setting the pwm signal given as a parameter to this function */
  analogWrite(MOTOR_PIN_EN1, pwm);
  analogWrite(MOTOR_PIN_EN2, pwm);
  analogWrite(MOTOR_PIN_EN3, pwm);
  analogWrite(MOTOR_PIN_EN4, pwm);
}


/* This function is used to move backward the robot at a certain velocity */
void MOTOR_backward(int pwm)
{
#ifdef DEBUG
  /* The following part is used only to display on the LCD the speed and the
  movement that the robot is doing */
  lcd.clear();
  lcd.print(" Forward speed: ");
  lcd.setCursor(0, 1);
  lcd.print("    ");
  lcd.print(pwm);
#endif

  /* setting the inputs in order to go backward */
  digitalWrite(MOTOR_PIN_AM1, HIGH);
  digitalWrite(MOTOR_PIN_BM1, LOW);
  digitalWrite(MOTOR_PIN_AM2, HIGH);
  digitalWrite(MOTOR_PIN_BM2, LOW);
  digitalWrite(MOTOR_PIN_AM3, HIGH);
  digitalWrite(MOTOR_PIN_BM3, LOW);
  digitalWrite(MOTOR_PIN_AM4, HIGH);
  digitalWrite(MOTOR_PIN_BM4, LOW);

  /* setting the pwm signal given as a parameter to this function */
  analogWrite(MOTOR_PIN_EN1, pwm);
  analogWrite(MOTOR_PIN_EN2, pwm);
  analogWrite(MOTOR_PIN_EN3, pwm);
  analogWrite(MOTOR_PIN_EN4, pwm);
}


/* This function is used to stop the motors fast */
void MOTOR_stop(void)
{
  /* setting the inputs in order to stop motor */
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


void MOTOR_right(int pwm, int angle)
{
  if ((angle >= 0) && (angle <= 90))
  {
    float previewHeading = 0;
    float heading = COMPASS_headingDetect();

    while ((abs(heading - previewHeading)) > HEADING_LIMIT)
    {
      MOTOR_turnRight(pwm);
      delay(100);
      MOTOR_stop();
      delay(50);
      previewHeading = heading;
      heading = COMPASS_headingDetect();
    }
  }
}


/* This function is used to turn right the robot at a certain velocity */
void MOTOR_turnRight(int pwm)
{
#ifdef LCD
  /* The following part is used only to display on the LCD the speed and the
  movement that the robot is doing */
  lcd.clear();
  lcd.print("Turn right speed");
  lcd.setCursor(0, 1);
  lcd.print("    ");
  lcd.print(pwm);
#endif

  /* setting the pwm signal given as a parameter to this function */
  analogWrite(MOTOR_PIN_EN1, 0);
  analogWrite(MOTOR_PIN_EN2, 0);
  analogWrite(MOTOR_PIN_EN3, 0);
  analogWrite(MOTOR_PIN_EN4, 0);

  /* setting the inputs in order to go forward */
  digitalWrite(MOTOR_PIN_AM1, HIGH);
  digitalWrite(MOTOR_PIN_BM1, LOW);
  digitalWrite(MOTOR_PIN_AM2, LOW);
  digitalWrite(MOTOR_PIN_BM2, HIGH);
  digitalWrite(MOTOR_PIN_AM3, HIGH);
  digitalWrite(MOTOR_PIN_BM3, LOW);
  digitalWrite(MOTOR_PIN_AM4, LOW);
  digitalWrite(MOTOR_PIN_BM4, HIGH);

  /* setting the pwm signal given as a parameter to this function */
  analogWrite(MOTOR_PIN_EN1, pwm);
  analogWrite(MOTOR_PIN_EN2, pwm);
  analogWrite(MOTOR_PIN_EN3, pwm);
  analogWrite(MOTOR_PIN_EN4, pwm);
}


void MOTOR_left(int pwm, int angle)
{
  if ((angle >= 0) && (angle <= 90))
  {
    float previewHeading = 0;
    float heading = COMPASS_headingDetect();

    while ((abs(heading - previewHeading)) > HEADING_LIMIT)
    {
      //turnLeft;  IMPLEMENTA FUNZIONI
      delay(100);
      //stop;       IMPLEMENTA FUNZIONI
      delay(50);
      previewHeading = heading;
      heading = COMPASS_headingDetect();
    }
  }
}


/* This function is used to turn left the robot at a certain velocity */
void MOTOR_turnLeft(int pwm)
{
#ifdef LCD
  /* The following part is used only to display on the LCD the speed and the
  movement that the robot is doing */
  lcd.clear();
  lcd.print("Turn left speed");
  lcd.setCursor(0, 1);
  lcd.print("    ");
  lcd.print(pwm);
#endif

  /* setting the pwm signal given as a parameter to this function */
  analogWrite(MOTOR_PIN_EN1, 0);
  analogWrite(MOTOR_PIN_EN2, 0);
  analogWrite(MOTOR_PIN_EN3, 0);
  analogWrite(MOTOR_PIN_EN4, 0);

  /* setting the inputs in order to go forward */
  digitalWrite(MOTOR_PIN_AM1, LOW);
  digitalWrite(MOTOR_PIN_BM1, HIGH);
  digitalWrite(MOTOR_PIN_AM2, HIGH);
  digitalWrite(MOTOR_PIN_BM2, LOW);
  digitalWrite(MOTOR_PIN_AM3, LOW);
  digitalWrite(MOTOR_PIN_BM3, HIGH);
  digitalWrite(MOTOR_PIN_AM4, HIGH);
  digitalWrite(MOTOR_PIN_BM4, LOW);

  /* setting the pwm signal given as a parameter to this function */
  analogWrite(MOTOR_PIN_EN1, pwm);
  analogWrite(MOTOR_PIN_EN2, pwm);
  analogWrite(MOTOR_PIN_EN3, pwm);
  analogWrite(MOTOR_PIN_EN4, pwm);
}


/* ROBOT FUNCTIONES*/

void ROBOT_moveControl(void)
{
  previewError = error;
  error = abs(previewHeading - heading);

  if (error > previewError)
  {
    if (correctionDir == SX)
      correctionDir = DX;
    else
      correctionDir = SX;
  }

  if (error >= HEADING_LIMIT)
  {
    if (correctionDir == SX)
    {
      MOTOR_move(LEFT, 150, NONE);
      delay(50);
      MOTOR_move(STOP, NONE, NONE);
      /*previewHeading = heading;
      heading = COMPASS_headingDetect();
      previewError = error;
      error = abs(previewHeading - heading);

      if(error > previewError)
          correctionDir = DX;*/
    }
    else
    {
      MOTOR_move(RIGHT, 150, NONE);
      delay(50);
      MOTOR_move(STOP, NONE, NONE);
      /*previewHeading = heading;
      heading = COMPASS_headingDetect();
      previewError = error;
      error = abs(previewHeading - heading);

      if(error > previewError)
          correctionDir = SX;*/
    }
  }
}


int ROBOT_dirToMove(void)
{
  int previewDist = 0;
  int servoAngle = 0;
  int distance, angleToMove, distanceToMove;

  while (servoAngle <= 180)
  {
    pingServo.write(servoAngle);
    delay(100);
    distance = ULTRASONIC_readDistance();

    if (distance > previewDist)
    {
      angleToMove = servoAngle;
      distanceToMove = distance;
    }

    previewDist = distance;
    servoAngle = servoAngle + 20;
  }

  if (distance < DISTANCE_LIMIT)
  {
    MOTOR_move(BACKWARD, 50, NONE);
    delay(1000);
    MOTOR_move(STOP, NONE, NONE);
  }

  return angleToMove;
}


float ULTRASONIC_readDistance(void)
{
  unsigned long timeUltrasonicResponse, microsecondsToCm;
  unsigned long currentMillis = millis();
  static float microsecondsPerCm, sensorOffset;
  float netDistance;

  if (ANALOG_tempMeasurated)
  {
    microsecondsPerCm = 1 / ((331.5 + (0.6 * temperatura)) / 10000);
    sensorOffset = SENSOR_GAP * microsecondsPerCm * 2;
    ANALOG_tempMeasurated = 0;
  }

  pinMode(PING_SENSOR_PIN, OUTPUT);
  digitalWrite(PING_SENSOR_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(PING_SENSOR_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(PING_SENSOR_PIN, LOW);
  pinMode(PING_SENSOR_PIN, INPUT);
  timeUltrasonicResponse = pulseIn(PING_SENSOR_PIN, HIGH);
  netDistance = max(0, timeUltrasonicResponse - sensorOffset);
  microsecondsToCm = netDistance / microsecondsPerCm / 2;

  return microsecondsToCm;
}

