#include <Servo.h>
#include <LiquidCrystal.h>
#include "Jhonny.h"

Servo pingServo;
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup()
{
  ARDUINO_init();
  ARDUINO_comunicationInit();
  ANALOG_init();
  COMPASS_init();
  MOTOR_init();
}


void loop()
{

  delay(10);
}


/* ARDUINO FUNCTIONES */

void ARDUINO_init(void)
{
  pingServo.attach(PING_SERVO_PIN);
  lcd.begin(16, 2);
}


void ARDUINO_comunicationInit(void)
{
  Serial.begin(BAUD_RATE);
}


/* ANALOG FUNCTIONS */

void ANALOG_init(void)
{
}


float ANALOG_temperatureRead(void)
{
  float temperature = 0;
  ANALOG_tempMeasurated = 1;

  return temperature;
}


float ANALOG_luxRead(void)
{
  float lux;

  return lux;
}


float* ANALOG_accelerometerRead(void)
{
  float accelerometer[3];

  return accelerometer;
}


float* ANALOG_tiltSensor(void)
{
  float tilt[2];

  return tilt;
}


/* COMPASS FUNCTIONES */

void COMPASS_init(void)
{

  /* Firs read of all data for initialization value */
  COMPASS_readValue();
}


float COMPASS_headingDetect(void)
{
  float Xh, Yh, atanArg;
  float heading;

  if (COMPASS_newMeasure)
  {
    Xh = COMPASS_X * cos(ANALOG_phi) + COMPASS_Y * sin(ANALOG_teta) - COMPASS_Z * cos(ANALOG_phi) * sin(ANALOG_teta);
    Yh = COMPASS_Y * cos(ANALOG_teta) + COMPASS_Z * sin(ANALOG_teta);
    COMPASS_newMeasure = 0;
  }

  if (Xh == 0)
  {
    if (Yh > 0)
      heading = 270;
    else if (Yh < 0)
      heading = 90;
  }
  else
  {
    atanArg = Yh / Xh;
    heading = atan(atanArg);

    if (Xh < 0)
      heading = 180 - heading;
    else if ((Xh > 0) && (Yh < 0))
      heading = - heading;
    else if ((Xh > 0) && (Yh > 0))
      heading = 360 - heading;
  }

  return heading;
}


void COMPASS_readValue(void)
{

  COMPASS_newMeasure = 1;
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


void MOTOR_move(int comand, int pwm, int angle)
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


/* This function is used to stop the motors fast */
void MOTOR_stop(void)
{
  /* setting the inputs in order to go forward */
  digitalWrite(MOTOR_PIN_AM1, HIGH);
  digitalWrite(MOTOR_PIN_BM1, HIGH);
  digitalWrite(MOTOR_PIN_AM2, HIGH);
  digitalWrite(MOTOR_PIN_BM2, HIGH);
  digitalWrite(MOTOR_PIN_AM3, HIGH);
  digitalWrite(MOTOR_PIN_BM3, HIGH);
  digitalWrite(MOTOR_PIN_AM4, HIGH);
  digitalWrite(MOTOR_PIN_BM4, HIGH);

  /* setting the pwm signal given as a parameter to this function */
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
  float heading = COMPASS_headingDetect();
  float error = abs(previewHeading - heading);

  if (error >= HEADING_LIMIT)
  {
    if (correctionDir == SX)
    {
      MOTOR_move(LEFT, 150, NONE);
      delay(50);
      MOTOR_move(STOP, NONE, NONE);
      previewHeading = heading;
      heading = COMPASS_headingDetect();
      previewError = error;
      error = abs(previewHeading - heading);

      if (error > previewError)
        correctionDir = DX;
    }
    else
    {
      MOTOR_move(RIGHT, 150, NONE);
      delay(50);
      MOTOR_move(STOP, NONE, NONE);
      previewHeading = heading;
      heading = COMPASS_headingDetect();
      previewError = error;
      error = abs(previewHeading - heading);

      if (error > previewError)
        correctionDir = SX;
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
    microsecondsPerCm = 1 / ((331.5 + (0.6 * ANALOG_temperatureRead())) / 10000);
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

