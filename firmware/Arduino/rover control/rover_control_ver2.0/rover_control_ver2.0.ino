/******************************************************
* Authors: Gabriele Bè - Harpal Singh - Alice Rosa    *
* Date: xx/05/2015                                    *
* Version: V2.0                                       *
* Project name: Rover Control Board                   *
******************************************************/

#include <Servo.h>
Servo towerServo;

#define MANUAL_STATE 0
#define AUTO_STATE   1
#define SERVO_10     0
#define SERVO_90     1
#define SERVO_170    2
#define DISTANCE_10  3
#define DISTANCE_170 4
#define BACK_CONTROL 5
#define NORMAL_STOP  0
#define FAST_STOP    1

#define ARM_SUPPLAY_ENABLE 12
#define SERVO_PIN          44
#define ECHO_PIN           34
#define TRIG_PIN           36
#define BATTERY_PIN        A4
#define BATTERY_LIMIT      (15.0F)
#define BATT_MAX_SUPPLY    (18.2F)

unsigned int state = MANUAL_STATE;
unsigned int autoStates = SERVO_90;
unsigned long tempLastTime = 0;
unsigned long battLastTime = 0;

unsigned int pwmMax = 170;
unsigned int stopMode = NORMAL_STOP;
unsigned int autoVelocity = 100;
unsigned int turnVelocity = autoVelocity + 50;

float temperature = 20.0;
float microsecondsPerCm = 0.034385;
float sensorOffset = 0.013754;
float battVoltage = 0.0;
int battSample = 0.0;


void setup()
{
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ARM_SUPPLAY_ENABLE, OUTPUT);
  digitalWrite(ARM_SUPPLAY_ENABLE, LOW);
  motorInit();
  towerServo.attach(SERVO_PIN);
  temperature = getTemperature();
  microsecondsPerCm = 1 / ((331.45 + (0.62 * temperature)) / 10000);
  sensorOffset = 0.2 * microsecondsPerCm * 2;
  compassInit();
  accelerometerCalibration();
  Serial.begin(115200);
  Serial1.begin(38400);
}


void loop()
{
  if ((millis() - tempLastTime) > 4000)
  {
    temperature = getTemperature();
    microsecondsPerCm = 1 / ((331.45 + (0.62 * temperature)) / 10000);
    sensorOffset = 0.2 * microsecondsPerCm * 2;
    tempLastTime = millis();
  }

  if ((millis() - battLastTime) > 10000)
  {
    battSample = analogRead(BATTERY_PIN);
    battVoltage = (battSample * 5.0) / 1024;
    pwmMax = int(float(3825) / battVoltage);

    if (battVoltage < BATTERY_LIMIT)
    {

    }
  }

  switch (state)
  {
    case MANUAL_STATE:
      serialRead();
      break;

    case AUTO_STATE:
      autoStateOut();
      serialReadAuto();
      autoMove();
      break;
  }
}


void autoMove(void)
{
  switch (autoStates)
  {
    case SERVO_10:
      servoGetObj(10);
      break;

    case SERVO_90:
      servoGetObj(90);
      break;

    case SERVO_170:
      servoGetObj(170);
      break;

    case DISTANCE_10:
      measureObjDx();
      break;

    case DISTANCE_170:
      measureObjSx();
      break;

    case BACK_CONTROL:
      backControl();
      break;
  }
}

