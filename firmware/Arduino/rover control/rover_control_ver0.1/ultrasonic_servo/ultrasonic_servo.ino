#include <Servo.h>

#define SERIAL_DEBUG
#define BAUD_RATE 115200
#define SERVO_PIN 9
#define PING_PIN 7
#define ECHO_PIN 2
#define TRIG_PIN 4
#define TEMP_PIN A3
#define FORE_SENSOR 0
#define BACK_SENSOR 1

int getObjectContinuos(int directions);
bool getErrorObject(int directions);
int dirToMove(int servoStep);
float readUltrasonic(int nSensor);
float getTemperature(void);

Servo towerServo;

float temperature;
unsigned long tempLastTime = 0;
unsigned long distLastTime = 0;

void setup()
{
#ifdef SERIAL_DEBUG
  Serial.begin(BAUD_RATE);
#endif

  towerServo.attach(SERVO_PIN);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  temperature = getTemperature();

  /*#ifdef SERIAL_DEBUG
          Serial.print("Temperature:  ");
          Serial.println(temperature);
  #endif*/

  /* getObjectContinuos(FORE_SENSOR); */
}


void loop()
{
  /*if((millis() - tempLastTime) > (10 * 1000))
  {
      temperature = getTemperature();
      tempLastTime = millis();

      #ifdef SERIAL_DEBUG
          Serial.print("Temperature:  ");
          Serial.print(temperature);
      #endif
  }

  if((millis() - distLastTime) > 500)
  {
      float distFore = readUltrasonic(FORE_SENSOR);
      float distBack = readUltrasonic(BACK_SENSOR);
      distLastTime = millis();

      #ifdef SERIAL_DEBUG
          Serial.print("   Distance foreward:  ");
          Serial.print(distFore);
          Serial.print("   Distance backward:  ");
          Serial.print(distBack);
          Serial.println();
      #endif
  }*/

  /*bool foreObject = getErrorObject(FORE_SENSOR);
  bool backObject = getErrorObject(BACK_SENSOR);
  Serial.print("Foreward object: ");
  Serial.print(foreObject);
  Serial.print("   Backward object: ");
  Serial.println(backObject);*/

  delay(100);
}


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
    Serial.print("   Distance foreward:  ");
    Serial.print(foreDistance);
    Serial.print("   Distance backward:  ");
    Serial.print(backDistance);
    Serial.println();
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


float getTemperature(void)
{
  int sensorVoltage = analogRead(TEMP_PIN);
  float voltage = (sensorVoltage * 5.0) / 1023;
  return (((voltage * 1000) - 500.0) / 10.0);
}

