#define PING_PIN   46
#define TEMP_PIN   A3
#define AUTO_LED   45
#define BUZZER_PIN 47

#define FORE_SENSOR               0
#define BACK_SENSOR               1
#define TIMEOUT_ULTRASONIC_MICROS 50000
#define COLLISION_LIMIT           50

boolean autoToggle = true;
boolean towerFirst = true;
boolean dxMeasureFirst = true;
boolean sxMeasureFirst = true;
boolean backMeasureFirst = true;
unsigned int dxDistance = 0;
unsigned int sxDistance = 0;

unsigned long towerMillis = 0;
unsigned long autoLastTime = 0;
unsigned long sxMeasureTime = 0;
unsigned long dxMeasureTime = 0;
unsigned long backControlTime = 0;


void autoStateOut(void)
{
  if ((millis() - autoLastTime) > 500)
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

    autoLastTime = millis();
  }
}


void servoGetObj(unsigned int angle)
{
  towerServo.write(angle);

  if (towerFirst)
  {
    towerMillis = millis();
    towerFirst = false;
  }

  if ((millis() - towerMillis) > 500)
  {
    towerFirst = true;
    if (checkObject(FORE_SENSOR))
    {
      autoStates = DISTANCE_10;
    }
    else
    {
      switch (angle)
      {
        case 10:
          autoStates = SERVO_90;
          break;

        case 90:
          if (autoStates == SERVO_10)
            autoStates = SERVO_170;
          else if (autoStates == SERVO_170)
            autoStates = SERVO_10;
          else
            autoStates = SERVO_10;

          break;

        case 170:
          autoStates = SERVO_90;
          break;
      }
    }
  }
}


void measureObjDx(void)
{
  motorFastStop();
  towerServo.write(10);

  if (dxMeasureFirst)
  {
    dxMeasureFirst = false;
    dxMeasureTime = millis();
  }

  if ((millis() - dxMeasureTime) > 500)
  {
    dxMeasureFirst = true;
    dxDistance = readUltrasonic(FORE_SENSOR);
    autoStates = DISTANCE_170;
  }
}


void measureObjSx(void)
{
  towerServo.write(170);

  if (sxMeasureFirst)
  {
    sxMeasureFirst = false;
    sxMeasureTime = millis();
  }

  if ((millis() - sxMeasureTime) > 500)
  {
    sxMeasureFirst = true;
    sxDistance = readUltrasonic(FORE_SENSOR);

    if ((sxDistance > COLLISION_LIMIT) || (dxDistance > COLLISION_LIMIT))
    {
      if (sxDistance >= dxDistance)
        motorLeft(turnVelocity, 80);
      else
        motorRight(turnVelocity, 80);

      motorForward(autoVelocity);
      autoStates = SERVO_90;
    }
    else
    {
      autoStates = BACK_CONTROL;
    }
  }
}


void backControl(void)
{
  if (backMeasureFirst)
  {
    backMeasureFirst = false;
    backControlTime = millis();
  }

  if (readUltrasonic(BACK_SENSOR > 70))
  {
    motorBackward(autoVelocity);

    if ((millis() - backControlTime) > 1500)
    {
      backMeasureFirst = true;
      motorFastStop();
      autoStates = DISTANCE_10;
    }
  }
}


boolean checkObject(int directions)
{
  if (readUltrasonic(directions) <= COLLISION_LIMIT)
    return true;
  else
    return false;
}


int readUltrasonic(int sensor)
{
  unsigned long timeUltrasonicResponse = 0;
  float netDistance = 0.0;

  switch (sensor)
  {
    case BACK_SENSOR:
      pinMode(PING_PIN, OUTPUT);
      digitalWrite(PING_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(PING_PIN, HIGH);
      delayMicroseconds(5);
      digitalWrite(PING_PIN, LOW);
      pinMode(PING_PIN, INPUT);
      timeUltrasonicResponse = pulseIn(PING_PIN, HIGH, TIMEOUT_ULTRASONIC_MICROS);
      break;

    case FORE_SENSOR:
      digitalWrite(TRIG_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(TRIG_PIN, HIGH);
      delayMicroseconds(5);
      digitalWrite(TRIG_PIN, LOW);
      timeUltrasonicResponse = pulseIn(ECHO_PIN, HIGH, TIMEOUT_ULTRASONIC_MICROS);
      break;
  }

  netDistance = max(0, timeUltrasonicResponse - sensorOffset);
  return (int((netDistance / microsecondsPerCm / 2)));
}


float getTemperature(void)
{
  int sensorVoltage = analogRead(TEMP_PIN);
  float voltage = (sensorVoltage * 5.0) / 1023;
  return (((voltage * 1000) - 500.0) / 10.0);
}

