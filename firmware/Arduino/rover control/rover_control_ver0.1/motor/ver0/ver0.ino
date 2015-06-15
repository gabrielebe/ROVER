#include "motor.h"

void setup()
{
  Serial.begin(9600);
  MOTOR_init();
  MOTOR_forward(0);
}


void loop()
{
  for (int i = 0; i < 255; i += 50)
  {
    Serial.print("Velocità pwm avanti: ");
    Serial.println(i);
    MOTOR_forward(i);
    delay(5000);
    MOTOR_stop();
    Serial.println("Stop!");
    delay(2000);
  }

  for (int i = 0; i < 255; i += 50)
  {
    Serial.print("Velocità pwm dietro: ");
    Serial.println(i);
    MOTOR_backward(i);
    delay(5000);
    MOTOR_stop();
    Serial.println("Stop!");
    delay(2000);
  }

  for (int i = 0; i < 255; i += 50)
  {
    Serial.print("Velocità pwm destra: ");
    Serial.println(i);
    MOTOR_turnRight(i);
    delay(5000);
    MOTOR_stop();
    Serial.println("Stop!");
    delay(2000);
  }

  for (int i = 0; i < 255; i += 50)
  {
    Serial.print("Velocità pwm sinistra: ");
    Serial.println(i);
    MOTOR_turnLeft(i);
    delay(5000);
    MOTOR_stop();
    Serial.println("Stop!");
    delay(2000);
  }
}


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
      //MOTOR_turn(pwm, angle);
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
      //MOTOR_right(pwm, angle);
      break;

    case LEFT:
      //MOTOR_left(pwm, angle);
      break;
  }
}


/*void MOTOR_turn(int angle, int pwm)
{
    if((angle >= 0) && (angle <= 360))
    {
        if(angle == 90)
            MOTOR_forward(pwm);
        else if(angle == 270)
            MOTOR_backward(pwm);
        else if(angle > 90)
            MOTOR_right(pwm, angle);
        else if ((angle < 180) && (angle > 90))
        {
            angle = 90 - angle;
            MOTOR_left(pwm, angle);
        }
    }
}*/


/* This function is used to move forward the robot at a certain velocity */
void MOTOR_forward(int pwm)
{
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
  /* setting the inputs in order to stop */
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


/*void MOTOR_right(int pwm, int angle)
{
    if((angle >= 0) && (angle <= 90))
    {
        float previewHeading = 0;
        float heading = COMPASS_headingDetect();

        while((abs(heading - previewHeading)) > HEADING_LIMIT)
        {
            MOTOR_turnRight(pwm);
            delay(100);
            MOTOR_stop();
            delay(50);
            previewHeading = heading;
            heading = COMPASS_headingDetect();
        }
    }
}*/


/* This function is used to turn right the robot at a certain velocity */
void MOTOR_turnRight(int pwm)
{
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


/*void MOTOR_left(int pwm, int angle)
{
    if((angle >= 0) && (angle <= 90))
    {
        float previewHeading = 0;
        float heading = COMPASS_headingDetect();

        while((abs(heading - previewHeading)) > HEADING_LIMIT)
        {
            //turnLeft;  IMPLEMENTA FUNZIONI
            delay(100);
            //stop;       IMPLEMENTA FUNZIONI
            delay(50);
            previewHeading = heading;
            heading = COMPASS_headingDetect();
        }
    }
}*/


/* This function is used to turn left the robot at a certain velocity */
void MOTOR_turnLeft(int pwm)
{
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

