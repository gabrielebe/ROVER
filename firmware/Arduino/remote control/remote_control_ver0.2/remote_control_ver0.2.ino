/******************************************************
* The Author: Singh Harpal - Bè Gabriele - Rosa Alice *
* Date: xx/03/2015                                    *
* Version: V1.0                                       *
* Project name: Johnny Remote Control                 *
*******************************************************/

//Inclusion of the needed libraries
#include <SoftwareSerial.h>       //Used to cmmunicate with the                                
#include "motor.h"

//*************************************************//
/**                Delay table                     **
/**  k = 4.21 @9600bps                             **
/**  k = 3.35 @19200bps                            **
/**  k = 3.1  @38400bps and @57600bps              **
/**  k = 2.8  @115200bps                           **
 //*************************************************/
#define delayRF(numByte)  (4 + numByte)    //the maximum number of byte is 240

#define  X_PIN A0
#define  Y_PIN A1
#define  Z_PIN A2
SoftwareSerial mySerial(10, 11); //RX, TX
char index;
unsigned char dataArray[10] = {
  0
};

int xValue, yValue, zValue;
void send6char(char, char, char, char, char, char);

void setup()
{
  Serial.begin(115200);
  mySerial.begin(38400);
  //while (!mySerial.available());
  index = 0;
  MOTOR_init();

  Serial.println("Start");
}

void loop()
{
  if (mySerial.available() > 0) {
    while (mySerial.available() > 0) {
      dataArray[index++] = mySerial.read();
      //delayMicroseconds(100);      //delay needed but don't know why
    }
    if (index == 0 || index == 1)
    {
      Serial.println("NoCommand");
      index = 0;
    }
    else if (index == 2)
    {
      // in case which receive only one character
      // the data array 0 is the command char received
      Serial.println("one Command");
      if (dataArray[0] == 'A')
      {
        xValue = analogRead(X_PIN);
        Serial.print("valore x:");
        Serial.println(xValue);
        yValue = analogRead(Y_PIN);
        zValue = analogRead(Z_PIN);
        send6char(xValue & 0xff, (xValue >> 8) & 0xff,
                  yValue & 0xff, (yValue >> 8) & 0xff, zValue & 0xff, (zValue >> 8) & 0xff);


      }
      index = 0;
    }
    else if (index == 3)
    {
      //Serial.println("Receiving 3 char");
      // in case which receive only two characters
      // the data array 0 is the first received character
      // the data array 1 is the second received chracter

      //The data array 0 represent the instruction
      //The data array 1 represent the first parameter
      //The data array 2 represent the second parameter
      switch (dataArray[0])
      {
        case '0':
          MOTOR_stop();
          Serial.println("Stopping the motor");
          break;
        case '1':
          MOTOR_forward(dataArray[1]);
          Serial.print("Moving the motor forward by: ");
          Serial.write(dataArray[1]);
          Serial.println("  ");
          break;
        case '2':
          MOTOR_backward(dataArray[1]);
          Serial.print("Moving the motor backward by: ");
          Serial.write(dataArray[1]);
          Serial.println("  ");
          break;
        case '3':
          MOTOR_turnRight(dataArray[1]);
          Serial.print("turning the motor right with: ");
          Serial.write(dataArray[1]);
          //Serial.print("of degree");
          //Serial.write(secondParameter);
          Serial.println(" ");
          break;
        case '4':
          MOTOR_turnLeft(dataArray[1]);
          Serial.print("turning the motor left with: ");
          Serial.write(dataArray[1]);
          //Serial.print("of degree");
          //Serial.write(secondParameter);
          Serial.println(" ");
          break;
        default:
          MOTOR_stop();
          Serial.println("Incorrect command");
      }
      delayRF(5);
      index = 0;
    }
    else if (index == 4) {
      Serial.println("Receiving 5 char");
      index = 0;
    }
    else if (index = 6)
    {
      Serial.println("Receiving 6 char");

      index = 0;
    }
    else
    {
      Serial.println("Receiving more than 6 char");
      // add here new if else statements if needed when the received characters
      // are more than those above declared



      index = 0;
    }
  }
}


void send6char(char val0, char val1, char val2, char val3, char val4, char val5)
{
  Serial.println("Printing back");
  char finalArray[7] = {val0, val1, val2, val3, val4, val5, '\n'};
  Serial.println(finalArray);
  mySerial.write(finalArray, 7);
  delayRF(7);
}


/*****************************************************************/
/*****************************************************************/
/*****************************************************************/
/*****************************************************************/
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

