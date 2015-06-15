/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://arduino.cc/en/Tutorial/Knob
*/

#include <Servo.h>

Servo myservo1;  // servo direzione sinistra destra limiti 20 e 130
Servo myservo2;  // servo picker limiti 135 e 180
Servo myservo3;  // servo direzione su e giù limiti 0 a 180
Servo myservo4;  // servo torretta

void setup()
{
  //myservo1.attach(38);  // attaches the servo on pin 9 to the servo object
  //myservo2.attach(40);
  //myservo3.attach(42);
  myservo4.attach(44);
}

void loop()
{
  //myservo1.write(130);
  //myservo2.write(180);
  //myservo3.write(180);
  myservo4.write(90);

  /* for (int i = 20; i < 160; i++)
   {
     myservo1.write(i);                  // sets the servo position according to the scaled value
     delay(10);
   }
   for (int i = 0; i < 180; i++)
   {
     myservo2.write(i);                  // sets the servo position according to the scaled value
     delay(0);
   }

   for (int i = 20; i < 160; i++)
   {
     myservo3.write(i);                  // sets the servo position according to the scaled value
     delay(10);
   }
   for (int i = 20; i < 160; i++)
   {
     myservo4.write(i);                  // sets the servo position according to the scaled value
     delay(10);
   }
  */
  //delay(10000);
}

