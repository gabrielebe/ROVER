//Send data

//*************************************************//
/**                Delay table                     **
/**  k = 4.21 @9600bps                             **
/**  k = 3.35 @19200bps                            **
/**  k = 3.1  @38400bps and @57600bps              **
/**  k = 2.8  @115200bps                           **
 //*************************************************/
#define delayRF(numByte)  (4 + numByte)    //the maximum number of byte is 240

#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11); // RX, TX
int analogX = A0;
int analogY = A1;
int analogZ = A2;

char up = 13,
     down = 12,
     left = 11,
     right = 10,
     pot = A0,
     analogValue = 0,
     buffer[6] = {0};

void setup()
{
  Serial.begin(115200);      //used to debug
  mySerial.begin(38400);    //this is the maximum velocity with S2 closed
  pinMode(pot, INPUT);
  pinMode(up, INPUT);
  pinMode(down, INPUT);
  pinMode(left, INPUT);
  pinMode(right, INPUT);
}


void loop()
{
  /*
  x = 'A';//analogRead(analogX);
   y = 'B';//analogRead(analogY);
   z = 'C';//analogRead(analogZ);
   Serial.println("The data is: ");
   Serial.print("X is: ");
   Serial.println(x);
   Serial.print("Y is: ");
   Serial.println(y);
   Serial.print("Z is: ");
   Serial.println(z);
   Serial.println("----------------------\n\n");

  //if(!(mySerial.available() >0))
  /*{
   mySerial.write("0254*");
   delay(delayRF(5));
   //mySerial.write(y);
   //delay(delayRF(1));
   //mySerial.write(z);
   //delay(delayRF(1));
   }
   */
  analogValue = map(0, 1023, 0, 255, analogRead(pot));
  buffer[0] = 'a';
  buffer[1] = 'b';
  buffer[2] = 'c';
  buffer[3] = 'd';
  buffer[4] = analogValue;
  //buffer[5] = ;
  Serial.write(buffer);
  Serial.println("********************************");
  delay(500);
  /*
  while(digitalRead(up) == HIGH)
  {
   analogValue = (char *)map(0,1023,0,256,analogRead(pot));
   mySerial.write(sprintf("1",analogValue,"0","0"));
   delay(delayRF(5));
  }
  while(digitalRead(down) == HIGH)
  {
   analogValue = (char *)map(0,1023,0,256,analogRead(pot));
   mySerial.write(sprintf("2",analogValue,"0","0"));
   delay(delayRF(5));
  }
  while(digitalRead(left) == HIGH)
  {
   analogValue = (char *)map(0,1023,0,256,analogRead(pot));
   mySerial.write(sprintf("3",analogValue,"0","0"));
   delay(delayRF(5));
  }
  while(digitalRead(right) == HIGH)
  {
   analogValue = (char *)map(0,1023,0,256,analogRead(pot));
   mySerial.write(sprintf("4",analogValue,"0","0"));
   delay(delayRF(5));
  }
   mySerial.write(sprintf("0","0","0","0"));
   delay(delayRF(5));*/

}

