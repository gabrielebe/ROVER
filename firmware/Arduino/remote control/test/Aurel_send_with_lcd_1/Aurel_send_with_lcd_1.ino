#include <avr/pgmspace.h>
//Send data

//*************************************************//
/**                Delay table                     **
/**  k = 4.21 @9600bps                             **
/**  k = 3.35 @19200bps                            **
/**  k = 3.1  @38400bps and @57600bps              **
/**  k = 2.8  @115200bps                           **
 //*************************************************/
#define delayRF(numByte)  (4 + numByte)    //the maximum number of byte is 240

#include <SoftwareSerial.h>       //Library used to communicate with software serial
#include "font.h"


// code in common between uno and mega that specifies instructions
#define GO_STOP       '0'
#define GO_FOREWARD   '1'
#define GO_BACKWARD   '2'
#define GO_LEFT       '3'
#define GO_RIGHT      '4'
#define VOID          '0'
#define TIMEOUT  10000    // Timeout for the pc detection

// pin definitions on the uno
// defining the lcd pins
#define RST 13
#define CE  12
#define DC  11
#define DIN 10
#define CLK  9
// defining the pushbutton pins
#define up    5       // pin used to read up button
#define down  4       // pin used to read down button
#define left  3       // pin used to read left button
#define right 6       // pin used to read right button
// defining the joystick pins
#define joyX1 A0      // vertical of first joystick
#define joyY1 A1      // horizontal of first joystick
#define joyX2 A2      // vertical of second joystick
#define joyY2 A3      // horizontal of second joystick

//*********************************************************//
//************** String messages **************************//
//const char pcAttempt[] PROGMEM  = {"Attempting to connect to PC:"};

SoftwareSerial mySerial(8, 7); // RX, TX

// functions used to control the lcd
void LcdWriteString(char *characters);
void LcdWriteChar(char character);
void LcdWriteData(byte dat);
void LcdSetPos(int x, int y);
void LcdWriteCmd(byte cmd);
void LcdClear(void);
void LcdSetMode(char mode);
// functions used to communicate with robot
void sendCmd(char instruction, char firstParameter, char secondParameter);

char pot = A0,                    //pin used to read analog value
     analogValue = 0;             //variable used to store an analog value

// Variables used to store the analog values.
int joyX1_value = 0,
    joyY1_value = 0,
    joyX2_value = 0,
    joyY2_value = 0;

char dataArray[10] = {    // Buffer used to store communication data
  0
};

char index,           // numbers of byte received
     command = -1,    // command received
     isTherePc = 1;   // Flag that indicates if the pc is connected

unsigned long initialTime = 0;

const char string_0[] PROGMEM = "Attempting pc connection";   // "String 0" etc are strings to store - change to suit.
const char string_1[] PROGMEM = "Time Out occured during PC connection";
const char string_2[] PROGMEM = "Forward by:";
const char string_3[] PROGMEM = "Backward by:";
const char string_4[] PROGMEM = "Turn left by:";
const char string_5[] PROGMEM = "Turn right by:";


// Then set up a table to refer to your strings.

const char* const string_table[] PROGMEM = {string_0, string_1, string_2, string_3, string_4, string_5};

void printStrToLcd( const char i)
{
  char string[50] = {0};
  strcpy_P(string, (char*)pgm_read_word(&(string_table[i])));
  LcdWriteString(string);

}
void setup()
{
  Serial.begin(115200);      //used to debug thee whole process
  mySerial.begin(9600);     //this is the maximum velocity with S2 closed
  pinMode(pot, INPUT);       //initializing pins as desired
  pinMode(up, INPUT);
  pinMode(down, INPUT);
  pinMode(left, INPUT);
  pinMode(right, INPUT);

  //LcdInit();
  Serial.println('a');

  char a = 'b';
  initialTime = millis();
  //Serial.println("Atempting to connect with PC:");
  //printStrToLcd(0);
  while (a != 'a' && isTherePc)
  {
    /* Wait for a specific character from the PC */
    a = Serial.read();

    // wait only for 5 second if it exceeds the time it means there isn't any pc connected
    isTherePc = (initialTime + millis()) > TIMEOUT ? 0 : 1;
    if (isTherePc == 0)  printStrToLcd(1);
  }
  Serial.println("Start");
}


void loop()
{
  /*if (mySerial.available() > 0)
  {
    while (mySerial.available()) {
      dataArray[index++] = mySerial.read();
      delayMicroseconds(100);
    }
    if (index == 7)
    {
      // this is used to receive the accelerometer data that is echoed
      // to the main serial port in order to send it to the pc and read through MATLAB
      Serial.print("the x value is:");
      Serial.println(dataArray[0] + (dataArray[1] << 8));     // print the x axis
      Serial.print("the y value is:");
      Serial.println(dataArray[2] + (dataArray[3] << 8));     // print the y axis
      Serial.print("the z value is:");
      Serial.println(dataArray[4] + (dataArray[5] << 8));     // print the z axis

      index = 0;
    }
  } else {
    index = 0;
  }*/
  /*******************************************************/
  /*******************************************************/
  /*******************************************************/
  while (digitalRead(up) == LOW) mySerial.write("c");
  delay(100);
  /*while (digitalRead(up) == LOW)
  {
    analogValue = map(0, 1023, 0, 256, analogRead(pot));
    sendCmd(GO_FOREWARD, analogValue, VOID);
    Serial.print("Going Foreward by: ");
    Serial.println(analogValue);
    //LcdClear();
    //printStrToLcd(2);
    //LcdSetPos(0, 1);
    //LcdWriteChar(analogValue);  //need to convert in number
    //delay(100);

  }
  while (digitalRead(down) == LOW)
  {
    analogValue = map(0, 1023, 0, 256, analogRead(pot));
    sendCmd(GO_BACKWARD, analogValue, VOID);
    Serial.print("Going Backward by: ");
    Serial.println(analogValue);
    //LcdClear();
    //printStrToLcd(3);
    //LcdSetPos(0, 1);
    //LcdWriteChar(analogValue);  //need to convert in number
    //delay(100);
  }
  while (digitalRead(left) == LOW)
  {
    analogValue = map(0, 1023, 0, 256, analogRead(pot));
    sendCmd(GO_LEFT, analogValue, VOID);
    Serial.print("turning left by: ");
    Serial.println(analogValue);
    //LcdClear();
    //printStrToLcd(4);
    //LcdSetPos(0, 1);
    //LcdWriteChar(analogValue);  //need to convert in number
    //delay(100);
  }
  while (digitalRead(right) == LOW)
  {
    analogValue = map(0, 1023, 0, 256, analogRead(pot));
    sendCmd(GO_RIGHT, analogValue, VOID);
    Serial.print("turning right by: ");
    Serial.println(analogValue);
    //LcdClear();
    //printStrToLcd(5);
    //LcdSetPos(0, 1);
    //LcdWriteChar(analogValue);  //need to convert in number
    //delay(100);
  }
  //sendCmd(GO_STOP, VOID, VOID);
  //Serial.print("Stop");
  //LcdClear();
  //LcdWriteString("Stop");
  //delay(100);*/
}


void sendCmd(char instruction, char firstParameter, char secondParameter) {
  char  dataArray[4] = {0};             //buffer use to hold the data package
  dataArray[0] = instruction;
  dataArray[1] = firstParameter;
  dataArray[2] = secondParameter;
  mySerial.write(dataArray, 4);
  delay(delayRF(4));
}
/***************************************************************/

void LcdWriteString(char *characters)
{
  while (*characters) LcdWriteChar(*characters++);
}

void LcdWriteChar(char character)
{
  for (int i = 0; i < 5; i++) LcdWriteData(ASCII[character - 0x20][i]);
  LcdWriteData(0x00);
}

void LcdWriteData(byte dat)
{
  digitalWrite(DC, HIGH); //DC pin is low for commands
  digitalWrite(CE, LOW);
  shiftOut(DIN, CLK, MSBFIRST, dat); //transmit serial data
  digitalWrite(CE, HIGH);
}

void LcdSetPos(int x, int y)
{
  LcdWriteCmd(0x80 | x);  // Column.
  LcdWriteCmd(0x40 | y);  // Row.
}

void LcdWriteCmd(byte cmd)
{
  digitalWrite(DC, LOW); //DC pin is low for commands
  digitalWrite(CE, LOW);
  shiftOut(DIN, CLK, MSBFIRST, cmd); //transmit serial data
  digitalWrite(CE, HIGH);
}
void LcdInit(void)
{
  pinMode(RST, OUTPUT);
  pinMode(CE, OUTPUT);
  pinMode(DC, OUTPUT);
  pinMode(DIN, OUTPUT);
  pinMode(CLK, OUTPUT);
  digitalWrite(RST, LOW);
  digitalWrite(RST, HIGH);

  LcdWriteCmd(0x21);  // LCD extended commands
  LcdWriteCmd(0xBF);  // set LCD Vop (contrast)
  LcdWriteCmd(0x04);  // set temp coefficent
  LcdWriteCmd(0x11);  // LCD bias mode 1:40
  LcdWriteCmd(0x20);  // LCD basic commands
  LcdSetMode(2);      // LCD normal mode

  for (int i = 0; i < 528; i++) LcdWriteData(Hello[i]);
  delay(2000);
  LcdClear();
}

void LcdClear()
{
  for (int i = 0; i < 504; i++) LcdWriteData(0x00); // clear LCD
  LcdSetPos(0, 0);

}

void LcdSetMode(char mode)
{
  switch (mode)
  {
    case 0:
      LcdWriteCmd(0b00001000);  // LCD display blank
      break;
    case 1:
      LcdWriteCmd(0b00001001);  // LCD turn All segments on
      break;
    case 2:
      LcdWriteCmd(0b00001100);  // LCD normal video
      break;
    case 3:
      LcdWriteCmd(0b00001101);  // LCD inverse video
      break;

  }

}

