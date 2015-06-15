#include <SoftwareSerial.h>       //Library used to communicate with software serial
#include "font.h"
//Send data
//*************************************************//
/**                Delay table                     **
/**  k = 4.21 @9600bps                             **
/**  k = 3.35 @19200bps                            **
/**  k = 3.1  @38400bps and @57600bps              **
/**  k = 2.8  @115200bps                           **
//*************************************************/
#define delayRF(numByte)  (4 + numByte)    //the maximum number of byte is 240

// code in common between uno and mega that specifies instructions
#define GO_STOP       '0'
#define GO_FORWARD   '1'
#define GO_BACKWARD   '2'
#define GO_LEFT       '3'
#define GO_RIGHT      '4'
#define VOID          '0'
#define TIMEOUT  10000    // Timeout for the pc detection

#define UP 	'0'
#define	DOWN 	'1'
#define LEFT 	'2'
#define RIGHT 	'3'

// pin definitions on the uno
// defining the lcd pins
#define RST 13
#define CE  12
#define DC  11
#define DIN 10
#define CLK  9
// defining the pushbutton pins
#define up    4       // pin used to read up button
#define down  5       // pin used to read down button
#define left  6       // pin used to read left button
#define right 3       // pin used to read right button
#define robot_mode  2       // pin used to change the mode
// defining the joystick pins
#define joyX1 A2      // vertical of first joystick
#define joyY1 A3      // horizontal of first joystick
#define joyX2 A0      // vertical of second joystick
#define joyY2 A1      // horizontal of second joystick
SoftwareSerial mySerial(8, 7);  //Rx - Tx
// functions used to control the lcd
void LcdWriteString(char *characters);
void LcdWriteChar(char character);
void LcdWriteData(byte dat);
void LcdSetPos(int x, int y);
void LcdWriteCmd(byte cmd);
void LcdClear(void);
void LcdSetMode(char mode);
void LcdClearLine(char line);
void printStrToLcd( const char i);
// functions used to communicate with robot
void sendCmd(char instruction, char firstParameter, char secondParameter);
// Variables used to store the analog values.
void menu_update(int action);

int joyX1_value = 0,
    joyY1_value = 0,
    joyX2_value = 0,
    joyY2_value = 0;


char index,           // numbers of byte received
     command = -1,    // command received
     isTherePc = 1;   // Flag that indicates if the pc is connected

unsigned long waitTime = 0;
boolean auto_manual = 1;
boolean armEnabled = 0;
boolean lcdMap[5][5] = {{1, 1, 1, 1, 1},
  {0, 1, 1, 1, 1},
  {0, 1, 0, 0, 1},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0}
};

void setup() {
  Serial.begin(115200);      // Used to connect with
  mySerial.begin(38400);     // This is the maximum velocity with S2 closed

  pinMode(up, INPUT);
  pinMode(down, INPUT);
  pinMode(left, INPUT);
  pinMode(right, INPUT);
  pinMode(robot_mode, INPUT);
  LcdInit();                  // Initialize the lcd
  //ConnectToPc();
  LcdClear();
  printStrToLcd(8);       // Printing on screen the start status
  delay(1000);
}

void loop()
{
  byte dataPacket[5] = {0};
  receiveData();

  if (digitalRead(up) == LOW)
  {
    menu_update(UP);
    delay(200);
  }
  if (digitalRead(down) == LOW)
  {
    menu_update(DOWN);
    delay(200);
  }
  if (digitalRead(left) == LOW)
  {
    menu_update(LEFT);
    delay(200);
  }
  if (digitalRead(right) == LOW)
  {
    menu_update(RIGHT);
    delay(200);
  }
  if ( armEnabled == 1)
  {
    joyX2_value = (byte)map(analogRead(joyX2), 0, 1023, '0', '9');
    joyY2_value = (byte)map(analogRead(joyY2), 0, 1023, '0', '9');
    //implement all the arms movement functions

  }
  if (auto_manual == 1)
  {
    joyX1_value = (byte)map(analogRead(joyX1), 0, 1023, 0, 511);
    joyY1_value = (byte)map(analogRead(joyY1), 0, 1023, 0, 511);
    if (joyX1_value < 240 || joyX1_value > 260 || joyY1_value < 240 || joyY1_value > 260)
    {
      if (joyX1_value > 260)
      {
        joyX1_value = map(joyX1_value, 130, 255, 0, 255);
        moveCommand(GO_BACKWARD, joyX1_value);
      }
    }
    else if (joyX1_value < '3')
    {
      if (((joyX1_value = (byte)map(analogRead(joyX1), 0, 1023, 0, 255)) < 120))
      {
        joyX1_value = map(joyX1_value, 120, 0, 0, 255);
        moveCommand(GO_FORWARD, joyX1_value);
      }
    }
    if (joyY1_value > '5')
    {
      if (((joyY1_value = (byte)map(analogRead(joyY1), 0, 1023, 0, 255)) > 130))
      {
        joyY1_value = map(joyY1_value, 130, 255, 0, 255);
        moveCommand(GO_LEFT, joyY1_value);
      }
    }
    else if (joyY1_value < '3')
    {
      if (((joyY1_value = (byte)map(analogRead(joyY1), 0, 1023, 0, 255)) < 120))
      {
        joyY1_value = map(joyY1_value, 120, 0, 0, 255);
        moveCommand(GO_RIGHT, joyY1_value);
      }
    }
  } else
  {
    moveCommand(GO_STOP, 255);
  }
}


void togleMode(void)
{
  static byte last_mode;
  LcdClearLine(1);
  LcdSetPos(0, 1);
  if (last_mode == 'a')
  {
    printStrToLcd(1);
    mySerial.write('m');
    last_mode = 'm';
  }
  else if (last_mode == 'm')
  {
    printStrToLcd(0);
    mySerial.write('a');
    last_mode = 'a';
  }
  while (digitalRead(robot_mode) == 0);
  delay(10);
}


void moveCommand(char command, byte velocity)
{
  byte dataPacket[2] = {0};
  dataPacket[0] = command;
  dataPacket[1] = velocity;
  mySerial.write(dataPacket, 2);
  delay(delayRF(2));

}
void menu_update(int action) {
  static byte m = 0, submenu = 0;
  switch (action) {                    // the action depends on the button that has changed
    case RIGHT: if (m + 1 > 4) m = 4;
      else if (lcdMap[submenu][m + 1] == 1)
      {
        m++;
        submenu = 0;
      }
      break;
    case LEFT: if (m - 1 < 0) m = 0;
      else if (lcdMap[submenu][m - 1] == 1)
      {
        m--;
        submenu = 0;
      }
      break;
    case UP: if (submenu - 1 < 0) submenu = 0;
      else	if (lcdMap[submenu - 1][m] == 1)
      {
        submenu--;
      }
      break;
    case DOWN: if (submenu + 1 > 4) submenu = 4;
      else if (lcdMap[submenu + 1][m] == 1)
      {
        submenu++;
      }
      break;
  }//switch
  if (m < 0) m = 0;
  if (m > 3) m = 3;
  if (submenu < 0) submenu = 0;
  if (submenu > 3) submenu = 3;
  display_option(m, submenu);
}//menu_update

void display_option(byte column, byte row)
{
  switch (column)
  {
    case 0:
      LcdClear();
      LcdWriteString("FIRST SCREEN");
      switch (row)
      {
        case 0:
          LcdSetPos(0, 1);
          LcdWriteString("OP1");

          break;
        case 1:
          LcdSetPos(0, 1);
          LcdWriteString("OP2");

          break;
        case 2:
          LcdSetPos(0, 1);
          LcdWriteString("OP3");

          break;
        case 3:
          LcdSetPos(0, 1);
          LcdWriteString("OP4");

          break;
      }
      break;
    case 1:
      LcdClear();
      LcdWriteString("SECOND SCREEN");

      switch (row)
      {
        case 0:
          LcdSetPos(0, 1);
          LcdWriteString("OP1");

          break;
        case 1:
          LcdSetPos(0, 1);
          LcdWriteString("OP2");

          break;
        case 2:
          LcdSetPos(0, 1);
          LcdWriteString("OP3");

          break;
        case 3:
          LcdSetPos(0, 1);
          LcdWriteString("OP4");

          break;
      }
      break;
    case 2:
      LcdClear();
      LcdWriteString("THIRD SCREEN");

      switch (row)
      {
        case 0:
          LcdSetPos(0, 1);
          LcdWriteString("OP1");

          break;
        case 1:
          LcdSetPos(0, 1);
          LcdWriteString("OP2");

          break;
        case 2:
          LcdSetPos(0, 1);
          LcdWriteString("OP3");

          break;
        case 3:
          LcdSetPos(0, 1);
          LcdWriteString("OP4");

          break;
      }
      break;
    case 3:
      LcdClear();
      LcdWriteString("LAST SCREEN");

      switch (row)
      {
        case 0:
          LcdSetPos(0, 1);
          LcdWriteString("OP1");

          break;
        case 1:
          LcdSetPos(0, 1);
          LcdWriteString("OP2");

          break;
        case 2:
          LcdSetPos(0, 1);
          LcdWriteString("OP3");

          break;
        case 3:
          LcdSetPos(0, 1);
          LcdWriteString("OP4");

          break;
      }
      break;

  }

}
void LcdClearLine(char line)
{
  LcdSetPos(0, line);
  for (int i = 0; i < 84; i++)
  {
    LcdWriteData(0x00);
  }
}

void receiveData()
{
  char dataArray[10] = {    // Buffer used to store communication data
    0
  };
  if (mySerial.available() > 0)
  {
    while (mySerial.available() && index < 10) {
      dataArray[index++] = mySerial.read();
      delayMicroseconds(100);
    }
    if (index == 7)
    {
      // this is used to receive the accelerometer data that is echoed
      // to the main serial port in order to send it to the pc and read through MATLAB

      Serial.println(dataArray[0] + (dataArray[1] << 8));     // print the x axis
      Serial.println(dataArray[2] + (dataArray[3] << 8));     // print the y axis
      Serial.println(dataArray[4] + (dataArray[5] << 8));     // print the z axis
      index = 0;
    }
  } else {
    index = 0;
  }
}
void ConnectToPc(void)
{
  // Sending 'a' charachter to pc
  Serial.println('a');

  char a = 'b';
  waitTime = millis() + TIMEOUT;
  printStrToLcd(0);     // Printing on screen the status
  while (a != 'a' && isTherePc)
  {
    /* Wait for a specific character from the PC */
    a = Serial.read();
    // wait only for 5 second if it exceeds the time it means there isn't any pc connected
    isTherePc = millis() > TIMEOUT ? 0 : 1;
    if (isTherePc == 0)
    {
      printStrToLcd(1);
      delay(1000);
      LcdClear();
    }
  }
}

void printStrToLcd( const char i)
{
  char string[50] = {0};
  strcpy_P(string, (char*)pgm_read_word(&(string_table[i])));
  LcdWriteString(string);
}

void LcdWriteString(char *characters)
{
  while (*characters) LcdWriteChar(*characters++);
}

void LcdWriteChar(char character)
{
  for (int i = 0; i < 5; i++) LcdWriteData(pgm_read_byte(&(ASCII[character - 0x20][i])));
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
  LcdWriteCmd(0x12);  // LCD bias mode 1:40
  LcdWriteCmd(0x20);  // LCD basic commands
  LcdSetMode(3);      // LCD normal mode

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

