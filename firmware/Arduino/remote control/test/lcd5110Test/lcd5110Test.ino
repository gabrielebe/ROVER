#define RST 13
#define CE  12
#define DC  11
#define DIN 10
#define CLK  9

#include "font.h";

void LcdWriteString(char *characters)
{
  while (*characters) LcdWriteCharacter(*characters++);
}

void LcdWriteCharacter(char character)
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

void LcdXY(int x, int y)
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

void setup()
{
  pinMode(RST, OUTPUT);
  pinMode(CE, OUTPUT);
  pinMode(DC, OUTPUT);
  pinMode(DIN, OUTPUT);
  pinMode(CLK, OUTPUT);
  digitalWrite(RST, LOW);
  digitalWrite(RST, HIGH);

  LcdWriteCmd(0x21);  // LCD extended commands
  LcdWriteCmd(0xB9);  // set LCD Vop (contrast)
  LcdWriteCmd(0x04);  // set temp coefficent
  LcdWriteCmd(0x12);  // LCD bias mode 1:40
  LcdWriteCmd(0x20);  // LCD basic commands
  LcdWriteCmd(0x0C);  // LCD normal video

  for (int i = 0; i < 504; i++) LcdWriteData(0x00); // clear LCD

  LcdXY(20, 2);
  LcdWriteString("THE END");
}

void loop()
{
}

