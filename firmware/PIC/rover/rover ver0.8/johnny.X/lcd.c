#include <stdio.h>              //Standard c library for IO
#include <stdlib.h>             //Standard c library
#include <xc.h>                 //XC compiler library
#include <pic18f47j53.h>        //pic microcontroller library

#include "config.h"
#include "lcd.h"                //library used for the lcd control


void lcdInit(void)
{
  // Initialization of the microcontroller pins
  lcdD4_DIR = 0;
  lcdD5_DIR = 0;
  lcdD6_DIR = 0;
  lcdD7_DIR = 0;

  lcdRS_DIR = 0;
  lcdRS = 0;			// initially pulling low the pin for security

  lcdE_DIR = 0;
  lcdE = 0;				// initially pulling low the pin for security

  // Delay needed to make sure that Vcc has stabilized
  __delay_ms(15);

  // execution of the sequence of resets as suggested by the datasheet
  // and to initialize the lcd correctly
  lcdRS = 0;			// pull down the RS pin
  writeDigit(0x03);
  for(int i = 0; i < 100; i++) __delay_us(41);  //the wait time has to be at least4,1mS

  writeDigit(0x03);
  for(int i = 0; i < 10; i++) __delay_us(12);	//the wait time has to be at least 100uS

  writeDigit(0x03);
  for(int i = 0; i < 10; i++) __delay_us(12);

  writeDigit(0x02);
  for(int i = 0; i < 10; i++) __delay_us(12);
  // finish the reset sequence

  // send the command that tell how many rows are there
  // and than send the right instruction
  #if (1 == lcdLINES)
    lcdWriteCmd(0x20);
  #elif (2 == lcdLINES)
    lcdWriteCmd(0x28);
  #else
	//#error "the number of rows aren't correct or defined"
  #endif

  // clear the display
  lcdWriteCmd(0x01);

  // it sets the mode in which the direction register works
  // it is set to increment
  lcdWriteCmd(0x06);

  // turn on the lcd with cursor visible without the line
  lcdWriteCmd(0x0C);

  //intro();		//display the intro
}

void writeDigit(unsigned char n)
{
  // the pin must stay high for at least 10 nS
  lcdE = 1;
  __delay_us(1);
  // put the data on output lines
  if (n & 0x01) lcdD4 = 1; else lcdD4 = 0;
  if (n & 0x02) lcdD5 = 1; else lcdD5 = 0;
  if (n & 0x04) lcdD6 = 1; else lcdD6 = 0;
  if (n & 0x08) lcdD7 = 1; else lcdD7 = 0;
  __delay_us(2);
  // the hold time must be more than 80ns
  lcdE = 0;
}

void writeByte(unsigned char n)
{
  // It sends the higher bits first
  writeDigit(n >> 4);
  // then it sends the lower bits
  writeDigit(n & 0x0F);
}

void lcdWriteCmd(unsigned char n)
{
  // To send a command the RS pin should go low
  lcdRS = 0;
  // Sending the command
  writeByte(n);
  // Waiting for the command to be executed by the lcd driver
  // And this wait time can be as long as 1600 us and as low as 43 us
  // depending on the command itself
  if (!(n & 0xFC)) for(int i = 0; i < 100; i++) __delay_us(16);
  else __delay_us(43);
}

void lcdWriteChar(unsigned char n)
{
  // To send data the RS should be High
  lcdRS = 1;
  // And then send data to the lcd
  writeByte(n);
  // There is need to wait at least 43uS for execution
  // of the instruction inside the lcd driver
  __delay_us(43);
}



void lcdClear(void)
{
  lcdWriteCmd(lcdClear_CMD);	//clear the lcd
}

void lcdSetPos(unsigned char x, unsigned char y)
{
  unsigned char p;				//variable used to calculate the instruction

  switch(y)						//select the row on the lcd
  {
    case 0: p = x; break;
    case 1: p = 64 + x; break;
    case 2: p = 20 + x; break;
    case 3: p = 84 + x; break;
  }

  p |= lcdSetADR_CMD;			//ad the instruction to the position calculated previously
  lcdWriteCmd(p);				//send the command
}

void lcdWriteStrC(const char* s)
{
  while(*s) lcdWriteChar(*s++);	//Continue writing the single character until the pointer is false
}
//this function is used to clear the lcd with animation
void lcdClean()
{
    int i;
    for(i = 0; i < 16; i++)
        {
            lcdSetPos(i,0);
            lcdWriteChar('-');
            __delay_ms(20);

        }

        for(i = 0; i < 16; i++)
        {
            lcdSetPos(i,0);
            lcdWriteChar(' ');
            lcdSetPos(16-i,1);
            lcdWriteChar('-');
            __delay_ms(20);

        }

        for(i = 0; i < 16; i++)
        {
            lcdSetPos(i,1);
            lcdWriteChar(' ');
            __delay_ms(20);

        }

    lcdClear();
}

void intro(void)
{
    // introduction displayed on the boot
    int i;
    lcdSetPos(0,0);
    lcdWriteStrC("Johnny V0.000001");
    lcdSetPos(0,1);
    lcdWriteStrC("Singh Be");

    for(i = 0; i < 100; i++) __delay_ms(50);
    lcdClean();

}
