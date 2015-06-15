/* 
 * File:   lcd.h
 * Author: Harpal
 *
 * Created on 27 settembre 2014, 21.20
 */


#ifndef LCD_H
#define	LCD_H
//***********************************//
//commands
#define lcdClear_CMD  0x01
#define lcdHome_CMD      0x02
#define lcdSetCGADR_CMD 0x40
#define lcdSetADR_CMD   0x80

//Pinout
#ifndef lcdD4
  #define lcdD4 LATDbits.LATD0
  #define lcdD4_DIR TRISDbits.TRISD0
#endif

#ifndef lcdD5
  #define lcdD5 LATDbits.LATD1
  #define lcdD5_DIR TRISDbits.TRISD1
#endif

#ifndef lcdD6
  #define lcdD6 LATDbits.LATD2
  #define lcdD6_DIR TRISDbits.TRISD2
#endif

#ifndef lcdD7
  #define lcdD7 LATDbits.LATD3
  #define lcdD7_DIR TRISDbits.TRISD3
#endif

#ifndef lcdRS
  #define lcdRS LATEbits.LATE0
  #define lcdRS_DIR TRISEbits.TRISE0
#endif

#ifndef lcdE
  #define lcdE LATEbits.LATE1
  #define lcdE_DIR TRISEbits.TRISE1
#endif

#define lcdLINES 2


//functions
void lcdInit(void);
void lcdWriteCmd(unsigned char n);

void writeDigit(unsigned char n);
void writeByte(unsigned char n);

//******************************//
void lcdWriteChar(char ch);
void lcdClear(void);
void lcdSetPos(unsigned char x, unsigned char y);
void lcdWriteStrC(const char* s);
void lcdWriteStr(char* s);

void lcdClean(void);

#endif	/* LCD_H */

