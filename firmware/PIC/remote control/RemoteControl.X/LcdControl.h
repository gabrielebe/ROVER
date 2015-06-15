/* 
 * File:   LcdControl.h
 * Author: Harpal
 *
 * Created on 8 marzo 2015, 14.59
 */

#ifndef LCDCONTROL_H
#define	LCDCONTROL_H

//commands
#define lcdClear_CMD  0x01
#define lcdHome_CMD      0x02
#define lcdSetCGADR_CMD 0x40
#define lcdSetADR_CMD   0x80

//Pinout
#ifndef lcdD4
  #define lcdD4 LATEbits.LATE2
  #define lcdD4_DIR TRISEbits.TRISE2
#endif

#ifndef lcdD5
  #define lcdD5 LATCbits.LATC0
  #define lcdD5_DIR TRISCbits.TRISC0
#endif

#ifndef lcdD6
  #define lcdD6 LATCbits.LATC1
  #define lcdD6_DIR TRISCbits.TRISC1
#endif

#ifndef lcdD7
  #define lcdD7 LATCbits.LATC2
  #define lcdD7_DIR TRISCbits.TRISC2
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
void intro(void);
void lcdClean(void);

#endif	/* LCDCONTROL_H */

