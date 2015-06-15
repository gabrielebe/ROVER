/* 
 * File:   lcd.h
 * Author: Harpal
 *
 * Created on 27 settembre 2014, 21.20
 */


#ifndef LCD_H
#define	LCD_H
//***********************************//
//defining all the commands f the lcd display
#define lcdClear_CMD  		0x01
#define lcdHome_CMD      	0x02
#define lcdSetCGADR_CMD 	0x40
#define lcdSetADR_CMD   	0x80

//defining how the lcd is connected to the micro
#ifndef lcdD4							//data pin D4
  #define lcdD4 LATDbits.LATD0
  #define lcdD4_DIR TRISDbits.TRISD0
#endif

#ifndef lcdD5							//data pin D5
  #define lcdD5 LATDbits.LATD1
  #define lcdD5_DIR TRISDbits.TRISD1
#endif

#ifndef lcdD6							//data pin D6
  #define lcdD6 LATDbits.LATD2
  #define lcdD6_DIR TRISDbits.TRISD2
#endif

#ifndef lcdD7							//data pin D7
  #define lcdD7 LATDbits.LATD3
  #define lcdD7_DIR TRISDbits.TRISD3
#endif

#ifndef lcdRS							//register select pin
  #define lcdRS LATEbits.LATE0
  #define lcdRS_DIR TRISEbits.TRISE0
#endif

#ifndef lcdE							//Clock enable pin
  #define lcdE LATEbits.LATE1
  #define lcdE_DIR TRISEbits.TRISE1
#endif

//defining how many rows does the lcd has
#define lcdLINES 2					

//functions prototypes
void lcdInit(void);						//used to initialize the display
void lcdWriteChar(char ch);				//used to write a single character
void lcdClear(void);					//used to clear the lcd
void lcdSetPos(unsigned char x, unsigned char y);	//used to set the position of the cursor
void lcdWriteStrC(const char* s);		//used to write a string on the lcd
void intro(void);						//used as boot introuction
void lcdClean(void);					//used to clear the screen with animation
//******************************//
/*private functions used only by this library*/
extern void lcdWriteCmd(unsigned char n);	//used to send a command to the driver
extern void writeByte(unsigned char n);		//used to send a single byte
extern void writeDigit(unsigned char n);	//used to send a digit on the display

#endif	/* LCD_H */
