/* 
 * File:   main.c
 * Author: Harpal
 *
 * Created on 30 settembre 2014, 14.30
 */

#include <stdio.h>
#include <stdlib.h>
#include <pic18f47j53.h>

// PIC18F47J53 Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>
#define _XTAL_FREQ 12000000
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1L
#pragma config WDTEN = OFF      // Watchdog Timer (Disabled - Controlled by SWDTEN bit)
#pragma config PLLDIV = 1       // PLL Prescaler Selection (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CFGPLLEN = OFF   // PLL Enable Configuration Bit (PLL Disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset (Enabled)
#pragma config XINST = OFF      // Extended Instruction Set (Disabled)

// CONFIG1H
#pragma config CPUDIV = OSC1    // CPU System Clock Postscaler (No CPU system clock divide)
#pragma config CP0 = OFF        // Code Protect (Program memory is not code-protected)

// CONFIG2L
#pragma config OSC = HSPLL      // Oscillator (HS+PLL, USB-HS+PLL)
#pragma config SOSCSEL = HIGH   // T1OSC/SOSC Power Selection Bits (High Power T1OSC/SOSC circuit selected)
#pragma config CLKOEC = OFF     // EC Clock Out Enable Bit  (CLKO output disabled on the RA6 pin)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor (Enabled)
#pragma config IESO = ON        // Internal External Oscillator Switch Over Mode (Enabled)

// CONFIG2H
#pragma config WDTPS = 32768    // Watchdog Postscaler (1:32768)

// CONFIG3L
#pragma config DSWDTOSC = INTOSCREF// DSWDT Clock Select (DSWDT uses INTRC)
#pragma config RTCOSC = T1OSCREF// RTCC Clock Select (RTCC uses T1OSC/T1CKI)
#pragma config DSBOREN = ON     // Deep Sleep BOR (Enabled)
#pragma config DSWDTEN = OFF    // Deep Sleep Watchdog Timer (Disabled)
#pragma config DSWDTPS = G2     // Deep Sleep Watchdog Postscaler (1:2,147,483,648 (25.7 days))

// CONFIG3H
#pragma config IOL1WAY = OFF    // IOLOCK One-Way Set Enable bit (The IOLOCK bit (PPSCON<0>) can be set and cleared as needed)
#pragma config ADCSEL = BIT10   // ADC 10 or 12 Bit Select (10 - Bit ADC Enabled)
#pragma config MSSP7B_EN = MSK7 // MSSP address masking (7 Bit address masking mode)

// CONFIG4L
#pragma config WPFP = PAGE_127  // Write/Erase Protect Page Start/End Location (Write Protect Program Flash Page 127)
#pragma config WPCFG = OFF      // Write/Erase Protect Configuration Region  (Configuration Words page not erase/write-protected)

// CONFIG4H
#pragma config WPDIS = OFF      // Write Protect Disable bit (WPFP<6:0>/WPEND region ignored)
#pragma config WPEND = PAGE_WPFP// Write/Erase Protect Region Select bit (valid when WPDIS = 0) (Pages WPFP<6:0> through Configuration Words erase/write protected)
#pragma config LS48MHZ = SYS48X8// Low Speed USB mode with 48 MHz system clock bit (System clock at 48 MHz USB CLKEN divide-by is set to 8)


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
void lcdWriteData(unsigned char n);
void lcdWriteCmd(unsigned char n);

void writeDigit(unsigned char n);
void writeByte(unsigned char n);

//******************************//
void lcdWriteChar(char ch);
void lcdClear(void);
void lcdSetPos(unsigned char x, unsigned char y);
void lcdWriteStrC(const char* s);
void lcdWriteStr(char* s);

/*
 * 
 */
//*****************************************//
//*****************************************//
int main(int argc, char** argv) {
  lcdInit();

  // Scritte dimostrative
  lcdSetPos(0,0);
  lcdWriteStrC("Roberino");
  lcdSetPos(0,1);
  lcdWriteStrC("Singh -Be -Rosa");
  while(1)
  {
      
  }


    return (EXIT_SUCCESS);
}
//*****************************************//
//*****************************************//
void writeDigit(unsigned char n)
{
  // Setup che dovrebbe essere di 10ns
  lcdE = 1;

  // Mette il digit nelle linee da D4 a D7
  if (n & 0x01) lcdD4 = 1; else lcdD4 = 0;
  if (n & 0x02) lcdD5 = 1; else lcdD5 = 0;
  if (n & 0x04) lcdD6 = 1; else lcdD6 = 0;
  if (n & 0x08) lcdD7 = 1; else lcdD7 = 0;

  //Hold time sicuramente superiore a 80ns.

  lcdE = 0;
}

void writeByte(unsigned char n)
{
  // Prima invia la parte alta
  writeDigit(n >> 4);
  // Dopo invia la parte bassa
  writeDigit(n & 0x0F);
}

void lcdWriteCmd(unsigned char n)
{
  // Per inviare un comando la linea RS deve essere posta a 0
  lcdRS = 0;
  // Invia il dato
  writeByte(n);
  // Ritarda per attendere la completa esecuzione del comando
  // In questo caso dipende dal comando
  // questi ritardi sono di 1600 e 43 us massimi
  if (!(n & 0xFC)) for(int i = 0; i < 100; i++) __delay_us(16);
  else __delay_us(43);
}

void lcdWriteData(unsigned char n)
{
  // Per inviare un comando la linea RS deve essere posta a 0
  lcdRS = 1;
  // Invia il dato
  writeByte(n);
  // Ritarda per attendere la completa esecuzione del comando
  // anche questo ritardo deve essere 43us
  __delay_us(43);
}

void lcdInit(void)
{
  // inizializza le linee di uscita
  lcdD4_DIR = 0;
  lcdD5_DIR = 0;
  lcdD6_DIR = 0;
  lcdD7_DIR = 0;

  lcdRS_DIR = 0;
  lcdRS = 0;

  lcdE_DIR = 0;
  lcdE = 0;

  // Ritardo di 15ms per stabilizzazione VCC
  __delay_ms(15);
  // esegue la sequenza di reset, quella che si usa in caso di alimentazione
  // che non garantisce un buon reset interno.

  lcdRS = 0;           // mette RS a 0
  writeDigit(0x03);
  for(int i = 0; i < 100; i++) __delay_us(41);  //sul datasheet e' indicato 4,1 ms

  writeDigit(0x03);
  for(int i = 0; i < 10; i++) __delay_us(12);// 120us. Il datasheet indica > 100 us.

  writeDigit(0x03);
  for(int i = 0; i < 10; i++) __delay_us(12);

  writeDigit(0x02);
  for(int i = 0; i < 10; i++) __delay_us(12);
  // Fine sequenza di reset

  // Programma il controller con il numero di righe da pilotare (1 o 2)
  // Data lenght 4 bit e font 5x8
  #if (1 == lcdLINES)
    lcdWriteCmd(0x20);
  #elif (2 == lcdLINES)
    lcdWriteCmd(0x28);
  #else
    errore [HD44780] numero di righe non supportato.
  #endif

  // Cancella il display
  lcdWriteCmd(0x01);

  // Imposta direzione d' incremento in avanti senza shift del display
  lcdWriteCmd(0x06);

  // lcd acceso e cursore lampeggiante a linea non visibile
  lcdWriteCmd(0x0C);
}

//******************************//
void lcdwriteChar(char ch)
{
  lcdWriteData(ch);
}

void lcdClear(void)
{
  lcdWriteCmd(lcdClear_CMD);
}

void lcdSetPos(unsigned char x, unsigned char y)
{
  unsigned char p;

  switch(y)
  {
    case 0: p = x; break;
    case 1: p = 64 + x; break;
    case 2: p = 20 + x; break;
    case 3: p = 84 + x; break;
  }
  p |= lcdSetADR_CMD;
  lcdWriteCmd(p);
}

void lcdWriteStrC(const char* s)
{
  while(*s) lcdWriteData(*s++);
}

void lcdWriteStr(char* s)
{
  while(*s) lcdWriteData(*s++);
}

