/* 
 * File:   Main.c
 * Author: Harpal
 *
 * Created on 18 novembre 2014, 10.26
 */

#include <stdio.h>
#include <stdlib.h>

// PIC18F47J53 Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>
#include <pic18f47j53.h>
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
#pragma config OSC = HS         // Oscillator (HS, USB-HS)
#pragma config SOSCSEL = HIGH   // T1OSC/SOSC Power Selection Bits (High Power T1OSC/SOSC circuit selected)
#pragma config CLKOEC = OFF     // EC Clock Out Enable Bit  (CLKO output disabled on the RA6 pin)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor (Enabled)
#pragma config IESO = OFF       // Internal External Oscillator Switch Over Mode (Disabled)

// CONFIG2H
#pragma config WDTPS = 32768    // Watchdog Postscaler (1:32768)

// CONFIG3L
#pragma config DSWDTOSC = INTOSCREF// DSWDT Clock Select (DSWDT uses INTRC)
#pragma config RTCOSC = T1OSCREF// RTCC Clock Select (RTCC uses T1OSC/T1CKI)
#pragma config DSBOREN = OFF    // Deep Sleep BOR (Disabled)
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

/*
 * 
 */

#define STRLEN 20
void USART_init();
void sendByte(char Byte);
void sendString(char *s);

volatile unsigned char t = 0;
volatile unsigned char rcindex = 0;
volatile unsigned char rcbuf[STRLEN];

int main(int argc, char** argv) {
    USART_init();
    __delay_ms(50);
    sendString("Comunication is ok\n");

    INTCONbits.PEIE = 1;    // enable peripheral interrupts
    INTCONbits.GIE = 1;     // enable interrupts

    while (1) {

    }
    return (EXIT_SUCCESS);
}

void USART_init() {
    TXSTA1bits.TXEN = 1;
    TXSTA1bits.BRGH = 1;
    RCSTA1bits.CREN = 1;
    TXSTA1bits.SYNC = 0;


    TRISCbits.TRISC6 = 1;
    TRISCbits.TRISC7 = 1;
    
    SPBRG1 = 77;


    RCSTA1bits.SPEN = 1;
    PIE1bits.RC1IE = 1;      // enable USART receive interrupt
    
}

void sendByte(unsigned char Byte) {
    while (TXSTA1bits.TRMT == 0);
    TXREG1 = Byte;
}

void sendString(unsigned char *s) {
    while (*s) {
        sendByte(*s);
        __delay_us(10);
        s++;
    }
}

void interrupt ISR(void) {
    if (PIR1bits.RC1IF) // check if receive interrupt has fired
    {
        t = RCREG1; // read received character to buffer

        // check if received character is not new line character
        // and that maximum string length has not been reached
        if ((t != '\n') && (rcindex < STRLEN)) {
            rcbuf[rcindex] = t; // append received character to string
            rcindex++; // increment string index
        }
        else {
            rcindex = 0; // reset string index
            sendString(rcbuf); // echo received string
            sendByte('\n');
            for(int i = 0; i < STRLEN; i++)
                rcbuf[i] = 0;
        }


        PIR1bits.RC1IF = 0; // reset receive interrupt flag
    }
}
