/* 
 * File:   joystickMain.c
 * Author: Harpal
 *
 * Created on 28 novembre 2014, 11.53
 */

#include <stdio.h>
#include <stdlib.h>
#include<pic16f628a.h>

// PIC16F628A Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>
#define _XTAL_FREQ 16000000
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG
#pragma config FOSC = EXTRCCLK  // Oscillator Selection bits (RC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, Resistor and Capacitor on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = OFF      // RA5/MCLR/VPP Pin Function Select bit (RA5/MCLR/VPP pin function is digital input, MCLR internally tied to VDD)
#pragma config BOREN = ON       // Brown-out Detect Enable bit (BOD enabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable bit (RB4/PGM pin has digital I/O function, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EE Memory Code Protection bit (Data memory code protection off)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)


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

    while(1)
    {

    }
    return (EXIT_SUCCESS);
}

void USART_init() {
    TRISBbits.TRISB1 = 1;
    TRISBbits.TRISB2 = 1;
    TXSTAbits.TXEN = 1;
    TXSTAbits.BRGH = 1;
    SPBRG = 104;
    TXSTAbits.SYNC = 0;
    RCSTAbits.SPEN = 1;
    RCSTAbits.CREN = 1;

    PIE1bits.RCIE = 1;

}
void sendByte(unsigned char Byte) {
    while (TXSTAbits.TRMT == 0);
    TXREG = Byte;
}

void sendString(unsigned char *s) {
    while (*s) {
        sendByte(*s);
        __delay_us(10);
        s++;
    }
}

void interrupt ISR(void) {
    if (PIR1bits.RCIF) // check if receive interrupt has fired
    {
        t = RCREG; // read received character to buffer

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


        PIR1bits.RCIF = 0; // reset receive interrupt flag
    }
}