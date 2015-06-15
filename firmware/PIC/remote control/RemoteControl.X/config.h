/* 
 * File:   config.h
 * Author: Harpal
 *
 * Created on 8 marzo 2015, 15.01
 */

#ifndef CONFIG_H
#define	CONFIG_H

// PIC18F47J53 Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>
#include <pic18f47j53.h>
#define _XTAL_FREQ 12000000
//Config bits
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1L
#pragma config WDTEN = OFF      // Watchdog Timer (Disabled - Controlled by SWDTEN bit)
#pragma config PLLDIV = 1       // PLL Prescaler Selection (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CFGPLLEN = OFF   // PLL Enable Configuration Bit (PLL Disabled)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset (Disabled)
#pragma config XINST = OFF      // Extended Instruction Set (Disabled)

// CONFIG1H
#pragma config CPUDIV = OSC1    // CPU System Clock Postscaler (No CPU system clock divide)
#pragma config CP0 = OFF        // Code Protect (Program memory is not code-protected)

// CONFIG2L
#pragma config OSC = HS         // Oscillator (HS, USB-HS)
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
#pragma config ADCSEL = BIT12   // ADC 10 or 12 Bit Select (12 - Bit ADC Enabled)
#pragma config MSSP7B_EN = MSK7 // MSSP address masking (7 Bit address masking mode)

// CONFIG4L
#pragma config WPFP = PAGE_127  // Write/Erase Protect Page Start/End Location (Write Protect Program Flash Page 127)
#pragma config WPCFG = OFF      // Write/Erase Protect Configuration Region  (Configuration Words page not erase/write-protected)

// CONFIG4H
#pragma config WPDIS = OFF      // Write Protect Disable bit (WPFP<6:0>/WPEND region ignored)
#pragma config WPEND = PAGE_WPFP// Write/Erase Protect Region Select bit (valid when WPDIS = 0) (Pages WPFP<6:0> through Configuration Words erase/write protected)
#pragma config LS48MHZ = SYS48X8// Low Speed USB mode with 48 MHz system clock bit (System clock at 48 MHz USB CLKEN divide-by is set to 8)


#endif	/* CONFIG_H */

