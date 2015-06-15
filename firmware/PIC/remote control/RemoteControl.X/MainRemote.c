/* 
 * File:   MainRemote.c
 * Author: Harpal
 *
 * Created on 8 marzo 2015, 14.58
 */
#include <stdio.h>
#include <stdlib.h>
#include "config.h"
#include "LcdControl.h"
#include <pic18f47j53.h>
#include <xc.h>

// Assigning the action name to the buttons
#ifndef readUp
#define readUp (PORTDbits.RD0)
#define readUp_DIR (TRISDbits.TRISD0)
#endif

#ifndef readDown
#define readDown (PORTDbits.RD1)
#define readDown_DIR (TRISDbits.TRISD1)
#endif

#ifndef readEnter
#define readEnter (PORTDbits.RD2)
#define readEnter_DIR (TRISDbits.TRISD2)
#endif

#ifndef readExit
#define readExit (PORTDbits.RD3)
#define readExit_DIR (TRISDbits.TRISD3)
#endif

/*
 *
 */
/*************************/
int read_button(void);
void menu_update(int);
void display_option(int, int);
void enter_option(void);
void exit_option(void);

int menu = 0, m = 0, submenu = 0;
/*************************/
void interrupt ISR(void);
void config_init();
/***********************************/

/**********MAIN ROUTINE*************/
int main(int argc, char** argv) {
    //char i = '0';
    config_init();
    lcdInit();
    lcdWriteStrC("Counting test: ");
    while (1) {
        /*
        lcdSetPos(0,1);
        lcdWriteChar(i++);
        if(i > '9') i = '0';
        for(int j = 0; j < 10; j++) __delay_ms(50);
         */
        menu = read_button(); // wait for any changes in buttons
        menu_update(menu);
    }

    return (EXIT_SUCCESS);
}

/***********************************/
void config_init() {
    /*
    TRISD = 0x00;
    TRISA = 0xff;
    TRISC = 0xff;

    EECON2 = 0x55;
    EECON2 = 0xAA;
    PPSCONbits.IOLOCK = 0;
    RPINR1 = 18;
    EECON2 = 0x55;
    EECON2 = 0xAA;
    PPSCONbits.IOLOCK = 1;

    INTCON2bits.INTEDG1 = 1;        //Interrupt on rising edge
    INTCON3bits.INT1IE = 1;         //enable external interrupt 1
    INTCONbits.GIE = 1;             //enabling all the interrupts
     */
    ANCON0 = 0b11111111; //set All pin as digital
    ANCON1 = 0b11111111;

    readUp_DIR = 1;
    readDown_DIR = 1;
    readEnter_DIR = 1;
    readExit_DIR = 1;
}

void interrupt ISR(void) {
    if (INTCON3bits.INT1IF == 1) {
        /*************************/
        /****USER-INTERFACE*******/

        /*
        lcdClear();
        lcdWriteStrC(readSwitch());
        __delay_ms(50);
         */
        INTCON3bits.INT1IF = 0;
    }
}

//*****************************************************************//
//*****************************************************************//

int read_button() {
    static char last_val[4] = {0}; //array initialized to 0
    char actual_val[4] = {0};
    while (1) {
        
        actual_val[0] = readUp;
        actual_val[1] = readDown;
        actual_val[2] = readEnter;
        actual_val[3] = readExit;
        
        __delay_ms(10);
        for (int i = 0; i < 4; i++) {
            if (last_val[i] != actual_val[i])
            {
                last_val[i] = actual_val[i];
                return i; //check if there has occured any change
            }
            else last_val[i] = actual_val[i];
        } //for loop
    }// while loop
    return -1;
} //read_button

void menu_update(int action) {
    switch (action) { // the action depends on the button that has changed
        case 0: m++;
            break;
        case 1: m--;
            break;
        case 2: if (m == 1) {
                submenu = 1;
                m = 0;
            }
            break;
        case 3: if (submenu == 1) {
                submenu = 0;
                m = 0;
            }
            break;
    }//switch
    if (m < 0) m = 0;
    if (m > 4) m = 4;
    display_option(m, submenu);
}//menu_update

void display_option(int option, int sub) {
    lcdClear();
    switch (sub) {
        case 0: switch (option) {
                case 0: lcdWriteStrC("op0");
                    break;
                case 1: lcdWriteStrC("op1");
                    break;
                case 2: lcdWriteStrC("op2");
                    break;
                case 3: lcdWriteStrC("op3");
                    break;
                case 4: lcdWriteStrC("op4");
                    break;
            }
            break;
        case 1: switch (option) {
                case 0: lcdWriteStrC("sop0");
                    break;
                case 1: lcdWriteStrC("sop1");
                    break;
                case 2: lcdWriteStrC("sop2");
                    break;
                case 3: lcdWriteStrC("sop3");
                    break;
                case 4: lcdWriteStrC("sop4");
                    break;
            }
    }//switch
}//display_option