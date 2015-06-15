/* 
 * File:   main.c
 * Author: Harpal
 *
 * Created on 25 settembre 2014, 10.24
 */

#include <stdio.h>
#include <stdlib.h>


#include "config.h"
#include "functions.h"
#include "lcd.h"

/*
 * 
 */
void interrupt ISR(void);
void showTempLight(void);
void showAccelerometerVal(void);
void showDistance(void);

int main(int argc, char** argv) {
    int i;
    
    TRISB = 0b11111100;
    TRISD = 0x00;
    TRISA = 0xff;

    //INTCONbits.RBIE = 1;            //PortB interrupt on change;
    //INTCON2bits.INTEDG1 = 0;        //Interrupt on falling edge
    //INTCON3bits.INT1IE = 1;         //enable external interrupt 1
    INTCONbits.GIE = 1;             //enabling all the interrupts



    ANCON0  = 0b11111111;            //set All pin as digital
    ANCON1 |= 0b00001111;

    lcdInit();

    while(1)
    {
        showDistance();

        __delay_ms(50);
        __delay_ms(50);


        for(i = 1; i < 180; i+=15)
        {
            setServo(i);
            __delay_ms(50);
            __delay_ms(50);
            __delay_ms(50);
            __delay_ms(50);
            __delay_ms(50);
            __delay_ms(50);
            __delay_ms(50);
            __delay_ms(50);
            __delay_ms(50);
        }

        for(i = 180; i > 1; i-=15)
        {
            setServo(i);
            __delay_ms(50);
            __delay_ms(50);
            __delay_ms(50);
            __delay_ms(50);
            __delay_ms(50);
            __delay_ms(50);
            __delay_ms(50);
            __delay_ms(50);
            __delay_ms(50);
        }
        
        /*
        showTempLight();

        for(i = 0; i < 10; i++) __delay_ms(10);
        showAccelerometerVal();
        for(i = 0; i < 100; i++) __delay_ms(10);
*/
    }

    return (EXIT_SUCCESS);
}

void interrupt ISR(void)
{
    if(INTCONbits.RBIF == 1)
    {
        //__delay_us(100);
        if(PORTBbits.RB4 == 0)
        {
            //showTempLight();
        }
        if(PORTBbits.RB5 == 0)
        {
            //showAccelerometerVal();
        }
        if(PORTBbits.RB6 == 0)
        {

        }
        INTCONbits.RBIF = 0;        //resetting the interrupt flag
    }
    if(INTCON3bits.INT1IF == 1)
    {
        showAccelerometerVal();
        INTCON3bits.INT1IF = 0;
    }
}
void showDistance(void)
{
    char buffer [10];
    lcdClear();
    lcdSetPos(0,0);
    lcdWriteStrC("Distance: [cm/uS]");
    lcdSetPos(0,1);
    sprintf(buffer,"%3.3f",readDistance());
    lcdWriteStrC(buffer);

}
void showTempLight(void)
{
    char buffer [10];
    lcdClear();
    lcdSetPos(0,0);
    sprintf(buffer,"temp: %1.3f",readTempF());
    lcdWriteStrC(buffer);
    lcdSetPos(0,1);
    sprintf(buffer,"light: %d",readLight());
    lcdWriteStrC(buffer);


}
void showAccelerometerVal(void)
{
    char buffer [10];
    lcdClear();

    lcdWriteStrC("Aclmtr values:");
    lcdSetPos(0,1);
    sprintf(buffer,"%1.2f",single_axis_measure(X_AXIS, iteration_point));
    lcdWriteStrC(buffer);
    lcdWriteChar(' ');
    sprintf(buffer,"%1.2f",single_axis_measure(Y_AXIS, iteration_point));
    lcdWriteStrC(buffer);
    lcdWriteChar(' ');
    sprintf(buffer,"%1.2f",single_axis_measure(Z_AXIS, iteration_point));
    lcdWriteStrC(buffer);
}
