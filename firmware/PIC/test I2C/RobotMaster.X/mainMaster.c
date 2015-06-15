/* 
 * File:   mainMaster.c
 * Author: Harpal
 *
 * Created on 22 gennaio 2015, 20.36
 */

#include <stdio.h>
#include <stdlib.h>

#include "lcd.h"
#include "config.h"
#include "masterI2C.h"
/*
 * 
 */
int main(int argc, char** argv) {
    char value, count = '0';
    lcdInit();
    i2c_init();
    for(int i = 0; i < 16; i++)
    {
        sendByte(count++,0x30<<1);
        lcdSetPos(i,0);
        lcdWriteChar(count);
        //for(int i = 0; i<10; i++ )
        __delay_ms(10);
        value = readByte(0x30<<1);
        lcdSetPos(i,1);
        lcdWriteChar(value);
        //for(int i = 0; i<10; i++ )
        __delay_ms(10);
    }
    while(1)
    {
        

    }

    return (EXIT_SUCCESS);
}

