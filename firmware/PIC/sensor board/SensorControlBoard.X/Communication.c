#include "Communication.h"
#include <stdio.h>
#include <stdlib.h>
#include <pic16f88.h>
#include <xc.h>
void initialize_i2c(void)
{
    //setting the SDA and SCL pin as input
    TRISBbits.TRISB1 = 1;   //SDA pin
    TRISBbits.TRISB4 = 1;   //SCL pin
    //selecting the funztion mode as I2C slave with 7 bit address
    //without any interrupt on start and stop
    SSPCONbits.SSPM0 = 0;
    SSPCONbits.SSPM1 = 1;
    SSPCONbits.SSPM2 = 1;
    SSPCONbits.SSPM3 = 0;
    SSPCONbits.CKP = 1;     //enabling the clock
    SSPCONbits.SSPEN = 1;   //enabling the module
    //setting the adress of the devices
    SSPADD = 0x30 << 1;
    //clearing all the current status of the module
    SSPSTAT = 0x00;
    //enabling the intterrupt
    PIE1bits.SSPIE = 1;
    //clearing the intterrupt flag
    PIR1bits.SSPIF = 0;
}