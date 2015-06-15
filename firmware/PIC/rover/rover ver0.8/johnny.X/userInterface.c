#include <stdio.h>              //Standard c library for IO
#include <stdlib.h>             //Standard c library
#include <xc.h>                 //XC compiler library
#include <pic18f47j53.h>        //pic microcontroller library

#include "config.h"
#include "functions.h"          //library used for the sensors
#include "lcd.h"                //library used for the lcd control
#include "motorEncoder.h"       //library used for the motor and encoder control
#include "userInterface.h"      //library used for implementing the userinterface

//this function is used to display the distance on the lcd display
void showDistance(void) {
    char buffer [10];
    lcdClear();
    lcdSetPos(0, 0);
    lcdWriteStrC("Distance: [cm/uS]");
    lcdSetPos(0, 1);
    sprintf(buffer, "%3.3f", readDistance());
    lcdWriteStrC(buffer);
}
//this function is used to display the temprature and the light intensity in the room
void showTempLight(void) {
    char buffer [10];
    lcdClear();
    lcdSetPos(0, 0);
    sprintf(buffer, "temp: %1.3f", readTempF());
    lcdWriteStrC(buffer);
    lcdSetPos(0, 1);
    sprintf(buffer, "light: %d", readLight());
    lcdWriteStrC(buffer);
}
//this function is used to display the acceleration on the lcd display
void showAccelerometerVal(void) {
    char buffer [10];
    lcdClear();
    lcdWriteStrC("Aclmtr values:");
    lcdSetPos(0, 1);
    sprintf(buffer, "%1.2f", single_axis_measure(X_AXIS, iteration_point));
    lcdWriteStrC(buffer);
    lcdWriteChar(' ');
    sprintf(buffer, "%1.2f", single_axis_measure(Y_AXIS, iteration_point));
    lcdWriteStrC(buffer);
    lcdWriteChar(' ');
    sprintf(buffer, "%1.2f", single_axis_measure(Z_AXIS, iteration_point));
    lcdWriteStrC(buffer);
}
