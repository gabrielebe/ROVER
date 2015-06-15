/*
 * File:   Analog.h
 * Author: Harpal
 *
 * Created on 9 gennaio 2015, 11.35
 */

#ifndef ANALOG_H
#define	ANALOG_H

#include <xc.h>
#include <pic16f886.h>

#define _XTAL_FREQ 8000000     // Defining the work frequancy. needed for the delay generation
#define POWER_SUPPLY 5

// Analog channel selection
#define CHANNEL_LIGHT       (1)
#define CHANNEL_TEMPRATURE  (0)
#define CHANNEL_X_AXIS      (9)
#define CHANNEL_Y_AXIS      (11)
#define CHANNEL_Z_AXIS      (13)
#define CHANNEL_CALIBRATION (2)

// Costants definition
#define TEMP_COSTANT       (-23.15)
#define C_COSTANT          (10)
#define A_COSTANT          (31,62)
#define M_COSTANT          (-4)
#define ALPHA_COSTANT      (0,5)
#define GAIN_LIGHT         (2.22)
#define ZERO_G_VOLTAGE_REF (1.65)
#define SENSOR_SENSITIVITY (0.300)

// Errors declaration
#ifndef ERROR_LED
#define ERROR_LED_DIR       (TRISCbits.TRISC6)
#define ERROR_LED           (PORTCbits.RC6)
#endif

#define ERROR_DIVIDE_BY_ZERO -10
#define ERROR_WRONG_CHANNEL  -11
#define ERROR_LIGHT_RANGE    -12
#define ERROR_TEMP_RANGE     -13

int read(int, char);
float readLight(void);
float readTemp(void);
float tempCalibration(void);
float singleAxisMeasure(int axis, int iteration);
float readAccelerometer(int axis);
double pointToVoltage(double point);

#endif	/* ANALOG_H */
