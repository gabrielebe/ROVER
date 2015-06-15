/* 
 * File:   Analog.h
 * Author: Harpal
 *
 * Created on 9 gennaio 2015, 11.35
 */

#ifndef ANALOG_H
#define	ANALOG_H

#include <xc.h>
#include <pic16f88.h>

#define _XTAL_FREQ 8000000     //defining the work frequancy. needed for the delay generation
#define POWER_SUPPLY 5
int read(int,char);
float readLight(void);
float readTemp(void);
void tempCalibration(void);

float single_axis_measure(unsigned int axis, int iteration);
float read_accelerometer(const unsigned int axis);
double pointToVoltage(double point);
float voltageToG(const float voltage);

//analog channel selection
#define CHANNEL_LIGHT 0
#define CHANNEL_TEMPRATURE 1
#define CHANNEL_X_AXIS 2
#define CHANNEL_Y_AXIS 3
#define CHANNEL_Z_AXIS 4
#define CHANNEL_CALIBRATION 5
//costants definition
#define TEMP_COSTANT -23.15
#define C_COSTANT 10
#define A_COSTANT 31,62
#define M_COSTANT -4
#define ALPHA_COSTANT 0,5
#define GAIN_LIGHT 2.22
#define ZERO_G_VOLTAGE_REF 1.65
#define SENSOR_SENSITIVITY 0.300
//#define ITERATION_POINTS 10
//errors declaration
#define ERROR_DIVIDE_BY_ZERO -10
#define ERROR_WRONG_CHANNEL -11
#define ERROR_LIGHT_RANGE -12
#define ERROR_TEMP_RANGE -13
#endif	/* ANALOG_H */
