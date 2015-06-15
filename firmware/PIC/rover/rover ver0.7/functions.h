/* 
 * File:   functions.h
 * Author: Harpal
 *
 * Created on 25 settembre 2014, 10.49
 */

#ifndef FUNCTIONS_H
#define	FUNCTIONS_H

//******************************//
//******************************//
//*******ADC**********//
//defines

//function
int read(int);

//******************************//
//******************************//
//*******Temp********//
//defines

//function
float readTempF(void);
//******************************//
//******************************//
//*******Light********//
//defines

//function
int readLight(void);
//******************************//
//button reading
char *readSwitch(void);
//******************************//
//*******Accelerometer********//
//defines
#define X_AXIS 2
#define Y_AXIS 3
#define Z_AXIS 4

#define zero_g_ref_voltage 1.65
#define sensor_sensitivity 0.300
#define iteration_point 10

//function
float single_axis_measure(unsigned int axis, const float iteration);
float read_accelerometer(const unsigned int axis);
float pointToVoltage(const unsigned int point);
float voltageToG(const float voltage);
//******************************//
//******************************//
//Ultrasonic distance sensor
#define distanceCalibration (((double)331.5 + ((double)0.6 * readTempF()))/(double)10000)
double readDistance(void);
//******************************//
//Servo
void setServo(int degree);
//******************************//

#endif	/* FUNCTIONS_H */
