/* 
 * File:   functions.h
 * Author: Harpal
 *
 * Created on 24 dicembre 2014, 10.56
 */

#ifndef FUNCTIONS_H
#define	FUNCTIONS_H
//definition of all the variables
#define X_AXIS 2
#define Y_AXIS 3
#define Z_AXIS 4

#define zero_g_ref_voltage 1.65
#define sensor_sensitivity 0.300
#define iteration_point 10

//definition of all the needed pins
#ifndef analogChannelPin0
#define analogChannelPin0       (LATEbits.LATE0)
#define analogChannelPin0_DIR   (TRISEbits.TRISE0)
#endif
#ifndef analogChannelPin1
#define analogChannelPin1       (LATCbits.LATC2)
#define analogChannelPin1_DIR   (TRISCbits.TRISC2)
#endif
#ifndef analogChannelPin2
#define analogChannelPin2       (LATDbits.LATD4)
#define analogChannelPin2_DIR   (TRISDbits.TRISD4)
#endif
#ifndef analogChannelPin3
#define analogChannelPin3       (LATDbits.LATD6)
#define analogChannelPin3_DIR   (TRISDbits.TRISD6)
#endif
#ifndef servoPin
#define servoPin                (LATDbits.LATD5)
#define servoPin_DIR            (TRISDbits.TRISD5)
#endif
#ifndef distanceTriggerPin
#define distanceTriggerPin      (LATEbits.LATE1)
#define distanceTriggerPin_DIR  (TRISEbits.TRISE1)
#endif
#ifndef echoPin
#define echoPin                 (PORTEbits.RE2)               //it has only to read
#define echoPin_DIR             (TRISEbits.TRISE2)
#endif
#ifndef switchDataPin
#define switchDataPin
#define switchDataPin_DIR
#endif
#ifndef switchClockPin
#define switchClockPin
#define switchClockPin_DIR
#endif
#ifndef switchLatchPin
#define switchLatchPin
#define switchLatchPin_DIR
#endif
#ifndef switchInputPin              //Warning: it need to read a PORT register
#define switchInputPin        (RD7)
#define switchInputPin_DIR    (TRISDbits.TRISD7)
#endif

void configSense(void);
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
