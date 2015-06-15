#include "main.h"

void setup()
{
    Serial.begin(BAUD_RATE);
}


void loop()
{   
    tiltSensorPrint();
    delay(100);
}


void tiltSensorPrint(void)
{
    float roll, pitch;
    tiltSensorRead(&roll, &pitch);
   
    Serial.print("Pitch:  ");
    Serial.print(pitch);
    Serial.print("   Roll:  ");
    Serial.println(roll);
    Serial.println();
}


void gAccelerationMeasuramentPrint(void)
{   
    float xAssG, yAssG, zAssG;
    gAccelerationMeasuramentRead(&xAssG, &yAssG, &zAssG);
	
    Serial.print("X: ");
    Serial.print(xAssG);
    Serial.print("  Y: ");
    Serial.print(yAssG);
    Serial.print("  Z: ");
    Serial.print(zAssG);
    Serial.println();
}


void tiltSensorRead(float* roll, float* pitch)
{
    float xAssG, yAssG, zAssG;
    gAccelerationMeasuramentRead(&xAssG, &yAssG, &zAssG);
    
    /* Roll & Pitch Equations */
    *roll  = (atan2(yAssG, zAssG) * 180.0) / M_PI;
    *pitch = (atan2(xAssG, sqrt(-yAssG * yAssG + zAssG * zAssG)) * 180.0) / M_PI;
}


/* Read the value from the sensor */
void gAccelerationMeasuramentRead(float* xAssG, float* yAssG, float* zAssG)
{
    *xAssG = singlAassMeasure(X_ASS);
    *yAssG = singlAassMeasure(Y_ASS);
    *zAssG = singlAassMeasure(Z_ASS);
}


float singlAassMeasure(int ass)
{
    int assPoint;
    float assRead;
    float assSum = 0.0;
	
    for(int i = 0; i < ITERATION_POINT; i++)
    {        
        assPoint = analogRead(ass);   
        assRead = (((float)assPoint) * 5.0) / 1023;        
        assSum = assSum + assRead;
    }
    
    float mediumValue = assSum / ITERATION_POINT;      
    float voltageToG = (mediumValue - ZERO_G_REF_VOLTAGE) / SENSOR_SENTITIVITY;
    return voltageToG;
}

