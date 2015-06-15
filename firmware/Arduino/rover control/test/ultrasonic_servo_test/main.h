#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

#define BAUD_RATE 9600              // Baud rate della comunicazione seriale emulata tramite USB
#define VERSION "PING ver1.2"       // Versione del firmware
#define LCD_COL 16                  // Colonne dell'LCD
#define LCD_ROW 2                   // Righe dell'LCD
#define SERVO_MAX_ANGLE 180         // Angolo massimo di rotazione del servomotore
#define SENSOR_GAP 0.2              // Parametro usato per correzione errori del sensore di distanza PING
#define ULTRASONIC_SENSOR_PIN 4
#define SERVO_PIN 10
#define COLLISION_DISTANCE 10
#define DEBUG_PINGDISTANCE          // Define che indica di aggiungere le istruzione per il dubug durante la compilazione

unsigned long lastMeasurement;
float currentTemperature;

void LCD_init(void);
void SERVO_test(void);
void SENSOR_serialPrint(void);
void SENSOR_lcdPrint(void);
int SERVO_ULTRASONIC_getAngleToMove(void);
unsigned long ULTRASONIC_getDistance(void);
float TEMPERATURE_get(void);
void bubbleSort(unsigned long* distance, const unsigned int passiServo);
int getBestAngle(const unsigned long* distance, const unsigned int passiServo, const unsigned int servoStep);
void SERVO_scan(unsigned long* distance, const unsigned int passiServo, const unsigned int servoStep);
long scaledValue(const float value);
const float microsecondsPerCm(void);
const float sensorOffset(void);
const float microsecondsToCm(const unsigned long microseconds);
const unsigned long measureDistance(void);

#endif

