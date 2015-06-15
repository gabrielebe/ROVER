#include "main.h"
#include <LiquidCrystal.h>
#include <Servo.h>

LiquidCrystal lcd(5, 6, 4, 11, 12, 13);
Servo servo;

unsigned long dist[5];

void setup() {
  Serial.begin(BAUD_RATE);
  LCD_init();
  SERVO_test();
}


void loop() {
  SENSOR_serialPrint();
  SENSOR_lcdPrint();
  servo.write(SERVO_ULTRASONIC_getAngleToMove());
  delay(5000);
}



// initializing the LCD display
void LCD_init(void) {
  lcd.begin(LCD_COL, LCD_ROW);
  lcd.clear();
  lcd.print(" Welcome to the ");
  lcd.setCursor(0, 1);
  lcd.print("  ");
  lcd.print(VERSION);
  lcd.print("   ");
  delay(5000);
  lcd.clear();
}



// Funzione test servomotore: esegue una rotazione completa di 180 gradi
void SERVO_test(void) {
#ifdef DEBUG_PINGDISTANCE
  lcd.clear();
  lcd.print("Servomotor test ");
  Serial.println("Servotest function");
#endif

  servo.write(90);
  delay(100);

  // Goes from 0 degrees to 180 degrees in steps of 1 degree
  for (unsigned int i = 0; i < SERVO_MAX_ANGLE; i++) {
    servo.write(i);
    delay(15);
  }

  // Goes from 180 degrees to 0 degrees in steps of 1 degree
  for (unsigned int i = SERVO_MAX_ANGLE; i >= 1; i--) {
    servo.write(i);
    delay(15);
  }

  servo.write(90);
  delay(15);

#ifdef DEBUG_PINGDISTANCE
  lcd.clear();
#endif
}



void SENSOR_serialPrint(void) {
  unsigned long distance = ULTRASONIC_getDistance();
  float temperature = TEMPERATURE_get();

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print("   ");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println();
}



void SENSOR_lcdPrint(void) {
  unsigned long distance = ULTRASONIC_getDistance();
  float temperature = TEMPERATURE_get();

  lcd.clear();
  lcd.print("Dist: ");
  lcd.print(distance);
  lcd.setCursor(0, 1);
  lcd.print("Temp: ");
  lcd.print(temperature);
}



// Ritorna -1 quando non è possibile individuare una direzione conforme alle specifiche,
// altrimenti ritorna l'angolo a cui corrisponde una distanza maggiore dall'oggetto più vicino
int SERVO_ULTRASONIC_getAngleToMove(void) {
  const unsigned int servoStep = 15;
  const unsigned int passiServo = 180 / servoStep;
  unsigned int angleToMove = 0;
  unsigned long distance[passiServo];

#ifdef DEBUG_PINGDISTANCE
  lcd.clear();
  lcd.print("  GetBestAngle  ");
  Serial.println("GetBestAngle function");
#endif

  SERVO_scan(&distance[0], passiServo, servoStep);
  bubbleSort(&distance[0], passiServo);
  angleToMove = getBestAngle(&distance[0], passiServo, servoStep);

#ifdef DEBUG_PINGDISTANCE
  lcd.setCursor(0, 1);
  lcd.print("AngolToMove: ");
  lcd.print(angleToMove);
  Serial.println();
  Serial.print("AngolToMove");
  Serial.println(angleToMove);
  Serial.println();
#endif

  return angleToMove;
}



// Funzione che restituisce la distanza dall'oggetto più vicino rispetto al sensore di distanza
unsigned long ULTRASONIC_getDistance(void) {
  unsigned long currentMillis = millis();

  if (abs(currentMillis - lastMeasurement) >= 1000) {
    currentTemperature = TEMPERATURE_get();
    lastMeasurement = currentMillis;
  }

  const unsigned long duration = measureDistance();
  return scaledValue(microsecondsToCm(duration));
}



float TEMPERATURE_get(void) {
  // Da implementare
}



// Funzione privata che implementa l'algoritmo bubble sort di ordinamento di un vettore in modo crescente
void bubbleSort(unsigned long* distance, const unsigned int passiServo) {
  unsigned long distanceTemp;

  for (unsigned int i = 0; i < (passiServo - 1); i++) {
    for (unsigned int j = (i + 1); j < passiServo; j++) {

      if (distance[j] < distance[i]) {
        distanceTemp = distance[i];
        distance[i] = distance[j];
        distance[j] = distanceTemp;
      }
    }
  }
}



// Funzione privata che restituisce l'angolo verso cui muoversi dato l'array con le distance rilevate
int getBestAngle(const unsigned long* distance, const unsigned int passiServo, const unsigned int servoStep) {
  int angleToMove = -1;
  boolean exit = false;

  for (unsigned int i = 0; i < passiServo; i++) {

    if ((!exit) && (distance[i] >= COLLISION_DISTANCE)) {
      angleToMove = servoStep;
      exit = true;
    }
  }

  return angleToMove;
}



//  Funzione privata per la movimentazione del servomotore e la misura della distanza in angoli prefissati
void SERVO_scan(unsigned long* distance, const unsigned int passiServo, const unsigned int servoStep) {
  unsigned int angle = 0;

  for (unsigned int i = 0; i <= passiServo; i++) {
    servo.write(angle);
    delay(15);
    distance[i] = ULTRASONIC_getDistance();
    angle += servoStep;
  }

  servo.write(90);
}



// Funzione privata per la gestione della misura della distanza tramite il sensore di distanza
long scaledValue(const float value) {
  float roundOffset = value < 0 ? -0.5 : 0.5;
  return (long)(value * 100 + roundOffset);
}



// Funzione privata per la gestione della misura della distanza tramite il sensore di distanza
const float microsecondsPerCm(void) {
  return 1 / ((331.5 + (0.6 * currentTemperature)) / 10000);
}



// Funzione privata per la gestione della misura della distanza tramite il sensore di distanza
const float sensorOffset(void) {
  return SENSOR_GAP * microsecondsPerCm() * 2;
}



// Funzione privata per la gestione della misura della distanza tramite il sensore di distanza
const float microsecondsToCm(const unsigned long microseconds) {
  const float netDistance = max(0, microseconds - sensorOffset());
  return netDistance / microsecondsPerCm() / 2;
}



// Funzione privata per la gestione della misura della distanza tramite il sensore di distanza
const unsigned long measureDistance(void) {
  pinMode(ULTRASONIC_SENSOR_PIN, OUTPUT);
  digitalWrite(ULTRASONIC_SENSOR_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_SENSOR_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(ULTRASONIC_SENSOR_PIN, LOW);
  pinMode(ULTRASONIC_SENSOR_PIN, INPUT);
  return pulseIn(ULTRASONIC_SENSOR_PIN, HIGH);
}

