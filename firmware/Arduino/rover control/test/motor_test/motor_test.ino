#include "main.h"
#include <LiquidCrystal.h>

LiquidCrystal lcd(6, 9, 10, 11, 12, 13);

void setup() {
  LCD_init();
  MOTOR_init();
  MOTOR_testMotors();
  MOTOR_moveTest();
}


void loop() {

  delay(100);
}



// initializing the LCD display
void LCD_init(void) {
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print(" Welcome to the ");
  lcd.setCursor(0, 1);
  lcd.print("  ");
  lcd.print(VERSION);
  lcd.print("  ");
  delay(5000);
  lcd.clear();
}



void MOTOR_init(void) {
  // Initializing the OUTPUT control array
  // Optional pulling down all the inputs for security
  for (unsigned int i = 0; i < NUM_MOTOR_OUTPUT_PIN; i++) {
    pinMode(motorPinOutput[i], OUTPUT);
    digitalWrite(motorPinOutput[i], LOW);
  }
}



// Funzione test movimentazione motori
void MOTOR_testMotors(void) {
  lcd.clear();
  lcd.print("Motor 1 testing ");
  MOTOR_testMotor(inp1M1, inp2M1, enableM1);
  lcd.print("Motor 2 testing ");
  MOTOR_testMotor(inp1M2, inp2M2, enableM2);
}



void MOTOR_testMotor(const unsigned int input1Pin, const unsigned int input2Pin, const unsigned int enablePin) {
  // Running the first motor clockwise
  lcd.setCursor(0, 1);
  lcd.print("  Accelerating  ");

  digitalWrite(enablePin, HIGH);
  digitalWrite(input1Pin, HIGH);
  digitalWrite(input2Pin, LOW);

  //Controlling the motor speed through PWM signal
  for (unsigned int i = 0; i < PWM_RESOLUTION; i++) {
    analogWrite(enablePin, i);
    delay(50);
  }

  // maintaining maximum speed
  lcd.setCursor(0, 1);
  lcd.print("    Maintain    ");
  delay(1000);
  lcd.setCursor(0, 1);
  lcd.print("  Decelerating  ");

  for (unsigned int i = PWM_RESOLUTION; i < 0; i--) {
    analogWrite(enablePin, i);
    delay(50);
  }

  delay(1000);

  //Running the second motor anticlockwise
  digitalWrite(input1Pin, LOW);
  digitalWrite(input2Pin, HIGH);

  lcd.setCursor(0, 1);
  lcd.print("  Accelerating  ");

  //Controlling the motor speed through PWM signal
  for (unsigned int i = 0; i < PWM_RESOLUTION; i++) {
    analogWrite(enablePin, i);
    delay(50);
  }

  //maintaining maximum speed
  lcd.setCursor(0, 1);
  lcd.print("    Maintain    ");
  delay(1000);
  lcd.setCursor(0, 1);
  lcd.print("  Decelerating  ");

  for (int i = PWM_RESOLUTION; i < 0; i--) {
    analogWrite(enablePin, i);
    delay(50);
  }

  digitalWrite(input2Pin, LOW);
  delay(1000);

  lcd.clear();
  lcd.print("   Motor TEST   ");
  lcd.setCursor(0, 1);
  lcd.print("     passed     ");
  delay(1000);
  //lcd.clear();
}



// Sequenza comandi di movimento per verificare il corretto funzionamento
void MOTOR_moveTest(void) {
  lcd.clear();
  lcd.print("   Motor move   ");
  lcd.setCursor(0, 1);
  lcd.print("     testing    ");

  MOTOR_forwardT(100, 5);
  delay(5000);

  MOTOR_forward(100);
  delay(5000);
  MOTOR_stop();

  MOTOR_backwardT(100, 5);
  delay(5000);

  MOTOR_backward(100);
  delay(5000);
  MOTOR_stop();

  MOTOR_turnRightT(100, 5);
  delay(5000);

  MOTOR_turnLeftT(100, 5);
  delay(5000);

  MOTOR_backward(100);
  delay(5000);
  MOTOR_stop();
}



// Funzione che manda sull'uscita di entrambi i motori lo stesso segnale PWM
// Movimento in avanti del robot
void MOTOR_forward(const unsigned int pwm) {
  lcd.clear();
  lcd.print(" Forward speed: ");
  lcd.setCursor(0, 1);
  lcd.print("     ");
  lcd.print(pwm);
  lcd.print("     ");

  digitalWrite(enableM1, LOW);
  digitalWrite(enableM2, LOW);
  digitalWrite(inp1M1, HIGH);
  digitalWrite(inp2M1, LOW);
  digitalWrite(inp1M2, HIGH);
  digitalWrite(inp2M2, LOW);
  analogWrite(enableM1, pwm);
  analogWrite(enableM2, pwm);
}



// Funzione che manda sull'uscita di entrambi i motori lo stesso segnale PWM per il tempo indicato
// Movimento in avanti del robot
void MOTOR_forwardT(const unsigned int pwm, const unsigned int time_s) {
  unsigned long time_ms;
  unsigned long time;

  lcd.clear();
  lcd.print(" Forward speed: ");
  lcd.setCursor(0, 1);
  lcd.print("     ");
  lcd.print(pwm);
  lcd.print("     ");

  digitalWrite(enableM1, LOW);
  digitalWrite(enableM2, LOW);
  digitalWrite(inp1M1, HIGH);
  digitalWrite(inp2M1, LOW);
  digitalWrite(inp1M2, HIGH);
  digitalWrite(inp2M2, LOW);
  analogWrite(enableM1, pwm);
  analogWrite(enableM2, pwm);

  delay(time_s * 1000);
  MOTOR_stop();
}



// Funzione che manda sull'uscita di entrambi i motori lo stesso segnale PWM
// Movimento in indietro del robot
void MOTOR_backward(const unsigned int pwm) {
  lcd.clear();
  lcd.print("Backward speed: ");
  lcd.setCursor(0, 1);
  lcd.print("     ");
  lcd.print(pwm);
  lcd.print("     ");

  digitalWrite(enableM1, LOW);
  digitalWrite(enableM2, LOW);
  digitalWrite(inp1M1, LOW);
  digitalWrite(inp2M1, HIGH);
  digitalWrite(inp1M2, LOW);
  digitalWrite(inp2M2, HIGH);
  analogWrite(enableM1, pwm);
  analogWrite(enableM2, pwm);
}



// Funzione che manda sull'uscita di entrambi i motori lo stesso segnale PWM per il tempo indicato
// Movimento in indietro del robot
void MOTOR_backwardT(const unsigned int pwm, const unsigned int time_s) {
  unsigned long time_ms;
  unsigned long time;

  lcd.clear();
  lcd.print("Backward speed: ");
  lcd.setCursor(0, 1);
  lcd.print("     ");
  lcd.print(pwm);
  lcd.print("     ");

  digitalWrite(enableM1, LOW);
  digitalWrite(enableM2, LOW);
  digitalWrite(inp1M1, LOW);
  digitalWrite(inp2M1, HIGH);
  digitalWrite(inp1M2, LOW);
  digitalWrite(inp2M2, HIGH);
  analogWrite(enableM1, pwm);
  analogWrite(enableM2, pwm);

  delay(time_s * 1000);
  MOTOR_stop();
}



// Funzione che arresta il movimento di entrambi i motori
void MOTOR_stop(void) {
  lcd.clear();
  lcd.print("   Motor STOP   ");

  digitalWrite(enableM1, LOW);
  digitalWrite(inp1M1, LOW);
  digitalWrite(inp2M1, LOW);
  digitalWrite(enableM2, LOW);
  digitalWrite(inp1M2, LOW);
  digitalWrite(inp2M2, LOW);
}



// Funzione che manda il segnale PW soltanto ad un motore per il tempo indicato
// Movimento a destra del robot
void MOTOR_turnRightT(const unsigned int pwm, const unsigned int time_s) {
  unsigned long time_ms;
  unsigned long time;

  lcd.clear();
  lcd.print("turn right speed");
  lcd.setCursor(0, 1);
  lcd.print("     ");
  lcd.print(pwm);
  lcd.print("     ");

  digitalWrite(enableM1, LOW);
  digitalWrite(enableM2, LOW);
  digitalWrite(inp1M1, HIGH);
  digitalWrite(inp2M1, LOW);
  analogWrite(enableM1, pwm);

  delay(time_s * 1000);
  MOTOR_stop();
}



// Funzione che manda il segnale PW soltanto ad un motore per il tempo indicato
// Movimento a destra del robot
void MOTOR_turnLeftT(const unsigned int pwm, const unsigned int time_s) {
  unsigned long time_ms;
  unsigned long time;

  lcd.clear();
  lcd.print("turn left speed: ");
  lcd.setCursor(0, 1);
  lcd.print("     ");
  lcd.print(pwm);
  lcd.print("     ");

  digitalWrite(enableM1, LOW);
  digitalWrite(enableM2, LOW);
  digitalWrite(inp1M2, HIGH);
  digitalWrite(inp2M2, LOW);
  analogWrite(enableM2, pwm);

  delay(time_s * 1000);
  MOTOR_stop();
}

