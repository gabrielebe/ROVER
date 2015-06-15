#ifndef JHONNY_H_INCLUDED
#define JHONNY_H_INCLUDED

/* Motor direction command define */
typedef enum motorDirection
{
  NONE,
  TURN,
  FOREWARD,
  BACKWARD,
  STOP,
  RIGHT,
  LEFT,
} MOTOR_COMMAND;

/* Motor pins defines */
typedef enum motorPins
{
  MOTOR_PIN_AM1  =  53,
  MOTOR_PIN_BM1  =  52,
  MOTOR_PIN_AM2  =  51,
  MOTOR_PIN_BM2  =  50,
  MOTOR_PIN_AM3  =  49,
  MOTOR_PIN_BM3  =  48,
  MOTOR_PIN_AM4  =  47,
  MOTOR_PIN_BM4  =  46,
  MOTOR_PIN_EN1  =  2,
  MOTOR_PIN_EN2  =  3,
  MOTOR_PIN_EN3  =  4,
  MOTOR_PIN_EN4  =  5,
} MOTOR_PIN;

void MOTOR_init(void);
void MOTOR_move(motorDirection comand, int pwm, int angle);
void MOTOR_turn(int angle, int pwm);
void MOTOR_forward(int pwm);
void MOTOR_backward(int pwm);
void MOTOR_stop(void);
void MOTOR_right(int pwm, int angle);
void MOTOR_turnRight(int pwm);
void MOTOR_left(int pwm, int angle);
void MOTOR_turnLeft(int pwm);

#endif // JHONNY_H_INCLUDED

