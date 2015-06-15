#define ENCODER_PIN 2
#define ENCODER_SIGNAL_TO_TURN 3
#define MOTOR_REDUCTION 43.3
#define ENABLE_MOTOR 3
#define PIN1_MOTOR 4
#define PIN2_MOTOR 7

float encoder_signal_to_turn_reduction;
float degree_per_signal;

float encoder_move_degree(float degree_to_do);
float get_encoder_velocity_ms(void);
float get_encoder_periode_ms(void);
void motor_go(void);
void motor_stop(void);
void init_encoder_variab(void);

void setup()
{
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  pinMode(ENABLE_MOTOR, OUTPUT);
  pinMode(PIN1_MOTOR, OUTPUT);
  pinMode(PIN2_MOTOR, OUTPUT);
  init_encoder_variab();
}


void loop()
{
}



float encoder_move_degree(float degree_to_do)
{

  float degree_done = 0;
  motor_go();

  while (degree_done <= degree_to_do)
  {
    while (!digitalRead(ENCODER_PIN));
    degree_done += degree_per_signal;
    while (digitalRead(ENCODER_PIN));
  }

  motor_stop();
  const float error_degree = degree_done - degree_to_do;
  return error_degree;
}



float get_encoder_velocity_ms(void)
{
  float periode = get_encoder_periode_ms();
  return (2 * PI) / periode;
}



float get_encoder_periode_ms(void)
{
  const unsigned long time_h_us = pulseIn(ENCODER_PIN, HIGH);
  const float time_h_ms = ((float)time_h_us) / 1000;
  return time_h_ms * 2;
}



void init_encoder_variab(void)
{
  encoder_signal_to_turn_reduction = MOTOR_REDUCTION * ENCODER_SIGNAL_TO_TURN;
  degree_per_signal = 360 / encoder_signal_to_turn_reduction;
}



void motor_go(void)
{
  digitalWrite(ENABLE_MOTOR, LOW);
  digitalWrite(PIN1_MOTOR, HIGH);
  digitalWrite(PIN2_MOTOR, LOW);
  digitalWrite(ENABLE_MOTOR, HIGH);
}



void motor_stop(void)
{
  digitalWrite(ENABLE_MOTOR, LOW);
  digitalWrite(PIN1_MOTOR, LOW);
  digitalWrite(PIN2_MOTOR, LOW);
}

