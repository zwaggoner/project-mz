#ifndef MOTOR_H
#define MOTOR_H

typedef enum
{
  ON,
  OFF,
} MotorState;

int motor_init(float max_voltage, unsigned int pwm_period_us);

void motor_set_state(MotorState state_des);

void motor_set_voltage(float v);

float motor_get_position();

void motor_deinit();

#endif
