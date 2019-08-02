#ifndef MOTOR_H
#define MOTOR_H

/**
 * Enum for setting desired motor state
 */
typedef enum
{
  ON,
  OFF,
} MotorState;

/**
 * Function to initialize motor driver and set the maximum voltage and the PWM period
 * @param max_voltage Maximum voltage supplied to motor driver (corresponding to 100% duty cycle)
 * @param pwm_period_us The desired period of the PWM signal in microseconds
 * @return Operation status
 */
int motor_init(float max_voltage, unsigned int pwm_period_us);

/**
 * Set the desired motor state
 * @param state_des Desired motor state
 */
void motor_set_state(MotorState state_des);

/**
 * Set the desired motor voltage
 * @param v The desired voltage
 */
void motor_set_voltage(float v);

/**
 * Get the current encoder position
 * @return Current encoder angle
 */
float motor_get_position();

/**
 * De initialize motor
 */
void motor_deinit();

#endif
