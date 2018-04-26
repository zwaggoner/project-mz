#include <motor.h>
#include <kmotor.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#define MOTOR_DEV "/dev/mz-motor0"
#define SEND_FLOAT(f) *((long*)&f)
#define EXTRACT_FLOAT(i) *((float*)&i)

static int motor_fd = 0;

// Initialize the motor
int motor_init(float max_voltage, unsigned int pwm_period_us)
{
  int ret = 0;

  if(!motor_fd)
  {
    motor_fd = open(MOTOR_DEV, O_RDWR); 

    if(motor_fd)
    {
      ioctl(motor_fd, MOTOR_SET_MAX_VOLTAGE, SEND_FLOAT(max_voltage));
      ioctl(motor_fd, MOTOR_SET_PWM_PERIOD, pwm_period_us);
  
      ret = 0;
    }
    else
    {
      ret = -1;
    }
  }

  return ret;
}

// Set the motor state (on / off)
void motor_set_state(MotorState state_des)
{
  if(motor_fd)
  {
    ioctl(motor_fd, MOTOR_SET_STATE, (state_des == ON) ? 1 : 0);
  }
}

// Set the motor voltage
void motor_set_voltage(float v)
{
  if(motor_fd)
  {
    ioctl(motor_fd, MOTOR_SET_VOLTAGE, SEND_FLOAT(v));
  }
}

// Get the motor position
float motor_get_position()
{
  float ret = 0.0f;
  int raw_angle;

  if(motor_fd)
  {
    raw_angle = ioctl(motor_fd, MOTOR_GET_POSITION);

    ret = EXTRACT_FLOAT(raw_angle); 
  }

  return ret;
}

// De-initialize the motor
void motor_deinit()
{
    if(motor_fd)
    {
      motor_set_state(OFF);
      close(motor_fd);
      motor_fd = 0;
    }
}
