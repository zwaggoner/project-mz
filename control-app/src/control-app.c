#include <potentiometer.h>
#include <motor.h>
#include <controllers.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>

#define MOTOR_VOLTAGE 12.0f
#define PWM_PERIOD_US 1000

#define POT_ADC_MAX 1745
#define POT_OUTPUT_MAX 360 

#define LOOP_RATE_NS 1000000

#define Kp 0.3
#define Ki 0
#define Kd 0.01
#define Saturation MOTOR_VOLTAGE

#define Freq 12.5f//15.885154736f
#define Damp 0.05f

static volatile unsigned char running = 1;

void intHandler(int dummy)
{
  running = 0;
}

int main(int argc, char* argv[])
{
  unsigned char shaper = 1;
  float des;
  float ctrl_des;
  struct timeval ctl_start;
  struct timeval ctl_end;
  struct timespec sleep_timespec;

  signal(SIGINT, intHandler);

  // Determine if the input shaper is being used
  if(argc > 1)
  {
    if(!strcmp(argv[1], "-noshaper"))
    {
      shaper = 0;
    }
    else if(!strcmp(argv[1], "-shaper"))
    {
      shaper = 1;
    }
  }

	// Initialize the potentiometer
  if(potentiometer_init(POT_ADC_MAX, POT_OUTPUT_MAX))
  {
    printf("Failed to initialize potentiometer\n");
    return -1;
  }

  // Initialize the motor
  if(motor_init(MOTOR_VOLTAGE, PWM_PERIOD_US))
  {
    printf("Failed to initialize motor\n");
    potentiometer_deinit();
  }

  // Set the parameters for the input shaper and PID controller
  set_input_shaper_params(Freq, Damp, (float)LOOP_RATE_NS / (float)1000000000);
  set_pid_params(Kp, Ki, Kd, (float)LOOP_RATE_NS / (float)1000000000, Saturation);

  // Turn the motor on
  motor_set_state(ON);

  setpriority(PRIO_PROCESS, 0, -20); 
  
  while(running)
  {
    gettimeofday(&ctl_start, NULL);

    // Determine the user's desired position of the motor
    des = potentiometer_read();
    des = round(des / 0.35211267605f) * 0.35211267605f;

    // Counteract the vibrations of the rod
    if(shaper)
    {
      des = input_shaper(des);
    }
    
    // Determine the desired motor position with PID controller
    ctrl_des = pid_controller(des, motor_get_position());
    motor_set_voltage(ctrl_des);

    gettimeofday(&ctl_end, NULL);
  
    sleep_timespec.tv_sec = 0;
    sleep_timespec.tv_nsec = LOOP_RATE_NS - (((ctl_end.tv_sec * 1000000000) + (ctl_end.tv_usec * 1000)) - ((ctl_start.tv_sec * 1000000000) + (ctl_start.tv_usec * 1000)));
    
    nanosleep(&sleep_timespec, NULL);
  }

  // De-initialize the motor and potentiometer
  motor_deinit();
  potentiometer_deinit();

  return 0; 
}    
