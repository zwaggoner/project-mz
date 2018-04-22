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

#define MOTOR_VOLTAGE 12.0f
#define PWM_PERIOD_US 1000

#define POT_ADC_MAX 1365
#define POT_OUTPUT_MAX 360 

#define LOOP_RATE_NS 1000000

#define Kp 0.142065690258213
#define Ki 0.081333487477284
#define Kd 0.0520942929296246
#define Saturation MOTOR_VOLTAGE

#define Freq 13.5860741971
#define Damp 0.03

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

	

  if(potentiometer_init(POT_ADC_MAX, POT_OUTPUT_MAX))
  {
    printf("Failed to initialize potentiometer\n");
    return -1;
  } 
  if(motor_init(MOTOR_VOLTAGE, PWM_PERIOD_US))
  {
    printf("Failed to initialize motor\n");
    potentiometer_deinit();
  }

  set_input_shaper_params(Freq, Damp, LOOP_RATE_NS / 1000000000);
  set_pid_params(Kp, Ki, Kd, LOOP_RATE_NS / 1000000000, Saturation);

  motor_set_state(ON);

  setpriority(PRIO_PROCESS, 0, -20); 
  
  while(running)
  {
    gettimeofday(&ctl_start, NULL); 
    des = potentiometer_read();

    if(shaper)
    {
      des = input_shaper(des);
    }
      
    ctrl_des = pid_controller(des, motor_get_position());

    motor_set_voltage(ctrl_des);

    gettimeofday(&ctl_end, NULL);
  
    sleep_timespec.tv_sec = 0;
    sleep_timespec.tv_nsec = LOOP_RATE_NS - (((ctl_end.tv_sec * 1000000000) + (ctl_end.tv_usec * 1000)) - ((ctl_start.tv_sec * 1000000000) + (ctl_start.tv_usec * 1000)));
    
    nanosleep(&sleep_timespec, NULL);
  }

  motor_deinit();
  potentiometer_deinit();

  return 0; 
}    
