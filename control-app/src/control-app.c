#include <potentiometer.h>
#include <motor.h>
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

#define LOOP_RATE_NS 1000000

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
    else
    {
      des = atof(argv[1]);
    }
  }

	

  if(potentiometer_init())
  {
    printf("Failed to initialize potentiometer\n");
    return -1;
  }

  if(motor_init(MOTOR_VOLTAGE, PWM_PERIOD_US))
  {
    printf("Failed to initialize motor\n");
    potentiometer_deinit();
  }

  motor_set_state(ON);

  setpriority(PRIO_PROCESS, 0, -20); 
  
  while(running)
  {
    gettimeofday(&ctl_start, NULL); 
    //des = potentiometer_read();
    /**if(shaper)
    {
    }**/

      
    ctrl_des = 0.446*(des - motor_get_position());
    ctrl_des = (abs(ctrl_des) > 12.0) ? 12.0 * (ctrl_des / abs(ctrl_des)) : ctrl_des;

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