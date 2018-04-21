#include <potentiometer.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>

#define POT_DEV "/sys/bus/iio/devices/iio:device1/in_voltage8_raw"
#define POT_DEGREES 270.0f
#define ADC_MAX 0xFFF
#define MAX_DIGITS 5

static int pot_fd = 0;
static char reading_buf[MAX_DIGITS + 1];

int potentiometer_init()
{
  int ret = 0;

  if(!pot_fd)
  {
    pot_fd = open(POT_DEV, O_RDONLY); 
    
    ret = (pot_fd) ? 0 : -1;
  }

  return ret;
}

float potentiometer_read()
{
  float ret = 0.0f;

  if(pot_fd)
  {
    if(read(pot_fd, reading_buf, sizeof(reading_buf)))
    {
      int raw = atoi(reading_buf);
      ret = ((float)ret) / ((float)ADC_MAX) * POT_DEGREES;
    }
  }

  return ret; 
}

void potentiometer_deinit()
{
  if(pot_fd)
  {
    close(pot_fd);
    pot_fd = 0;
  }
}


