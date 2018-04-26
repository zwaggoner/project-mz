#include <potentiometer.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>

#define POT_DEV "/sys/bus/iio/devices/iio:device1/in_voltage8_raw"
#define MAX_DIGITS 5

static int pot_fd = 0;
static char reading_buf[MAX_DIGITS + 1];
static unsigned int adc_max;
static float output_max;

// Initialize potentiometer
int potentiometer_init(unsigned int adc_max_val, float output_max_val)
{
  int ret = 0;

  adc_max = adc_max_val;
  output_max = output_max_val;

  if(!pot_fd)
  {
    pot_fd = open(POT_DEV, O_RDONLY); 
    
    ret = (pot_fd) ? 0 : -1;
  }

  return ret;
}

// Read potentiometer value
float potentiometer_read()
{
  float ret = 0.0f;

  if(pot_fd)
  {
    lseek(pot_fd, 0, SEEK_SET);
    if(read(pot_fd, reading_buf, sizeof(reading_buf)))
    {
      int raw = atoi(reading_buf);
      ret = ((float)raw) / ((float)adc_max) * output_max;
    }
  }

  return ret; 
}

// De-initialize potentiometer
void potentiometer_deinit()
{
  if(pot_fd)
  {
    close(pot_fd);
    pot_fd = 0;
  }
}


