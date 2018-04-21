#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

int potentiometer_init(unsigned int adc_max_val, float output_max_val);

float potentiometer_read();

void potentiometer_deinit();

#endif

