#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

/** 
 * Initialize potentiometer setting the maximum possible adc value, and the maximum angle returned from the read function
 * @param adc_max_val The maximum value out of the ADC
 * @param output_max_val The maximum scaled output of the potentiometer read function
 * @return The operation status
 */
int potentiometer_init(unsigned int adc_max_val, float output_max_val);

/**
 * Read the current angle of the potentiometer
 * @return The output angle scaled by the output maximum given
 */
float potentiometer_read();

/**
 * De initialize the potentiometer
 */
void potentiometer_deinit();

#endif

