#ifndef CONTROLLERS_H
#define CONTROLLERS_H

void set_input_shaper_params(float vib_freq, float vib_damping_ratio, float sample_time);

float input_shaper(float des);

void set_pid_params(float kp_val, float ki_val, float kd_val, float sample_time_val, float saturation_val);

float pid_controller(float des, float feedback); 
#endif
