#ifndef CONTROLLERS_H
#define CONTROLLERS_H

float input_shaper(float des, float vib_freq, float vib_damping_ratio, float sample_freq);

float pid_controller(float des, float feedback, float kp, float ki, float kd, float sample_time, float saturation); 
#endif
