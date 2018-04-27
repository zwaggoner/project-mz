#ifndef CONTROLLERS_H
#define CONTROLLERS_H

/** 
 * Function to set the initial input shaper parmeters
 * @param vib_frequency The vibration frequency to shape for
 * @param vib_damping_ratio The damping ratio of the frequency to shape for
 * @param sample_time The sampling time of the control loop that the input shaper loop is controled in
 */
void set_input_shaper_params(float vib_freq, float vib_damping_ratio, float sample_time);

/**
 * Input shape the value by the set parameters
 * @return The input shaped signal
 */
float input_shaper(float des);

/**
 * Set the PID controller paramters
 * @param kp_val The proportional gain
 * @param ki_val The integral gain
 * @param kd_val The derivative gain
 * @param sample_time_val The sample time of the control loop used for the PID controller
 * @param saturation_val The saturation value of the PID controller 
 */
void set_pid_params(float kp_val, float ki_val, float kd_val, float sample_time_val, float saturation_val);

/**
 * Determint the PID controller output for the desired and feedback value given the set parameters
 * @param des The desired output
 * @param feedback The current state of the feedback variable
 * @return The controller output
 */
float pid_controller(float des, float feedback); 
#endif
