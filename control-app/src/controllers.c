#include <controllers.h>
#include <math.h>
#include <stdlib.h>

#define MAX_BUFFER 1000000

static float kp = 0.0;
static float ki = 0.0;
static float kd = 0.0;
static float sample_time = 1;
static float saturation = 0.0;
static float prev_err = 0.0;
static float integral_err = 0.0;

static float shaper_buf[MAX_BUFFER] = {0};
static float shaper_buf1[MAX_BUFFER] = {0};
static unsigned int delay_idx = 0;
static float t0_amp;
static float t1_amp;
static float t2_amp;

// Set the parameters for the input shaper
void set_input_shaper_params(float vib_freq, float vib_damping_ratio, float sample_time)
{
	float K;

	delay_idx = (unsigned int)lroundf(((1 / (vib_freq * sqrt(1 - pow(vib_damping_ratio, 2)))) / 2) / sample_time);

	K = exp(-(vib_damping_ratio * M_PI) / sqrt(1 - pow(vib_damping_ratio, 2)));

	t0_amp = 1 / (1 + 2*K + pow(K, 2));
	t1_amp = 2*K / (1 + 2*K + pow(K, 2));
	t2_amp = pow(K, 2) / (1 + 2*K + pow(K, 2));
}

// Counteract the vibrations of the rod
float input_shaper(float des)
{
	float out = (t0_amp * des) + (t1_amp * shaper_buf[0]) + (t2_amp * shaper_buf1[0]);

	for(unsigned int i = 0; i < delay_idx; i++)
	{
		shaper_buf[i] = shaper_buf[i+1];	
	}

	for(unsigned int i = 0; i < 2*delay_idx; i++)
	{
		shaper_buf1[i] = shaper_buf1[i + 1];
	}

	shaper_buf[delay_idx] = des;
	shaper_buf1[2*delay_idx] = des;

	return out;		
}

// Set the parameters for the PID controller
void set_pid_params(float kp_val, float ki_val, float kd_val, float sample_time_val, float saturation_val)
{
	kp = kp_val;
	ki = ki_val;
	kd = kd_val;
	sample_time = sample_time_val;
	saturation = saturation_val;
}

// Calculate the error between the desired and actual positon of the motor and correct
float pid_controller(float des, float feedback)
{
	float err = des - feedback;
	float ctrl_out = 0.0;
	float sign = 1;
	integral_err = integral_err + err;

	ctrl_out = kp * err + (ki * integral_err * sample_time) + (kd * ((err - prev_err) / sample_time));

	sign = (ctrl_out >= 0.0f) ? 1.0f : -1.0f;

	ctrl_out = (abs(ctrl_out) > abs(saturation)) ? abs(saturation) * sign : ctrl_out;

	prev_err = err;

	return ctrl_out;
}
