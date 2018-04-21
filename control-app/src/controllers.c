#include <controllers.h>
#include <stdlib.h>

#define MAX_BUFFER 500

static float shaper_buf[MAX_BUFFER];

static float prev_err = 0.0;
static float integral_err = 0.0;

float input_shaper(float des, float vib_freq, float vib_damping_ratio, float sample_freq)
{
  
}

float pid_controller(float des, float feedback, float kp, float ki, float kd, float sample_time, float saturation)
{
	float err = des - feedback;
	float ctrl_out = 0.0;
	integral_err = integral_err + err;

	ctrl_out = kp * err + (ki * integral_err * sample_time) + (kd * ((err - prev_err) / sample_time));

	ctrl_out = (abs(ctrl_out) > abs(saturation)) ? abs(saturation) * (ctrl_out / abs(ctrl_out)) : ctrl_out;

	prev_err = err;

	return ctrl_out;
}

