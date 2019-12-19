#include "main.h"
#include <Windows.h>
using namespace std;

PID::PID(float init_p,float initial_p, float initial_i, float initial_d, float initial_imax, float dt):
kp_out(init_p),
kp_(initial_p),
ki_(initial_i),
kd_(initial_d),
i_max(initial_imax),
DT(dt)
{
	input_flag = false;
	low_pass = 1.0;
	_integrator = 0.0;
}

void PID::set_input(float input)
{
	if (false == input_flag)
	{
		_input = input;
		input_flag = true;
		return;
	}

	float input_filt_change = low_pass * (input - _input);
	_input = _input + input_filt_change;

	_derivative = input_filt_change;
}

float PID::get_pid() 
{
	_integrator += _input * ki_ * DT;
	if (_integrator > i_max)
		_integrator = i_max;
	else if(_integrator < -i_max)
		_integrator = -i_max;
	float output = _input * kp_ + _integrator + _derivative * kd_;


	if (output < -ACC_LIMIT)
		output = -ACC_LIMIT;
	else if (output > ACC_LIMIT)
		output = ACC_LIMIT;

	return output;
}

float PID::get_p(float target ,float data_) 
{
	return kp_out * (target - data_);
}

void PID::update_status(PID_STATUS &sta_,float tar_pos )
{
	sta_._v2 = get_p(tar_pos, sta_.v1);
	set_input(sta_._v2 - sta_.v2);
	sta_.noise = (float(50 - rand() % 100) / 100.0);
	sta_.u = get_pid() + sta_.noise;

	sta_.v1 = sta_.v1 + sta_.v2 *DT;
	sta_.v2 = sta_.v2 + sta_.u *DT;
	//printf("%.3f     %.3f      %.3f\n", sta_.v1, sta_.v2, sta_.u);
	//sta_.u = get_pid();
}