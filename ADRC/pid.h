#pragma once
#include "stdio.h"
#include "main.h"

typedef struct
{
	float v1;//Î»ÖÃ
	float v2;//ËÙ¶È
	float _v2;
	float u;
	float noise;
}PID_STATUS;//pid×´Ì¬¸ú×Ù

class PID
{
public:
	PID(float init_p, float initial_p, float initial_i, float initial_d, float initial_imax, float dt);
	void set_input(float input);
	float get_pid();
	float get_p(float target, float data_);
	void update_status(PID_STATUS &sta_, float tar_pos);
private:
	float kp_out;

	float kp_;
	float ki_;
	float kd_;
	float i_max;

	float _input;
	float low_pass;
	bool input_flag;
	float _integrator;
	float _derivative;
	float DT;
};