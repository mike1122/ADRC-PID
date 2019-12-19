#include "main.h"
#include <Windows.h>
using namespace std;

Tracking_Diff  td[3];
Tracking_Diff  tdr[3];


void ADRC::Init()
{
	TrackingDiffInit(td[x], 100);
	TrackingDiffInit(td[y], 1000);
	TrackingDiffInit(td[z], 1000);//for 
	//角加速度输入
	TrackingDiffInit(tdr[x], 90);
	TrackingDiffInit(tdr[y], 90);
	TrackingDiffInit(tdr[z], 90);//for derivative
}

float ADRC::Angular(float dt,float data_)
{
	TrackingDiffCal(td[x], data_, UPDATE_S);
	//printf("%.3F-->%.3f,%.3f\n", data_,td[x].v1, td[x].v2);
	return 0;
}


//跟踪微分器实现
void ADRC::TrackingDiffCal(Tracking_Diff &td, float tar_, float dt)//vt目标
{
	//采样步长s
	td.h = dt;
	td.u  = fhan(td.v1 - tar_, td.v2, td.r, td.h);//vt当前角速度
	//printf("%.3f-->%.3f,%.3f,%.3f\n", vt, fh, td.v1, td.v2);
	//td.noise = (float(50 - rand() % 100) / 100.0);
	td.u = td.u * 0.2 + td.u_*0.8;// +td.noise;

	td.v1 = td.v1 + td.h * td.v2;
	td.v2 = td.v2 + td.h * td.u;
	td.u_ = td.u;
	//printf("%.3f-->%.3f,%.3f,%.3f\n", tar_,fh,td.v1, td.v2);
}


//跟踪微分器参数初始化
void ADRC::TrackingDiffInit(Tracking_Diff &td, float r)
{
	td.h = 0;
	td.r = 0;
	td.v1 = 0;
	td.v2 = 0;
	td.r = r;
}

void ADRC::motion(Tracking_Diff &statie_, float dt)
{

}

float ADRC::fhan(float x1, float x2, float r, float h)
{
	float d;
	float a0;
	float y;
	float b_y;
	float a2;
	float x;
	float b_x;
	float c_x;
	float d_x;
	float b_a0;
	float e_x;
	float f_x;
	float c_a0;

	d = r * (h * h);
	a0 = h * x2;
	y = x1 + a0;
	if (y < 0.0F) {
		b_y = -1.0F;
	}
	else if (y > 0.0F) {
		b_y = 1.0F;
	}
	else if (y == 0.0F) {
		b_y = 0.0F;
	}
	else {
		b_y = y;
	}
	//printf("中间变量：d:%.3f,a0:%.3f,y:%.3f,b_y:%.3f,", d, a0, y, b_y);

	a2 = a0 + b_y * ((float)sqrt(d * (d + 8.0F * (float)fabs(y))) - d) / 2.0F;
	x = y + d;
	b_x = y - d;
	if (x < 0.0F) {
		c_x = -1.0F;
	}
	else if (x > 0.0F) {
		c_x = 1.0F;
	}
	else if (x == 0.0F) {
		c_x = 0.0F;
	}
	else {
		c_x = x;
	}

	if (b_x < 0.0F) {
		d_x = -1.0F;
	}
	else if (b_x > 0.0F) {
		d_x = 1.0F;
	}
	else if (b_x == 0.0F) {
		d_x = 0.0F;
	}
	else {
		d_x = b_x;
	}

	a0 = ((a0 + y) - a2) * ((c_x - d_x) / 2.0F) + a2;
	x = a0 + d;
	b_x = a0 - d;
	if (a0 < 0.0F) {
		b_a0 = -1.0F;
	}
	else if (a0 > 0.0F) {
		b_a0 = 1.0F;
	}
	else if (a0 == 0.0F) {
		b_a0 = 0.0F;
	}
	else {
		b_a0 = a0;
	}

	if (x < 0.0F) {
		e_x = -1.0F;
	}
	else if (x > 0.0F) {
		e_x = 1.0F;
	}
	else if (x == 0.0F) {
		e_x = 0.0F;
	}
	else {
		e_x = x;
	}

	if (b_x < 0.0F) {
		f_x = -1.0F;
	}
	else if (b_x > 0.0F) {
		f_x = 1.0F;
	}
	else if (b_x == 0.0F) {
		f_x = 0.0F;
	}
	else {
		f_x = b_x;
	}

	if (a0 < 0.0F) {
		c_a0 = -1.0F;
	}
	else if (a0 > 0.0F) {
		c_a0 = 1.0F;
	}
	else if (a0 == 0.0F) {
		c_a0 = 0.0F;
	}
	else {
		c_a0 = a0;
	}

	return -r * (a0 / d - b_a0) * ((e_x - f_x) / 2.0F) - r * c_a0;
}