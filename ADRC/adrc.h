#pragma once
#include "math.h"
#include <stdio.h>

typedef struct
{
	float v1;
	float v2;//ËÙ¶È
	float r;
	float h;
	float u;
	float u_;
	float noise;
}Tracking_Diff;//Tracking  Differentiator ¸ú×ÙÎ¢·ÖÆ÷

enum
{
	x = 0,
	y,
	z,
};

class ADRC
{
public:
	void Init(void);
	//void Angle(float dt);
	float Angular(float dt,float data_);
	void TrackingDiffCal(Tracking_Diff &td, float tar_, float dt);
	void TrackingDiffInit(Tracking_Diff &td, float r);
	float fhan(float x1, float x2, float r, float h);
	void motion(Tracking_Diff &statie_,float dt);

private:
	
};