#include "main.h"
#include <stdio.h>
#include "string.h"
#include "data.h"

Tracking_Diff status;
PID_STATUS pid_status;
FILE *fp;

int main(void)
{
	ADRC adrc_;
	PID pid_(INIT_P_OUT,INIT_P,INIT_I,INIT_D,INIT_I_MAX, UPDATE_S);

	float *data = NULL;
	adrc_.Init();
	data = x1;

	//³õÊ¼×´Ì¬
	status.v1 = 10.0;
	status.v2 = -3.0;
	status.h = 1000;
	status.r = ACC_LIMIT;

	pid_status.v1 = 10.0;
	pid_status.v2 = -3.0;


	fp = fopen("ADRC3.csv", "w");
	if (fp == NULL)
	{
		printf("open file error!\n");
		return 0;
	}
	fprintf(fp, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n", "num", "tar","contr", "pos", "vel","noise","pid_pos","pid_vel","pid_out","pid_in","noise");


	int data_len = sizeof(x1) / sizeof(float);
	int num = 0;

	for (float i = 0; i < 3; i=i+0.001)
	{
		num++;
		float target = 5*sin(i*3.1415/2);
		//target = 0;
		pid_.update_status(pid_status, target);
		adrc_.TrackingDiffCal(status, target, UPDATE_S);
		fprintf(fp, "%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", num,target,status.u,status.v1, status.v2, status.noise,\
			pid_status.v1, pid_status.v2,pid_status._v2, pid_status.u,pid_status.noise);
	}
	fclose(fp);
	printf("done!\n");
	return 0;

	for (int i = 1; i < data_len-1; i++)
	{
		printf("%.3f--->%.3f         ",data[i], (data[i]- data[i-1])/0.04);
		adrc_.Angular(0.02, data[i]);
	}
	return 0;
}