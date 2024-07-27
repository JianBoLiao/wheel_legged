#ifndef __PID__
#define __PID__

#include "public.h"

typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float LimitOutput;
	float LimitIntegral;
	float Integral;
	float PreError;

	int SectionFlag;
	float KpMax;
	float ErrorLine;
}ClassicPidStructTypedef;

float ClassicPidRegulate(float Reference, float PresentFeedback, ClassicPidStructTypedef* PID_Struct);

#endif
