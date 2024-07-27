#include "pid.h"


float ClassicPidRegulate(float Reference, float PresentFeedback, ClassicPidStructTypedef* PID_Struct) {

	float error;
	float error_inc;
	float pTerm;
	float iTerm;
	float dTerm;
	float dwAux;
	float output;
	/*error computation*/
	error = Reference - PresentFeedback;

	/*proportional term computation*/

	if (PID_Struct->SectionFlag == 1)
	{
		if (fabs(error) >= PID_Struct->ErrorLine)
			pTerm = error * PID_Struct->KpMax;
		else
			pTerm = error * PID_Struct->Kp;
	}
	else
		pTerm = error * PID_Struct->Kp;

	/*Integral term computation*/

	//iTerm = ( fabs(error) <  Motor_1.SpeedExpected/5 )  ? error * PID_Struct->Ki : error * 0.001f ;
	iTerm = error * PID_Struct->Ki;


	dwAux = PID_Struct->Integral + iTerm;
	/*limit integral*/
	if (dwAux > PID_Struct->LimitIntegral)
	{
		PID_Struct->Integral = PID_Struct->LimitIntegral;
	}
	else if (dwAux < -1 * PID_Struct->LimitIntegral)
	{
		PID_Struct->Integral = -1 * PID_Struct->LimitIntegral;
	}
	else
	{
		PID_Struct->Integral = dwAux;
	}
	/*differential term computation*/

	error_inc = error - PID_Struct->PreError;
	dTerm = error_inc * PID_Struct->Kd;
	PID_Struct->PreError = error;

	output = pTerm + PID_Struct->Integral + dTerm;

	/*limit output*/
	if (output >= PID_Struct->LimitOutput)
	{
		return (PID_Struct->LimitOutput);
	}
	else if (output < -1.0f * PID_Struct->LimitOutput)
	{
		return (-1.0f * PID_Struct->LimitOutput);
	}
	else
	{
		return output;
	}
}