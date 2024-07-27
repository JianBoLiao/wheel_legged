#ifndef __VIRTUAL_MODEL_CONTROL__
#define __VIRTUAL_MODEL_CONTROL__

#include "public.h"
#include "pid.h"
using namespace webots;

extern float g_Tp_l;
extern float g_Tp_r;


//����ͼ
/*
*   E   A
* D       B
*     C
*/

class VIRTUAL_MODEL_CONTROL {
	public:
		VIRTUAL_MODEL_CONTROL(Motor* Robotlegs[4], PositionSensor* LegsEncoder[4], InertialUnit* RobotImu, Robot* RobotRobot);
		/*Ԥ��״̬*/ 
		float Ld[2] = { 0.288, 0.288 };
		float roll_d = { 0 };

		/*ʵ��״̬*/
		float L[2] = { 0.288, 0.288};
		float roll = { 0 };
		Kinematic_Results AnglesMeasured[2] = { 0 };// �ؽڵ������������ֵ
		Point_XY PosMeasured[2] = { 0 };

		/*jacob����*/
		Transformed_Jacob VMCJacob[2] = { 0 };

		/*jacob������*/
		float F[2] = { 0 };
		float Tp[2] = { 0 };

	    /*jacob�����*/
		Kinematic_Results MotorTorque[2] = { 0 };

		/*��������*/
		ClassicPidStructTypedef LegLengthPid;
		float K_roll = 0;// 10;

		/*ˢ��ʵ��״̬*/
		void refresh_measured();

		/*ˢ��Ԥ��״̬*/
		void refresh_expected();

		/*���͹ؽڵ������*/
		void run(float Tp, float offset);


	private:
		Motor* legs[4];
		PositionSensor* Encoder[4];
		InertialUnit* Imu;
		Robot* robot;

		

};

#endif