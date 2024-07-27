#ifndef __VIRTUAL_MODEL_CONTROL__
#define __VIRTUAL_MODEL_CONTROL__

#include "public.h"
#include "pid.h"
using namespace webots;

extern float g_Tp_l;
extern float g_Tp_r;


//右视图
/*
*   E   A
* D       B
*     C
*/

class VIRTUAL_MODEL_CONTROL {
	public:
		VIRTUAL_MODEL_CONTROL(Motor* Robotlegs[4], PositionSensor* LegsEncoder[4], InertialUnit* RobotImu, Robot* RobotRobot);
		/*预期状态*/ 
		float Ld[2] = { 0.288, 0.288 };
		float roll_d = { 0 };

		/*实际状态*/
		float L[2] = { 0.288, 0.288};
		float roll = { 0 };
		Kinematic_Results AnglesMeasured[2] = { 0 };// 关节电机编码器测量值
		Point_XY PosMeasured[2] = { 0 };

		/*jacob矩阵*/
		Transformed_Jacob VMCJacob[2] = { 0 };

		/*jacob输入量*/
		float F[2] = { 0 };
		float Tp[2] = { 0 };

	    /*jacob输出量*/
		Kinematic_Results MotorTorque[2] = { 0 };

		/*参数整定*/
		ClassicPidStructTypedef LegLengthPid;
		float K_roll = 0;// 10;

		/*刷新实际状态*/
		void refresh_measured();

		/*刷新预期状态*/
		void refresh_expected();

		/*发送关节电机力矩*/
		void run(float Tp, float offset);


	private:
		Motor* legs[4];
		PositionSensor* Encoder[4];
		InertialUnit* Imu;
		Robot* robot;

		

};

#endif