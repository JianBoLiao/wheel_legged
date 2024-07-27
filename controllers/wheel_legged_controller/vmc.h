#ifndef __VMC__
#define __VMC__


#include "public.h"
using namespace webots;

class VMC_ER {
	public:

		VMC_ER(Motor* Robotlegs[4], PositionSensor* RobotEncoder[4], InertialUnit* RobotImu, Robot* RobotRobot );
		float Support_kx[2] = { 2000,2000, };
		float Support_ky[2] = { 4000,4000 };
		Point_XY PosExpect[2] = { 0 };

		void refresh_information();
		void run();
		void run(float Height_offset);
	private:
		Motor* legs[4];
		PositionSensor* Encoder[4];
		InertialUnit* Imu;
		Robot* robot;
		
		Point_XY PosMeasured[2] = { 0 };
		Point_XY PosMeasuredPre[2] = { 0 };
		Point_XY SpeedMeasured[2] = { 0 };
		Kinematic_Results AnglesMeasured[2] = { 0 };
		Transformed_Jacob VMCJacob[2] = { 0 };
		
		Point_XY SpeedExpect[2] = { 0 };
		Kinematic_Results MotorTorque[2] = { 0 };

		Point_XY support_force[2];

		float M = 12 + 4 * (0.28 + 0.124) + 2 * 3.0;
		float SupportHeigh[2] = { 0.288,0.288 };
		
		float Support_dx[2] = { 100,100, };
		
		float Support_dy[2] = { 300,300 };

		double IMU[3] = { 0 };
		double IMU_Speed[3] = { 0 };
		double IMU_k[3] = { 10, 10, 10 };

		double time_pre = 0.0;
		double time_now = 0.0;




};


#endif