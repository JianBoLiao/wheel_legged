#ifndef __LQR_CALCULATE__
#define __LQR_CALCULATE__

#include "public.h"
#include "pid.h"

extern float g_Tp_l;
extern float g_Tp_r;

class LQR_CALCULATE {
	public:
		LQR_CALCULATE(Motor* Robotwheels[2], PositionSensor* WheelsEncoder[2], PositionSensor* LegsEncoder[4], InertialUnit* RobotImu, Robot* RobotRobot);
        
		/* 反馈矩阵参数系数 */
float K_params[2][6][4] = 
{
	{
		{892.118732, -542.323587, 152.452601, 5.173959}, 
		{20.164422, -13.969725, 17.999176, 0.070830}, 
		{24.851856, -12.800111, 2.306963, 1.430398}, 
		{45.851750, -24.800832, 5.931883, 2.690405}, 
		{256.837607, -150.675752, 34.460945, -4.069363}, 
		{91.350536, -51.567955, 10.970152, -1.176191}
	}, 
{
		{177.606102, -119.263036, 32.515498, -6.978679}, 
		{9.016812, -5.945830, 2.052526, -0.545791}, 
		{168.475377, -94.381805, 19.729516, -1.738013}, 
		{309.407422, -173.289887, 36.256970, -3.180786}, 
		{-267.518151, 142.703983, -27.439719, -4.190842}, 
		{-73.829820, 38.707554, -7.222087, -2.756804}
	}
};
		/* 反馈矩阵参数 */
		float K[2][6] = { 0 };
		/*腿长*/
		float L_left = 0;
		float L_right = 0;
		float L = 0;

		/* 状态向量测量值 */
		float displacement_now = 0;
		float displacement_pre = 0;

		float speed = 0;

		float pitch_now = 0;
		float pitch_pre = 0;
		float pitch_speed = 0;
		float theta_now[2] = { 0 };
		float theta_pre[2] = { 0 };
		float theta_avg = 0;

		float theta_speed[2] = { 0 };
		float theta_speed_avg = 0;

		/* 状态向量期望值 */
        // 腿倾角的正方向为顺时针，零点为y轴正方向
		float theta_expected = 0;         // theta
		float theta_speed_expected = 0;   // dot_theta
		float displacement_expected = 0;  // x
		float speed_expected = 0;         // dot_x
        // 俯仰角的正方向为逆时针，零点为x轴正方向
		float pitch_expected = 0;         // phi
		float pitch_speed_expected = 0;   // dot_phi

		/* 计算控制向量 */
		float T = 0;
		float Tp = 0;

		float T_l = 0;
		float T_r = 0;

		float Tp_l = 0;
		float Tp_r = 0;

		/* 辅助变量 */
		float time_pre = 0;
		float time_now = 0;
		float time_interval = 0;
		
		/* 转向环 */
		float yaw_now = 0;
		float yaw_pre = 0;
		float yaw = 0;
		float yaw_last = 0;
		float yaw_speed = 0;

		float yaw_expected = 0;

		float Tp_offset = 0;
		float T_offset = 0;

		/* 计算body与leg夹角 */
		Kinematic_Results AnglesMeasured[2] = { 0 };// 关节电机编码器测量值
		Point_XY PosMeasured[2] = { 0 };
		float angle[2];

		/* 刷新实际状态 */
        void refresh_measured();

        /* 刷新预期状态 */
		void refresh_expected();

		/* 计算输出向量 */
		void run(float Speed, float YawExpected);

		/* PD控制器 */
		ClassicPidStructTypedef TpPid;
		ClassicPidStructTypedef TPid;
		

	private:
		Motor* wheels[2];
		PositionSensor* LegsEncoder[4];
		PositionSensor* WheelsEncoder[2];
		InertialUnit* Imu;
		Robot* robot;
		void get_Yaw();
};

	#endif
