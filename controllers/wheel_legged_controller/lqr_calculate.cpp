#include "lqr_calculate.h"

/* 全局变量 */
float g_Tp_l = 0;
float g_Tp_r = 0;

LQR_CALCULATE::LQR_CALCULATE(Motor* Robotwheels[2], PositionSensor* WheelsEncoder[2], PositionSensor* LegsEncoder[4], InertialUnit* RobotImu, Robot* RobotRobot) {
	for (int i = 0; i < 4; ++i) {
		this->LegsEncoder[i] = LegsEncoder[i];
	}
	for (int i = 0; i < 2; ++i) {
		this->WheelsEncoder[i] = WheelsEncoder[i];
		wheels[i] = Robotwheels[i]; 
	}
	Imu = RobotImu;
	robot = RobotRobot;

	TpPid.Kp = 300.0;
	TpPid.Kd = 300;// 1.0;
	TpPid.Ki = 0.0;
	TpPid.LimitOutput = 50;
	TpPid.LimitIntegral = 0;
	TpPid.Integral = 0;
	TpPid.PreError = 0;
	TpPid.SectionFlag = 0;

	TPid.Kp = 10.0;
	TPid.Ki = 0.0;
	TPid.Kd = 5;      // 0.0;
	TPid.LimitOutput = 50;
	TPid.LimitIntegral = 0;
	TPid.Integral = 0;
	TPid.PreError = 0;
	TPid.SectionFlag = 0;
}
void LQR_CALCULATE::get_Yaw() {
	yaw = Imu->getRollPitchYaw()[2];
	double Distance = yaw - yaw_last;
	if (fabs(Distance) > 3.0) {
		Distance = Distance - Distance / fabs(Distance) * 2 * PI;
	}
	yaw_now = yaw_now + Distance;
	yaw_last = yaw;
}

void LQR_CALCULATE::refresh_measured() {
	time_now = robot->getTime();
	time_interval = time_now - time_pre;

	displacement_now = (WheelsEncoder[0]->getValue() + WheelsEncoder[1]->getValue()) * 0.5 * WHEEL_R;
	
	/* 计算预期的位移 */
	displacement_expected += speed_expected * time_interval;
	
	speed = (displacement_now - displacement_pre) / time_interval;

	pitch_now = Imu->getRollPitchYaw()[1];

	pitch_speed = (pitch_now - pitch_pre) / time_interval;

	// //  /* 计算leg与body的夹角 */
	// // AnglesMeasured[0].rad_AB = LegsEncoder[0]->getValue() + 4.0 * PI / 3.0;
	// // AnglesMeasured[0].rad_ED = -LegsEncoder[1]->getValue() - PI / 3.0;
	// // AnglesMeasured[1].rad_AB = LegsEncoder[2]->getValue() + 4.0 * PI / 3.0;
	// // AnglesMeasured[1].rad_ED = -LegsEncoder[3]->getValue() - PI / 3.0;

    // 数学角度读取
    AnglesMeasured[0].rad_AB = - LegsEncoder[0]->getValue() + PI * 5.0 / 3.0;
    AnglesMeasured[0].rad_ED = LegsEncoder[1]->getValue() + PI * 4.0 / 3.0;
    AnglesMeasured[1].rad_AB = - LegsEncoder[2]->getValue() + PI * 5.0 / 3.0;
    AnglesMeasured[1].rad_ED = LegsEncoder[3]->getValue() + PI * 4.0 / 3.0;

	PosMeasured[0] = Kinematic_Forward(AnglesMeasured[0]);
	PosMeasured[1] = Kinematic_Forward(AnglesMeasured[1]);

    // // 由于观测系方向问题，暂时不该角度读取等其他参数，直接将运动学正解的x方向取反
    // PosMeasured[0].x = -PosMeasured[0].x;
    // PosMeasured[1].x = -PosMeasured[1].x;
    
	// atan2返回值：[-PI, PI]
    angle[0] = atan2(PosMeasured[0].y, PosMeasured[0].x);
	angle[1] = atan2(PosMeasured[1].y, PosMeasured[1].x);

    // 将角度转换到[0, 2PI]
    for (int i = 0; i < 2; i++){
        if (angle[i] < 0){
            angle[i] += 2 * PI;
        }
    }

    // theta_now[0] = -angle[0] - pitch_now;
	// theta_now[1] = -angle[1] - pitch_now;
	// theta_avg = (theta_now[0] + theta_now[1]) / 2.0;
    theta_now[0] = angle[0] + pitch_now;
    theta_now[1] = angle[1] + pitch_now;
    theta_now[0] = 1.5 * PI -theta_now[0]; 
    theta_now[1] = 1.5 * PI -theta_now[1];
    theta_avg = (theta_now[0] + theta_now[1]) / 2.0;

	theta_speed[0] = (theta_now[0] - theta_pre[0]) / time_interval;
	theta_speed[1] = (theta_now[1] - theta_pre[1]) / time_interval;
	theta_speed_avg = (theta_speed[0] + theta_speed[1]) / 2.0;

	get_Yaw();
	yaw_speed = (yaw_now - yaw_pre) / time_interval;


	/* 计算反馈矩阵参数 */
	L_left = Points_Distance(PosMeasured[0].x, PosMeasured[0].y, 0, 0);
	L_right = Points_Distance(PosMeasured[1].x, PosMeasured[1].y, 0, 0);
	L = (L_left + L_right) / 2.0;

	for (int i = 0; i < 2; ++i) {
		for (int j = 0; j < 6; ++j) {
			K[i][j] = K_params[i][j][0] * L * L * L + K_params[i][j][1] * L * L + K_params[i][j][2] * L + K_params[i][j][3];
		}
	}

	/* 刷新pre */
	time_pre = time_now;
	displacement_pre = displacement_now;
	pitch_pre = pitch_now;
	theta_pre[0] = theta_now[0];
	theta_pre[1] = theta_now[1];
	yaw_pre = yaw_now;


}

void LQR_CALCULATE::refresh_expected() {

}

void LQR_CALCULATE::run(float Speed = 0, float YawExpected = 0) {
    speed_expected = Speed;
    yaw_expected = YawExpected;
	refresh_measured();
	refresh_expected();
    // 打印正解计算的腿部末端坐标
    // std::cout << "================= PosMeasured =================" << std::endl;
    // std::cout << "PosMeasured[0].x = " << PosMeasured[0].x << std::endl;
    // std::cout << "PosMeasured[0].y = " << PosMeasured[0].y << std::endl;
    // std::cout << "PosMeasured[1].x = " << PosMeasured[1].x << std::endl;
    // std::cout << "PosMeasured[1].y = " << PosMeasured[1].y << std::endl;
    // std::cout << "================= PosMeasured =================" << std::endl;

	T = -(K[0][0] * (theta_expected - theta_avg) + K[0][1] * (theta_speed_expected - theta_speed_avg)
		+ K[0][2] * (displacement_expected - displacement_now) + K[0][3] * (speed_expected - speed)
		+ K[0][4] * (pitch_expected - pitch_now) + K[0][5] * (pitch_speed_expected - pitch_speed));
 

    // Tp理解为摆杆受到的力，方向为z轴右手向顺时针
    // 以pitch角为例，当机身仰起时，error为负值，机身受到的Tp的正方向为逆时针（pitch增加方向--- >正反馈）
	Tp = - (K[1][0] * (theta_expected - theta_avg) + K[1][1] * (theta_speed_expected - theta_speed_avg)
		+ K[1][2] * (displacement_expected - displacement_now) + K[1][3] * (speed_expected - speed)
		+ K[1][4] * (pitch_expected - pitch_now) + K[1][5] * (pitch_speed_expected - pitch_speed));
    
    std::cout<< "K[1][4]  " << K[1][4] << std::endl;
    std::cout<< "pitch_now  " << pitch_now << std::endl;


	T_offset = ClassicPidRegulate(yaw_expected, yaw_now, &TPid);
    // 打印期望力矩Tp
    std::cout << "================= Tp =================" << std::endl;
    std::cout << "Tp = " << Tp << std::endl;
    std::cout << "================= Tp =================" << std::endl;
    
    // 当前值为右腿减左腿，当左腿在前右腿在后时，左腿θ小于零，右腿大于0，
    // 右减左大于0，因此pid的输出为负值
    // 
	Tp_offset = ClassicPidRegulate(0, theta_now[1] - theta_now[0], &TpPid);
    // 打印期望力矩Tp_offset
    // Tp_r正方向 : 使右腿向后摆
	Tp_l = Tp - Tp_offset;
	// Tp_l  = - 25;
	Tp_r = Tp + Tp_offset;
    
    std::cout << "================= Tp_offset =================" << std::endl;
    std::cout << "Tp_offset = " << Tp_offset << std::endl;
    std::cout << "Tp_l = " << Tp_l << std::endl;
    std::cout << "Tp_r = " << Tp_r << std::endl;
    std::cout << "================= Tp_offset =================" << std::endl;

	g_Tp_l = Tp_l;
	g_Tp_r = Tp_r;

	T_l = T - T_offset;
	T_r = T + T_offset;
    
    // 打印期望力
    // std::cout << "================= T =================" << std::endl;
    // std::cout << "T = " << T << std::endl;
    // std::cout << "================= T =================" << std::endl;
	wheels[0]->setTorque(T_l);
	wheels[1]->setTorque(T_r);
}

