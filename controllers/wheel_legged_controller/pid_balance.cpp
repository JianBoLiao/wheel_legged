#include "pid_balance.h"
using namespace webots;

PID_BALANCE::PID_BALANCE(Motor* Robotwheels[2], PositionSensor* RobotEncoder[2], InertialUnit* RobotImu, Robot* RobotRobot){
	wheels[0] = Robotwheels[0];
	wheels[1] = Robotwheels[1];

	Encoder[0] = RobotEncoder[0];
	Encoder[1] = RobotEncoder[1];

	Imu = RobotImu;

	robot = RobotRobot;

	// 串级pid初始化

	SpeedPid.Kp = 0.0005;// 0.001;//0.0005;
	SpeedPid.Ki = 0.000025;// 0.000025;
	SpeedPid.Kd = 0.0;
	SpeedPid.LimitOutput = 10.0;
	SpeedPid.LimitIntegral = 10.0;
	SpeedPid.Integral = 0;
	SpeedPid.PreError = 0;
	SpeedPid.SectionFlag = 0;

	AnglePid.Kp = 50.0;
	AnglePid.Ki = 0.0;
	AnglePid.Kd = 1000.0;
	AnglePid.LimitOutput = 150;
	AnglePid.LimitIntegral = 0;
	AnglePid.Integral = 0;
	AnglePid.PreError = 0;
	AnglePid.SectionFlag = 0;

	TurnPid.Kp = 10;
	TurnPid.Ki = 0.0;
	TurnPid.Kd = 200;
	TurnPid.LimitOutput = 50;
	TurnPid.LimitIntegral = 0;
	TurnPid.Integral = 0;
	TurnPid.PreError = 0;
	TurnPid.SectionFlag = 0;
}
void PID_BALANCE::get_Yaw() {
	YawNow = Imu->getRollPitchYaw()[2];
	double Distance = YawNow - YawPre;
	if (fabs(Distance) > 3.0) {
		Distance = Distance - Distance / fabs(Distance) * 2 * PI;
	}
	Yaw = Yaw + Distance;
	YawPre = YawNow;
}
void PID_BALANCE::run(float SpeedExpected) {
	float BalanceSpeedFeedback;
	float BalanceAngleExpected;
	float BalanceAngleFeedBack;
	float BalanceTorqueExpected;

	// 计算速度
	encoder_now[0] = Encoder[0]->getValue();
	encoder_now[1] = Encoder[1]->getValue();

	time_now = robot->getTime();
	encoder_speed[0] = (encoder_now[0] - encoder_pre[0]) / (time_now - time_pre);
	encoder_speed[1] = (encoder_now[1] - encoder_pre[1]) / (time_now - time_pre);

	// 更新编码器数值与时间
	encoder_pre[0] = encoder_now[0];
	encoder_pre[1] = encoder_now[1];

	time_pre = time_now;

	//std::cout << "encoder_speed" << encoder_speed[0] << "  " << encoder_speed[1] << std::endl;
	//前进时，2轮反转
	BalanceSpeedFeedback = (encoder_speed[0] + encoder_speed[1]) / 2.0 * encoder2wheel;

	BalanceAngleExpected = ClassicPidRegulate(SpeedExpected, BalanceSpeedFeedback, &SpeedPid);
	//std::cout << "angle expected = " << BalanceAngleExpected << std::endl;
	//BalanceAngleExpected = 0;
	BalanceAngleFeedBack = Imu->getRollPitchYaw()[1];
	//std::cout << "pitch = " << BalanceAngleFeedBack << std::endl;
	
	BalanceTorqueExpected = ClassicPidRegulate(-BalanceAngleExpected, BalanceAngleFeedBack, &AnglePid);
	
	
	wheels[0]->setTorque(BalanceTorqueExpected);
	wheels[1]->setTorque(BalanceTorqueExpected);
	/*wheels[0]->setVelocity(-10);
	wheels[1]->setVelocity(-10);*/
}
void PID_BALANCE::run(float SpeedExpected, float YawExpected) {
	
	// 更新多圈累计yaw
	get_Yaw();
	// 计算速度
	encoder_now[0] = Encoder[0]->getValue();
	encoder_now[1] = Encoder[1]->getValue();

	time_now = robot->getTime();
	encoder_speed[0] = (encoder_now[0] - encoder_pre[0]) / (time_now - time_pre);
	encoder_speed[1] = (encoder_now[1] - encoder_pre[1]) / (time_now - time_pre);

	// 更新编码器数值与时间
	encoder_pre[0] = encoder_now[0];
	encoder_pre[1] = encoder_now[1];

	time_pre = time_now;

	//std::cout << "encoder_speed" << encoder_speed[0] << "  " << encoder_speed[1] << std::endl;
	//前进时，2轮反转
	BalanceSpeedFeedback = (encoder_speed[0] + encoder_speed[1]) / 2.0 * encoder2wheel;

	BalanceAngleExpected = ClassicPidRegulate(SpeedExpected, BalanceSpeedFeedback, &SpeedPid);
	//std::cout << "angle expected = " << BalanceAngleExpected << std::endl;
	//BalanceAngleExpected = 0;
	BalanceAngleFeedBack = Imu->getRollPitchYaw()[1];
	//std::cout << "pitch = " << BalanceAngleFeedBack << std::endl;

	BalanceTorqueExpected = ClassicPidRegulate(-BalanceAngleExpected, BalanceAngleFeedBack, &AnglePid);

	BalanceTurnExpected = ClassicPidRegulate(YawExpected, Yaw, &TurnPid);

	wheels[0]->setTorque(BalanceTorqueExpected - BalanceTurnExpected);
	wheels[1]->setTorque(BalanceTorqueExpected + BalanceTurnExpected);
}