#include "lqr.h"

LQR_ER::LQR_ER(Motor* Robotwheels[2], PositionSensor* RobotEncoder[2], InertialUnit* RobotImu, Robot* RobotRobot) {
	wheels[0] = Robotwheels[0];
	wheels[1] = Robotwheels[1];

	Encoder[0] = RobotEncoder[0];
	Encoder[1] = RobotEncoder[1];

	Imu = RobotImu;

	robot = RobotRobot;

 /*   LQR_K1 = -2.2361;
    LQR_K2 = -7.6005;
    LQR_K3 = -34.1447;
    LQR_K4 = -8.2225;
	LQR_K15 = 0.0224;
	LQR_K25 = -LQR_K15;
	LQR_K16 = 0.0551;
	LQR_K26 = -LQR_K16;*/

	//LQR_K1 = 0;// -2.2361;
	//LQR_K2 = -7.6450;
	//LQR_K3 = -35.2861;
	//LQR_K4 = -8.8004;
	//LQR_K15 = 2.2361;
	//LQR_K25 = -LQR_K15;
	//LQR_K16 = 2.3030;
	//LQR_K26 = -LQR_K16;

	LQR_K1 = -2.2361;
	LQR_K2 = -7.6450;
	LQR_K3 = -35.2861;
	LQR_K4 = -8.8004;
	LQR_K15 = 30.2361;
	LQR_K25 = -LQR_K15;
	LQR_K16 = 2.3030;
	LQR_K26 = -LQR_K16;
	
}
void LQR_ER::get_Yaw() {
	yaw = Imu->getRollPitchYaw()[2];
	double Distance = yaw - yaw_last;
	if (fabs(Distance) > 3.0) {
		Distance = Distance - Distance / fabs(Distance) * 2 * PI;
	}
	yaw_now = yaw_now + Distance;
	yaw_last = yaw;
}
void LQR_ER::refresh_information(float SpeedExpected) {
	time_now = robot->getTime();

	displacement_now = (Encoder[0]->getValue() + Encoder[1]->getValue()) * 0.5 * WHEEL_R;
	displacement_Expected += SpeedExpected * (time_now - time_pre);
	speed = (displacement_now - displacement_pre) / (time_now - time_pre);
	//std::cout << "speed = " << speed << std::endl;

	pitch_now = Imu->getRollPitchYaw()[1];
	pitch_speed = (pitch_now - pitch_pre) / (time_now - time_pre);

	get_Yaw();
	yaw_speed = (yaw_now - yaw_pre) / (time_now - time_pre);

	time_pre = time_now;
	displacement_pre = displacement_now;
	pitch_pre = pitch_now;
	yaw_pre = yaw_now;

}
void LQR_ER::lqr_calculate(float SpeedExpected, float YawExpected) {
	refresh_information(SpeedExpected);
	TorqueExpected[0] = -(-LQR_K1 * (displacement_Expected - displacement_now) - LQR_K2 * (SpeedExpected - speed) + LQR_K3 * (0 - pitch_now) +
		LQR_K4 * (0 - pitch_speed) + LQR_K15 * (YawExpected - yaw_now) + LQR_K16 * (0 - yaw_speed));

	TorqueExpected[1] = -(-LQR_K1 * (displacement_Expected - displacement_now) - LQR_K2 * (SpeedExpected - speed) + LQR_K3 * (0 - pitch_now) +
		LQR_K4 * (0 - pitch_speed) + LQR_K25 * (YawExpected - yaw_now) + LQR_K26 * (0 - yaw_speed));
	
	wheels[0]->setTorque(TorqueExpected[0]);
	wheels[1]->setTorque(TorqueExpected[1]);

}