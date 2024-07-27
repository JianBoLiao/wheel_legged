#include "vmc.h"
using namespace webots;

VMC_ER::VMC_ER(Motor* Robotlegs[4], PositionSensor* RobotEncoder[4], InertialUnit* RobotImu, Robot* RobotRobot) {
	for (int i = 0; i < 4; ++i) {
		legs[i] = Robotlegs[i];
		Encoder[i] = RobotEncoder[i];
	}
	Imu = RobotImu;
	robot = RobotRobot;
}

void VMC_ER::refresh_information() {
	// 更新时间
	time_now = robot->getTime();

	IMU[0] = Imu->getRollPitchYaw()[0];
	IMU[1] = Imu->getRollPitchYaw()[1];
	IMU[2] = Imu->getRollPitchYaw()[2];
	//std::cout << "imu[0] = " << IMU[0] << std::endl;

	AnglesMeasured[0].rad_AB = Encoder[0]->getValue() + 4.0 * PI / 3.0;
	AnglesMeasured[0].rad_ED = - Encoder[1]->getValue() - PI / 3.0;

	AnglesMeasured[1].rad_AB = Encoder[2]->getValue() + 4.0 * PI / 3.0;
	AnglesMeasured[1].rad_ED = -Encoder[3]->getValue() - PI / 3.0;

	/*PosExpect[0].x = 0;
	PosExpect[1].x = 0;*/
	PosExpect[0].y = -SupportHeigh[0] + IMU_k[0] * WHEEL_DISTANCE * sin(IMU[0]);
	PosExpect[1].y = -SupportHeigh[1] - IMU_k[0] * WHEEL_DISTANCE * sin(IMU[0]);

	SpeedExpect[0] = { 0 };
	SpeedExpect[1] = { 0 };

	PosMeasured[0] = Kinematic_Forward(AnglesMeasured[0]);
	PosMeasured[1] = Kinematic_Forward(AnglesMeasured[1]);

	SpeedMeasured[0].x = (PosMeasured[0].x - PosMeasuredPre[0].x) / (time_now - time_pre);
	SpeedMeasured[0].y = (PosMeasured[0].y - PosMeasuredPre[0].y) / (time_now - time_pre);
	SpeedMeasured[1].x = (PosMeasured[1].x - PosMeasuredPre[1].x) / (time_now - time_pre);
	SpeedMeasured[1].y = (PosMeasured[1].y - PosMeasuredPre[1].y) / (time_now - time_pre);

	
	// 参数迭代
	PosMeasuredPre[0] = PosMeasured[0];
	PosMeasuredPre[1] = PosMeasured[1];
	time_pre = time_now;
}

void VMC_ER::run() {
	refresh_information();

	support_force[0].x = Support_kx[0] * (PosExpect[0].x - PosMeasured[0].x) + Support_dx[0] * (SpeedExpect[0].x - SpeedMeasured[0].x);
	support_force[0].y = -0.5 * M * 9.8 + Support_ky[0] * (PosExpect[0].y - PosMeasured[0].y) + Support_dy[0] * (SpeedExpect[0].y - SpeedMeasured[0].y);

	support_force[1].x = Support_kx[1] * (PosExpect[1].x - PosMeasured[1].x) + Support_dx[1] * (SpeedExpect[1].x - SpeedMeasured[1].x);
	support_force[1].y = -0.5 * M * 9.8 + Support_ky[1] * (PosExpect[1].y - PosMeasured[1].y) + Support_dy[1] * (SpeedExpect[1].y - SpeedMeasured[1].y);

	//PosExpect[0].y = SupportHeigh[0];
	//PosExpect[1].y = SupportHeigh[1];

	VMCJacob[0] = TransJacob(AnglesMeasured[0]);
	VMCJacob[1] = TransJacob(AnglesMeasured[1]);

	MotorTorque[0].rad_AB = VMCJacob[0].dx_dq1 * support_force[0].x + VMCJacob[0].dy_dq1 * support_force[0].y;
	MotorTorque[0].rad_ED = VMCJacob[0].dx_dq2 * support_force[0].x + VMCJacob[0].dy_dq2 * support_force[0].y;

	MotorTorque[1].rad_AB = VMCJacob[1].dx_dq1 * support_force[1].x + VMCJacob[1].dy_dq1 * support_force[1].y;
	MotorTorque[1].rad_ED = VMCJacob[1].dx_dq2 * support_force[1].x + VMCJacob[1].dy_dq2 * support_force[1].y;

	
	 
	legs[0]->setTorque(MotorTorque[0].rad_AB);
	legs[1]->setTorque(-MotorTorque[0].rad_ED);
	legs[2]->setTorque(MotorTorque[1].rad_AB);
	legs[3]->setTorque(-MotorTorque[1].rad_ED);

	/*std::cout << "leg1 torque = " << MotorTorque[0].rad_AB << std::endl;
	std::cout << "leg2 torque = " << -MotorTorque[0].rad_ED << std::endl;
	std::cout << "leg3 torque = " << MotorTorque[1].rad_AB << std::endl;
	std::cout << "leg4 torque = " << -MotorTorque[1].rad_ED << std::endl;*/
}

void VMC_ER::run(float Height_offset) {
	refresh_information();
	PosExpect[0].y += Height_offset;
	PosExpect[1].y += Height_offset;

	std::cout << "POSEXPECTED = " << PosExpect[0].y << std::endl;

	support_force[0].x = Support_kx[0] * (PosExpect[0].x - PosMeasured[0].x) + Support_dx[0] * (SpeedExpect[0].x - SpeedMeasured[0].x);
	support_force[0].y = -0.5 * M * 9.8 + Support_ky[0] * (PosExpect[0].y - PosMeasured[0].y) + Support_dy[0] * (SpeedExpect[0].y - SpeedMeasured[0].y);

	support_force[1].x = Support_kx[1] * (PosExpect[1].x - PosMeasured[1].x) + Support_dx[1] * (SpeedExpect[1].x - SpeedMeasured[1].x);
	support_force[1].y = -0.5 * M * 9.8 + Support_ky[1] * (PosExpect[1].y - PosMeasured[1].y) + Support_dy[1] * (SpeedExpect[1].y - SpeedMeasured[1].y);

	//PosExpect[0].y = SupportHeigh[0];
	//PosExpect[1].y = SupportHeigh[1];

	VMCJacob[0] = TransJacob(AnglesMeasured[0]);
	VMCJacob[1] = TransJacob(AnglesMeasured[1]);

	MotorTorque[0].rad_AB = VMCJacob[0].dx_dq1 * support_force[0].x + VMCJacob[0].dy_dq1 * support_force[0].y;
	MotorTorque[0].rad_ED = VMCJacob[0].dx_dq2 * support_force[0].x + VMCJacob[0].dy_dq2 * support_force[0].y;

	MotorTorque[1].rad_AB = VMCJacob[1].dx_dq1 * support_force[1].x + VMCJacob[1].dy_dq1 * support_force[1].y;
	MotorTorque[1].rad_ED = VMCJacob[1].dx_dq2 * support_force[1].x + VMCJacob[1].dy_dq2 * support_force[1].y;



	legs[0]->setTorque(MotorTorque[0].rad_AB);
	legs[1]->setTorque(-MotorTorque[0].rad_ED);
	legs[2]->setTorque(MotorTorque[1].rad_AB);
	legs[3]->setTorque(-MotorTorque[1].rad_ED);
}