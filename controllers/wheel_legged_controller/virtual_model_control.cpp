#include "virtual_model_control.h"
#include "lqr_calculate.h"

VIRTUAL_MODEL_CONTROL::VIRTUAL_MODEL_CONTROL(Motor* Robotlegs[4], PositionSensor* LegsEncoder[4], InertialUnit* RobotImu, Robot* RobotRobot) {
	for (int i = 0; i < 4; ++i) {
		legs[i] = Robotlegs[i];
		Encoder[i] = LegsEncoder[i];
	}
	Imu = RobotImu;
	robot = RobotRobot;

	LegLengthPid.Kp = 50000.0;
	LegLengthPid.Ki = 50000.0;
	LegLengthPid.Kd = 100000.0;
	LegLengthPid.LimitOutput = 150;
	LegLengthPid.LimitIntegral = 0;
	LegLengthPid.Integral = 0;
	LegLengthPid.PreError = 0;
	LegLengthPid.SectionFlag = 0;
}

void VIRTUAL_MODEL_CONTROL::refresh_measured() {
	roll = Imu->getRollPitchYaw()[0];

	// AnglesMeasured[0].rad_AB = Encoder[0]->getValue() + 4.0 * PI / 3.0;
	// AnglesMeasured[0].rad_ED = -Encoder[1]->getValue() - PI / 3.0;
	// AnglesMeasured[1].rad_AB = Encoder[2]->getValue() + 4.0 * PI / 3.0;
	// AnglesMeasured[1].rad_ED = -Encoder[3]->getValue() - PI / 3.0;

    // 数学角度读取
    AnglesMeasured[0].rad_AB = - Encoder[0]->getValue() + PI * 5.0 / 3.0;
    AnglesMeasured[0].rad_ED = Encoder[1]->getValue() + PI * 4.0 / 3.0;
    AnglesMeasured[1].rad_AB = - Encoder[2]->getValue() + PI * 5.0 / 3.0;
    AnglesMeasured[1].rad_ED = Encoder[3]->getValue() + PI * 4.0 / 3.0;

	PosMeasured[0] = Kinematic_Forward(AnglesMeasured[0]);
	PosMeasured[1] = Kinematic_Forward(AnglesMeasured[1]);

	L[0] = Points_Distance(PosMeasured[0].x, PosMeasured[0].y, 0, 0);
	L[1] = Points_Distance(PosMeasured[1].x, PosMeasured[1].y, 0, 0);

}

void VIRTUAL_MODEL_CONTROL::refresh_expected() {

}

void VIRTUAL_MODEL_CONTROL::run(float T, float offset) {
	/* 刷新测量状态值 */
	refresh_measured();
	/* 刷新预期状态值 */
	refresh_expected();
	/* 刷新jacob矩阵 */
    // 打印正解计算的腿部末端坐标
    // std::cout << "================= PosMeasured =================" << std::endl;
    // std::cout << "PosMeasured[0].x = " << PosMeasured[0].x << std::endl;
    // std::cout << "PosMeasured[0].y = " << PosMeasured[0].y << std::endl;
    // std::cout << "PosMeasured[1].x = " << PosMeasured[1].x << std::endl;
    // std::cout << "PosMeasured[1].y = " << PosMeasured[1].y << std::endl;
    // std::cout << "================= PosMeasured =================" << std::endl;

	VMCJacob[0] = TransJacob_new(AnglesMeasured[0]);
	VMCJacob[1] = TransJacob_new(AnglesMeasured[1]);
	/* 计算jacob输入量 */
    // 使用PID跟踪期望腿长，不需要再使用重力补偿项做前馈
    // F0为左腿的虚拟力，当当前横滚角大于期望值时，机身左高右低，左腿需要向上施加力，F0为正
    // 且此时roll_d - roll 小于0 因此应使用减号
	F[0] =  - ClassicPidRegulate(Ld[0], L[0], &LegLengthPid) - K_roll * (roll_d - roll);//力的方向朝下
	//F[0] = -ClassicPidRegulate(Ld[0], L[0], &LegLengthPid);
	// 同理，机身左高右低时，roll_d - roll 小于0，右腿的虚拟力应该向下增大，因此用加号
	F[1] =  - ClassicPidRegulate(Ld[1], L[1], &LegLengthPid) + K_roll * (roll_d - roll);
	//F[1] = -ClassicPidRegulate(Ld[0], L[1], &LegLengthPid);
	Tp[0] = T - offset;
	Tp[1] = T + offset;
	// std::cout << "g_Tp_l = " << Tp[0] << std::endl;
	/* 计算Jacob输出量 */
	MotorTorque[0].rad_AB = VMCJacob[0].dx_dq1 * F[0] + VMCJacob[0].dy_dq1 * Tp[0];
	MotorTorque[0].rad_ED = VMCJacob[0].dx_dq2 * F[0] + VMCJacob[0].dy_dq2 * Tp[0];
	// std::cout << "motor0.ab" << - MotorTorque[0].rad_AB << std::endl;
	// std::cout << "motor0.ed" << MotorTorque[0].rad_ED << std::endl;
	MotorTorque[1].rad_AB = VMCJacob[1].dx_dq1 * F[1] + VMCJacob[1].dy_dq1 * Tp[1];
	MotorTorque[1].rad_ED = VMCJacob[1].dx_dq2 * F[1] + VMCJacob[1].dy_dq2 * Tp[1];
	/* 发送关节电机力矩 */
	legs[0]->setTorque(MotorTorque[0].rad_AB);
	legs[1]->setTorque(-MotorTorque[0].rad_ED);
	legs[2]->setTorque(MotorTorque[1].rad_AB);
	legs[3]->setTorque(-MotorTorque[1].rad_ED);

}