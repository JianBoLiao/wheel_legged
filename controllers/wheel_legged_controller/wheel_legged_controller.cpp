// File:          wheel_legged_controller.cpp
// Date:		  2023.11.03
// Description:   
// Author:        HustRobocon
// Modifications:
//#include "wheel_legged_controller.h"
#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Keyboard.hpp>
#include "MPC_controller.h"
#include "wheel_legged_controller.h"

using namespace webots;

int main(int argc, char** argv) {
	const double d = 0.2;
	const double DT = 0.005;
	const double m_0 = 12;
	const double M = 6;
	const double g = 9.8;
	//Eigen::MatrixXd A, B, Q, F, R;
	//A.resize(4, 4);
	//A << 1, 1 * DT, 0,                      0,
	//	 0, 1,      -m_0 * g / M * DT,      0,
	//	 0, 0,      1,                      1 * DT,
	//  	 0, 0,      (1/d + m_0/M) * g * DT, 1;
	////A << 1.0, -m_0 * g / M * DT, 0,
	////	 0, 1.0, 1 * DT,
	////	 0, (1 / d + m_0 / M)* g* DT, 1.0;

	//B.resize(4, 1);
	//B << 0,
	//	 1 / M * DT,
	//	 0,
	//	 -1 / M*d * DT;
	////B << 1 / M * DT,
	////	 0,
	////	 -1 / M*d * DT;

	//Q.resize(4, 4);
	//Q << 2000, 0, 0, 0,
	//	 0, 0, 0, 0,
	//	 0, 0, 0, 0,
	//	 0, 0, 0, 0;
	////Q << 10, 0, 0,
	////	 0, 5, 0,
	////	 0, 0, 0.01;

	//F.resize(4, 4);
	//F = Q;
	//F << 5000, 0, 0, 0,
	//	0, 0, 0, 0,
	//	0, 0, 0, 0,
	//	0, 0, 0, 0;

	//R.resize(1, 1);
	//R << 0.1;

	//MPC_Controller MPCController(A, B, Q, F, R, 300);
	//MPCController.MPC_Init();

	//Eigen::VectorXd xInit, xDes;
	//xInit.resize(4, 1);
	//xInit << 0,
	//		 0,
	//	     0,
	//	     0;

	//xDes.resize(4, 1);
	//xDes << 0,
	//		0,
	//		0,
	//		0;

	//Eigen::VectorXd u;

	Robot* robot = new Robot();
	 
	int timeStep = (int)robot->getBasicTimeStep();

	//motor inital
	Motor* wheels[2];
	std::string wheels_names[4] = { "L_Motor", "R_Motor" };
	Motor* legs[4];
	std::string legs_names[4] = { "FL_Motor", "BL_Motor", "FR_Motor", "BR_Motor" };

    

    for (int i = 0; i < 2; i++) {
		wheels[i] = robot->getMotor(wheels_names[i]);
		wheels[i]->setPosition(INFINITY);
		wheels[i]->setVelocity(0.0);
		wheels[i]->setAcceleration(-1);
	}

	for (int i = 0; i < 4; i++) {
		legs[i] = robot->getMotor(legs_names[i]);
		legs[i]->setPosition(0);
		//legs[i]->setVelocity(0.0);
		legs[i]->setAcceleration(-1);
	}	

	//sensor inital
	PositionSensor* Encoder[2];
	std::string Encoder_name[2] = { "encoder_wheelL", "encoder_wheelR" };
	for (int i = 0; i < 2; ++i) {
		Encoder[i] = robot->getPositionSensor(Encoder_name[i]);
		Encoder[i]->enable(timeStep);
	}

	PositionSensor* legsEncoder[4];
	std::string legsEncoder_name[4] = { "encoder_FL", "encoder_BL", "encoder_FR", "encoder_BR" };
	for (int i = 0; i < 4; ++i) {
		legsEncoder[i] = robot->getPositionSensor(legsEncoder_name[i]);
		legsEncoder[i]->enable(timeStep);
	}


	InertialUnit* Imu = robot->getInertialUnit("imu");
	Imu->enable(timeStep);

	// keyborad inital
	Keyboard keyboard;
	keyboard.enable(5);  // 启用键盘控制，参数表示更新频率
	
	float L = Get_L(legsEncoder);
	Eigen::MatrixXd A, B, Q, F, R;
	Generate_SpaceEquation(A, B, L, DT);

	Q.resize(6, 6);
	Q << 10, 0, 0, 0, 0, 0,
		 0, 10, 0, 0, 0, 0,
		 0, 0, 200, 0, 0, 0,
		 0, 0, 0, 100, 0, 0,
		 0, 0, 0, 0, 100, 0,
		 0, 0, 0, 0, 0, 1;

	F.resize(6, 6);
	F = Q;

	R.resize(2, 2);
	R << 1, 0,
		 0, 1;

	MPC_Controller MPCController(A, B, Q, F, R, 200);
	MPCController.MPC_Init();
	Eigen::VectorXd xInit, xDes;
	xInit.resize(6, 1);
	xInit << 0,
			 0,
			 0,
			 0,
			 0,
			 0;

	xDes.resize(6, 1);
	xDes << 0,
			0,
			0,
			0,
			0,
			0;

	Eigen::VectorXd u;
	VIRTUAL_MODEL_CONTROL virtual_model_controller(legs, legsEncoder, Imu, robot);
	ClassicPidStructTypedef TpPid;
	TpPid.Kp = 300.0;
	TpPid.Kd = 300;// 1.0;
	TpPid.Ki = 0.0;
	TpPid.LimitOutput = 50;
	TpPid.LimitIntegral = 0;
	TpPid.Integral = 0;
	TpPid.PreError = 0;
	TpPid.SectionFlag = 0;
	// 机器人控制
	int key;
	double last_pos = 0;
	double last_angle = 0;
	double virtual_angle = 0;
	double last_virtual_angle = 0;
	double now_time = 0;
	double last_time = 0;
	float offset;

	
	while (robot->step(timeStep) != -1) {
		key = keyboard.getKey();
		
		switch (key) {
        case 'S'://Keyboard::DOWN:
			xDes << 0,
					0,
					0,
					0,
					0,
					0;
			break;
		case 'A'://Keyboard::LEFT:
			xDes << 0,
				    0,
					-1,
					0,
					0,
					0;
			break;
		case 'D'://Keyboard::RIGHT:
			xDes << 0,
					0,
					1,
					0,
					0,
					0;
			break;
		default:
			// 处理其他键或不处理键的情况
			break;
		}

		L = Get_L(legsEncoder);
		std::cout << "L: " << L << std::endl;
		virtual_angle = Get_Virtual_Angle(legsEncoder, Imu, offset);
		now_time = robot->getTime();
		double avg_encoder = (Encoder[0]->getValue() + Encoder[1]->getValue()) / 2;
		xInit << virtual_angle,
			(virtual_angle - last_virtual_angle) / (now_time - last_time),
			avg_encoder * 0.05,
			(avg_encoder * 0.05 - last_pos) / (now_time - last_time),
			Imu->getRollPitchYaw()[1],
			(Imu->getRollPitchYaw()[1] - last_angle) / (now_time - last_time),
			
		Generate_SpaceEquation(A, B, L, DT);
		MPCController.MPC_Update(A, B);
		u = MPCController.MPC_Predict(xInit, xDes);
		double T = u(0, 0);
		for (int i = 0; i < 2; i++) {
			wheels[i]->setTorque(T / 2);
		}
		double Tp_offset = ClassicPidRegulate(0, offset, &TpPid);
		double Tp = u(1, 0);
		virtual_model_controller.run(Tp / 2, Tp_offset);
		/*std::cout << "avg_encoder: " << avg_encoder * 0.05 << "  \n" << "fi: " << Imu->getRollPitchYaw()[1] << "  "
			<< (Imu->getRollPitchYaw()[1] - last_angle) / (0.005 * timeStep) << " \n"
			<< "theta: " << virtual_angle << " \n"
			<< "T: " << T << "  Tp: " << Tp << std::endl;*/

		std::cout << "XInit: \n" << xInit << " \n"
			<< "XDes: \n" << xDes << " \n";

		std::cout << "T: " << T << "  Tp: " << Tp << std::endl;
		last_angle = Imu->getRollPitchYaw()[1];
		last_pos = avg_encoder * 0.05;
		last_virtual_angle = virtual_angle;
		last_time = now_time;
	};
	delete robot;
	return 0;
}
