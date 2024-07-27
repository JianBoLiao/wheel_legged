#ifndef __PUBLIC__
#define __PUBLIC__


#include <iostream>
#include <cmath>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Keyboard.hpp>
#include <string.h>
#include "Eigen/Dense"

using namespace webots;

#define PI 3.1415926
#define BIG_LEG 0.18
#define SMALL_LEG 0.20
#define JOINT_DISTANCE 0.12
#define WHEEL_DISTANCE 0.344
#define WHEEL_D 0.1
#define WHEEL_R 0.05

struct Kinematic_Results {
    float rad_AB;
    float rad_ED;
};

struct Point_XY {
    float x;
    float y;
};

struct Transformed_Jacob {
    float dx_dq1;
    float dx_dq2;
    float dy_dq1;
    float dy_dq2;
};

// 运动学反解函数
Kinematic_Results Kinematic_Inv(float x, float y);
// 运动学正解函数
Point_XY Kinematic_Forward(Kinematic_Results Angle);

Transformed_Jacob TransJacob(Kinematic_Results Angles);

Transformed_Jacob TransJacob_new(Kinematic_Results Angles);

// 求取输入的两个坐标点之间的距离
float Points_Distance(float x1, float y1, float x2, float y2);

float Points_Distance(Point_XY A, Point_XY B);

// 标准化三角函数值，使得弧度值规范在0～2pi 内
float Rad_Stand(float input);

float Get_L(PositionSensor* Encoder[4]);

float Get_Virtual_Angle(PositionSensor* Encoder[4], InertialUnit* Imu, float& offset);

void Generate_SpaceEquation(Eigen::MatrixXd& A, Eigen::MatrixXd& B, float L, float DT);
#endif