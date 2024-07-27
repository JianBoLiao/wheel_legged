#ifndef __LQR__
#define __LQR__

#include "public.h"


class LQR_ER {
public:
    LQR_ER(Motor* Robotwheels[2], PositionSensor* RobotEncoder[2], InertialUnit* RobotImu, Robot* RobotRobot);

    // LQR反馈增益矩阵参数
    float LQR_K1;
    float LQR_K2;
    float LQR_K3;
    float LQR_K4;
    float LQR_K15, LQR_K25; 
    float LQR_K16, LQR_K26;

    float TorqueExpected[2] = { 0 };
    // LQR求解器
    void lqr_calculate(float SpeedExpected, float YawExpected);

private:
    void get_Yaw();
    void refresh_information(float SpeedExpected);

    // 状态空间变量

    float displacement_now = 0;
    float displacement_pre = 0;

    float displacement_Expected = 0;

    float speed = 0; 
    
    float pitch_now = 0;
    float pitch_pre = 0;
    
    float pitch_speed = 0;

    float yaw_now = 0;
    float yaw_pre = 0;
    float yaw = 0;
    float yaw_last = 0;

    float yaw_speed = 0;

    float time_pre = 0;
    float time_now = 0;

    Motor* wheels[2];
    PositionSensor* Encoder[2];
    InertialUnit* Imu;
    Robot* robot;
    

};


#endif

