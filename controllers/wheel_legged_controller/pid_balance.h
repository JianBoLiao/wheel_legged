#ifndef __PID_BALANCE__
#define __PID_BALANCE__

#include "public.h"
#include "pid.h"

using namespace webots;
//ClassicPidStructTypedef AnglePid, SpeedPid;
//void pidInit();
//float pidCalculate();

#define encoder2wheel 1.00

class PID_BALANCE {
    public:
        double encoder_pre[2] = {0.0};
        double encoder_now[2] = {0.0};
        double time_pre = 0.0;
        double time_now = 0.0;
        double encoder_speed[2] = {0.0};
        double Yaw = 0;

        PID_BALANCE(Motor* Robotwheels[2], PositionSensor* RobotEncoder[2], InertialUnit* RobotImu, Robot* RobotRobot);
        ClassicPidStructTypedef AnglePid;
        ClassicPidStructTypedef SpeedPid;
        ClassicPidStructTypedef TurnPid;

        void get_Yaw();
        void run(float SpeedExpected);
        void run(float SpeedExpected, float YawExpected);



    private:
        double YawNow = 0;
        double YawPre = 0;
        

        float BalanceSpeedFeedback;
        float BalanceAngleExpected;
        float BalanceAngleFeedBack;
        float BalanceTorqueExpected;
        float BalanceTurnExpected;



        Motor* wheels[2];
        PositionSensor* Encoder[2];
        InertialUnit* Imu;
        Robot* robot;
};


#endif