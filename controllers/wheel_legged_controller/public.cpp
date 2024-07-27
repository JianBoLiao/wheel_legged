#include "public.h"
using namespace webots;

Kinematic_Results Kinematic_Inv(float x, float y) {
    Kinematic_Results answer;
    float rad_AC = Rad_Stand(atan2(y, x + 0.5 * JOINT_DISTANCE));
    float rad_EC = Rad_Stand(atan2(y, x - 0.5 * JOINT_DISTANCE));

    float L_AC = sqrt(pow((x + 0.5 * JOINT_DISTANCE), 2) + y * y);
    float L_EC = sqrt(pow((x - 0.5 * JOINT_DISTANCE), 2) + y * y);
    float BAC = Rad_Stand(acos(
        (pow(L_AC, 2) + pow(BIG_LEG, 2) - pow(SMALL_LEG, 2)) /
        (2 * BIG_LEG * L_AC)
    ));
    float DEC = Rad_Stand(acos(
        (pow(L_EC, 2) + pow(BIG_LEG, 2) - pow(SMALL_LEG, 2)) /
        (2 * BIG_LEG * L_EC)
    ));
    answer.rad_AB = rad_AC - BAC - 1.5 * PI; // 计算完成后偏移到相对于竖直轴的角度
    answer.rad_ED = rad_AC + DEC - 1.5 * PI; // 计算完成后偏移到相对于竖直轴的角度
    return answer;
}

Point_XY Kinematic_Forward(Kinematic_Results Angle) {

    Point_XY B;
    Point_XY D;
    Point_XY C;
    


    // B.x = BIG_LEG * cos(Angle.rad_AB) - JOINT_DISTANCE / 2.0;
    // B.y = BIG_LEG * sin(Angle.rad_AB);
    // D.x = BIG_LEG * cos(Angle.rad_ED) + JOINT_DISTANCE / 2.0;
    // D.y = BIG_LEG * sin(Angle.rad_ED);

    // float a = 2 * SMALL_LEG * (D.x - B.x);
    // float b = 2 * SMALL_LEG * (D.y - B.y);
    // float c = Points_Distance(B, D) * Points_Distance(B, D);

    // float Aerfa = Rad_Stand(2 * atan2(b + sqrt(a * a + b * b - c * c), a + c));

    // //足端朝上的解
    // C.x = B.x + SMALL_LEG * cos(Aerfa);
    // C.y = B.y + SMALL_LEG * sin(Aerfa);
    // //足端朝下的解
    // C.x = B.x + D.x - C.x;
    // C.y = B.y + D.y - C.y;

    // return C;

    // matlab 移植运动学正解
    Point_XY A;
    Point_XY E;
    A.x = 0.5 * JOINT_DISTANCE;
    A.y = 0;
    E.x = -0.5 * JOINT_DISTANCE;
    E.y = 0;

    Angle.rad_AB = Angle.rad_AB - PI;
    Angle.rad_ED = Angle.rad_ED - PI;

    B.x =  - A.x + BIG_LEG * cos(Angle.rad_AB);
    B.y = A.y + BIG_LEG * sin(Angle.rad_AB);
    D.x = - E.x + BIG_LEG * cos(Angle.rad_ED);
    D.y = E.y + BIG_LEG * sin(Angle.rad_ED);
    float lengthBD = Points_Distance(B, D);
    float A0 = 2 * SMALL_LEG * (D.x - B.x);
    float B0 = 2 * SMALL_LEG * (D.y - B.y);
    float C0 = lengthBD * lengthBD;
    float theta2 = Rad_Stand(2 * atan2(B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0), A0 + C0));

    C.x = B.x + SMALL_LEG * cos(theta2);
    C.y = B.y + SMALL_LEG * sin(theta2);

    C.x = - C.x;
    C.y = - C.y;
    return C;
}

Transformed_Jacob TransJacob(Kinematic_Results Angles) {
    Transformed_Jacob Jacob;
    float derta = 0.00001;
    Kinematic_Results Angles_AB_Incre = Angles;
    Angles_AB_Incre.rad_AB += derta;

    Kinematic_Results Angles_ED_Incre = Angles;
    Angles_ED_Incre.rad_ED += derta;

    Jacob.dx_dq1 = (Kinematic_Forward(Angles_AB_Incre).x - Kinematic_Forward(Angles).x) / derta;
    Jacob.dx_dq2 = (Kinematic_Forward(Angles_ED_Incre).x - Kinematic_Forward(Angles).x) / derta;
    Jacob.dy_dq1 = (Kinematic_Forward(Angles_AB_Incre).y - Kinematic_Forward(Angles).y) / derta;
    Jacob.dy_dq2 = (Kinematic_Forward(Angles_ED_Incre).y - Kinematic_Forward(Angles).y) / derta;
    return Jacob;
}

Transformed_Jacob TransJacob_new(Kinematic_Results Angles) {
    Transformed_Jacob Jacob;
    float derta = 0.00001;
    Kinematic_Results Angles_AB_Incre = Angles;
    Angles_AB_Incre.rad_AB += derta;
    
    Kinematic_Results Angles_ED_Incre = Angles;
    Angles_ED_Incre.rad_ED += derta;
    
    double L0_AB = sqrt(pow(Kinematic_Forward(Angles_AB_Incre).x, 2) + pow(Kinematic_Forward(Angles_AB_Incre).y, 2)) - sqrt(pow(Kinematic_Forward(Angles).x, 2) + pow(Kinematic_Forward(Angles).y, 2));
    double L0_ED = sqrt(pow(Kinematic_Forward(Angles_ED_Incre).x, 2) + pow(Kinematic_Forward(Angles_ED_Incre).y, 2)) - sqrt(pow(Kinematic_Forward(Angles).x, 2) + pow(Kinematic_Forward(Angles).y, 2));
    double Theta0_AB = atan2(Kinematic_Forward(Angles_AB_Incre).y, Kinematic_Forward(Angles_AB_Incre).x) - atan2(Kinematic_Forward(Angles).y, Kinematic_Forward(Angles).x);
    double Theta0_ED = atan2(Kinematic_Forward(Angles_ED_Incre).y, Kinematic_Forward(Angles_ED_Incre).x) - atan2(Kinematic_Forward(Angles).y, Kinematic_Forward(Angles).x);
    Jacob.dx_dq1 = L0_AB / derta;
    Jacob.dx_dq2 = L0_ED / derta;
    Jacob.dy_dq1 = Theta0_AB / derta;
    Jacob.dy_dq2 = Theta0_ED / derta;
    return Jacob;
}

float Points_Distance(float x1, float y1, float x2, float y2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

float Points_Distance(Point_XY A, Point_XY B) {
    return sqrt(pow(A.x - B.x, 2) + pow(A.y - B.y, 2));
}

float Rad_Stand(float input) {
    if (input < 0) {
        return (input + 2 * PI);
    }
    else if (input > 2 * PI)
    {
        return (input - 2 * PI);
    }
    else { return input; }
}

float Get_L(PositionSensor* Encoder[4]) {
    Kinematic_Results AnglesMeasured[2] = { 0 };// 关节电机编码器测量值
    AnglesMeasured[0].rad_AB = -Encoder[0]->getValue() + PI * 5.0 / 3.0;
    AnglesMeasured[0].rad_ED = Encoder[1]->getValue() + PI * 4.0 / 3.0;
    AnglesMeasured[1].rad_AB = -Encoder[2]->getValue() + PI * 5.0 / 3.0;
    AnglesMeasured[1].rad_ED = Encoder[3]->getValue() + PI * 4.0 / 3.0;

    Point_XY PosMeasured[2] = { 0 };
    PosMeasured[0] = Kinematic_Forward(AnglesMeasured[0]);
    PosMeasured[1] = Kinematic_Forward(AnglesMeasured[1]);

    float L_0 = Points_Distance(PosMeasured[0].x, PosMeasured[0].y, 0, 0);
    float L_1 = Points_Distance(PosMeasured[1].x, PosMeasured[1].y, 0, 0);
    return (L_0 + L_1) / 2 / 2;
}

float Get_Virtual_Angle(PositionSensor* Encoder[4], InertialUnit* Imu, float& offset)
{
    Kinematic_Results AnglesMeasured[2] = { 0 };// 关节电机编码器测量值
    AnglesMeasured[0].rad_AB = -Encoder[0]->getValue() + PI * 5.0 / 3.0;
    AnglesMeasured[0].rad_ED = Encoder[1]->getValue() + PI * 4.0 / 3.0;
    AnglesMeasured[1].rad_AB = -Encoder[2]->getValue() + PI * 5.0 / 3.0;
    AnglesMeasured[1].rad_ED = Encoder[3]->getValue() + PI * 4.0 / 3.0;

    Point_XY PosMeasured[2] = { 0 };
    PosMeasured[0] = Kinematic_Forward(AnglesMeasured[0]);
    PosMeasured[1] = Kinematic_Forward(AnglesMeasured[1]);

    float angle[2];
    angle[0] = atan2(PosMeasured[0].y, PosMeasured[0].x);
    angle[1] = atan2(PosMeasured[1].y, PosMeasured[1].x);

    float pitch = Imu->getRollPitchYaw()[1];
    angle[0] = angle[0] + pitch;
    angle[1] = angle[1] + pitch;
    angle[0] = -(0.5 * PI + angle[0]);
    angle[1] = -(0.5 * PI + angle[1]);


    //// 将角度转换到[0, 2PI]
    //for (int i = 0; i < 2; i++) {
    //    if (angle[i] < 0) {
    //        angle[i] += 2 * PI;
    //    }
    //}
    offset = angle[1] - angle[0];
    return (angle[0] + angle[1]) / 2;
}

void Generate_SpaceEquation(Eigen::MatrixXd& A, Eigen::MatrixXd& B, float L, float DT) {
    float c1 = 18.41572381051946;
    float p1 = c1 / L;
    float c2 = 0.42787754903428676;
    float p2 = c2 / L;
    float p3 = -25.782441070887693;
    float p4 = -0.023499528250387692;
    float p5 = 44.22827598517744;
    float p6 = 42.533568339520315;
    A.resize(6, 6);
    A << 1,      1 * DT, 0, 0,      0,      0,
        p1 * DT, 1,      0, 0,      p2 * DT,0,
        0,       0,      1, 1 * DT, 0,      0,
        p3 * DT, 0,      0, 1,      p4 * DT,0,
        0,       0,      0, 0,      1,      1 * DT,
        p5 * DT, 0,      0, 0,      p6 * DT,1;

    float c3 = 2.122411240161517;
    float c4 = 0.07579914388961863;
    float p7 = -c3 / L - c4 / L / L;
    float c5 = 0.18204364324117567;
    float c6 = 0.07579914388961863;
    float p8 = c5 / L + c6 / L / L;
    float c9 = 4.194868697801149;
    float c10 = 0.10612056200807585;
    float p9 = c9 + c10 / L;
    float c11 = 0.009998046746796488;
    float c12 = 0.10612056200807585;
    float p10 = -c11 - c12 / L;
    float c13 = 0.19996093493592976;
    float c14 = 0.18204364324117567;
    float p11 = -c13 - c14 / L;
    float c15 = 18.096218785139715;
    float c16 = 0.18204364324117567;
    float p12 = c15 + c16 / L;
    B.resize(6, 2);
    B << 0, 0,
        p7 * DT, p8 * DT,
        0, 0,
        p9 * DT, p10 * DT,
        0, 0,
        p11 * DT, p12 * DT;


}