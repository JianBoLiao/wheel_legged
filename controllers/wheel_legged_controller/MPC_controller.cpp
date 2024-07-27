#include "MPC_Controller.h"
#include <iostream>
/*
��ɢϵͳ��״̬����ΪX(k+1) = A * X(k) + B * U(k)
Q:Ԥ�������״̬��������Ȩ�ؾ���
F:Ԥ���β��״̬����������Ȩ�ؾ���
R:ϵͳ�����Ȩ�ؾ���
N:Ԥ��Ĳ���
*/

// Eigen::MatrixXd ˫���ȿɱ��С����
MPC_Controller::MPC_Controller(const Eigen::MatrixXd _A, const Eigen::MatrixXd _B,
    const Eigen::MatrixXd _Q, const Eigen::MatrixXd _F, const Eigen::MatrixXd _R,
    int _N) {
    this->A = _A;
    this->B = _B;
    this->Q = _Q;
    this->F = _F;
    this->R = _R;
    this->N = _N;
}


void MPC_Controller::MPC_Init() {
    num_state = A.rows();
    num_control = B.cols();

    //X(k) = M * x(k) + C * U(k)    ��д��ĸ��ʾԤ������е�ֵ���ɵ�����
    Eigen::MatrixXd C, M;
    C.resize((N + 1) * num_state, num_control * N);
    C.setZero();
    M.resize((N + 1) * num_state, num_state);

    Eigen::MatrixXd temp;
    temp.resize(num_state, num_state);
    temp.setIdentity();

    M.block(0, 0, num_state, num_state).setIdentity();

    for (int i = 1; i <= N; ++i) {
        Eigen::MatrixXd temp_c;
        temp_c.resize(num_state, (N + 1) * num_control);
        temp_c << temp * B, C.block(num_state * (i - 1), 0, num_state, C.cols());

        C.block(num_state * i, 0, num_state, C.cols())
            = temp_c.block(0, 0, num_state, temp_c.cols() - num_control);

        temp = A * temp;
        M.block(num_state * i, 0, num_state, num_state) = temp;
    }

    //Q_bar:����״̬���Ȩ�ؾ��� 
    //R_bar:��������Ȩ�ؾ���
    Eigen::MatrixXd Q_bar, R_bar;
    Q_bar.resize(num_state * (N + 1), num_state * (N + 1));
    Q_bar.setZero();
    for (unsigned int i = 0; i < N; ++i) {
        Q_bar.block(num_state * i, num_state * i, num_state, num_state) = Q;
    }
    Q_bar.block(num_state * N, num_state * N, num_state, num_state) = F;

    R_bar.resize(N * num_control, N * num_control);
    R_bar.setZero();
    for (unsigned int i = 0; i < N; ++i) {
        R_bar.block(i * num_control, i * num_control, num_control, num_control) = R;
    }

    //���Ĵ��ۺ�����дΪ:(2x(k)T * E - 2xd(k)G) * U(k) + U(k)T * H * U(k)
    G = Q_bar * C;
    E = M.transpose() * Q_bar * C;
    H = C.transpose() * Q_bar * C + R_bar;
}

void MPC_Controller::MPC_Update(const Eigen::MatrixXd _A, const Eigen::MatrixXd _B){
    A = _A;
    B = _B;
    this->MPC_Init();
}

/*
x:��ǰ״̬ numstate * 1�ľ���
xd:Ŀ��״̬ numstate(N +1) * 1�ľ���
*/
Eigen::VectorXd MPC_Controller::MPC_Predict(Eigen::VectorXd x, Eigen::VectorXd xd) {
    Eigen::VectorXd Xd;
    
    Xd.resize((N + 1) * num_state, 1);

    for (int i = 0; i <= N; i++) {
        Xd.block(i * num_state, 0, num_state, 1) = xd;
    }
    
    //����Ϊ��ʡ�£�û��ʹ��osqp�Ż���
    //���ݳ���ѧϰ�Ķ��η��̵ļ�ֵ��ȡ�� -b / 2a
    Eigen::VectorXd U = H.inverse() * ((Xd.transpose() * G - x.transpose() * E).transpose());
    
    return U.block(0, 0, num_control, 1);
}