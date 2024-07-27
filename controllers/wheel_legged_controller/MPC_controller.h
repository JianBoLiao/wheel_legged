#pragma once

#include <Eigen/Dense>
#include <math.h>

class MPC_Controller {
public:
    MPC_Controller(const Eigen::MatrixXd _A, const Eigen::MatrixXd _B,
        const Eigen::MatrixXd _Q, const Eigen::MatrixXd _F, const Eigen::MatrixXd _R,
        int _N);

    void MPC_Init();
    void MPC_Update(const Eigen::MatrixXd _A, const Eigen::MatrixXd _B);
    Eigen::VectorXd MPC_Predict(Eigen::VectorXd x, Eigen::VectorXd xd);

private:
    Eigen::MatrixXd A, B, Q, F, R;
    int N;
    Eigen::MatrixXd E, G, H;
    int num_state;
    int num_control;
};