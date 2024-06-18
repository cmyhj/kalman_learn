#include "kalman/kalman.hpp"

// sateSize状态量个数
// uSize输入的维度
using namespace std;

Kalman::Kalman(double _time) : T(_time) {
    x.setZero();
    P.setIdentity();

    A << 1, T, 0.5 * T * T,
            0, 1, T,
            0, 0, 1;

    H << 1, 0, 0;

    P.setIdentity();

    x.setZero();

};

void Kalman::Q_set(double qx) {
    Eigen::Vector3d G;
    G << T * T * T / 6.0, T * T / 2.0, T;
    Q = G * qx * G.transpose();
}

void Kalman::R_set(double rx) {
    R << rx;
}

Eigen::VectorXd Kalman::predict() {
    x = A * x;
    P = A * P * A.transpose() + Q;
    return x;
}

Eigen::VectorXd Kalman::update(const Eigen::VectorXd &z_meas) {

    Eigen::MatrixXd Ht = H.transpose();

    Eigen::MatrixXd K = P * Ht * (H * P * Ht + R).inverse();

    x = x + K * (z_meas - H * x);

    P = (Eigen::Matrix3d::Identity() - K * H) * P;

    return x;
}







