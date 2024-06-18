#ifndef KALMAN_HPP
#define KALMAN_HPP

#define QUEEN_LENGTH 20 //计算方差的窗口，窗口越大，滞后越大

#include <cmath>

#include <iostream>
#include <vector>
#include <atomic>

#include <Eigen/Dense>
//#include <angles/angles.h>

#include <rclcpp/rclcpp.hpp>


class Kalman {
private:

    //温馨提示,匀加速和匀速模型的维数不一样哦

    Eigen::Matrix3d A;
    Eigen::Matrix<double,1,3> H;

    Eigen::Matrix3d P;

    Eigen::Matrix3d Q;
    Eigen::Matrix<double,1,1> R;

    Eigen::Vector3d x;


    double T;

public:
    //初始化A，H，P矩阵等
    Kalman(double _time);
    void Q_set(double qx);
    void R_set(double rx);

    Eigen::VectorXd predict();
    Eigen::VectorXd update(const Eigen::VectorXd &z_meas);
};

#endif
