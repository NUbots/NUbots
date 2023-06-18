/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   RobotState.h
 *  @author Ross Hartley
 *  @brief  Source file for RobotState (thread-safe)
 *  @date   September 25, 2018
 **/

#include "RobotState.hpp"

#include "LieGroup.hpp"

namespace utility::math::filter::inekf {

    // Default constructor
    RobotState::RobotState()
        : X(Eigen::MatrixXd::Identity(5, 5))
        , Theta(Eigen::MatrixXd::Zero(6, 1))
        , P(Eigen::MatrixXd::Identity(15, 15)) {}
    // Initialize with X
    RobotState::RobotState(const Eigen::MatrixXd& x) : X(x), Theta(Eigen::MatrixXd::Zero(6, 1)) {
        P = Eigen::MatrixXd::Identity(3 * this->dim_X() + this->dim_theta() - 6,
                                      3 * this->dim_X() + this->dim_theta() - 6);
    }
    // Initialize with X and Theta
    RobotState::RobotState(const Eigen::MatrixXd& x, const Eigen::VectorXd& t) : X(x), Theta(t) {
        P = Eigen::MatrixXd::Identity(3 * this->dim_X() + this->dim_theta() - 6,
                                      3 * this->dim_X() + this->dim_theta() - 6);
    }
    // Initialize with X, Theta and P
    RobotState::RobotState(const Eigen::MatrixXd& x, const Eigen::VectorXd& t, const Eigen::MatrixXd& p)
        : X(x), Theta(t), P(p) {}
    // TODO: error checking to make sure dimensions are correct and supported

    const Eigen::MatrixXd RobotState::get_X() {
        return X;
    }
    const Eigen::VectorXd RobotState::get_theta() {
        return Theta;
    }
    const Eigen::MatrixXd RobotState::get_P() {
        return P;
    }
    const Eigen::Matrix3d RobotState::get_rotation() {
        return X.block<3, 3>(0, 0);
    }
    const Eigen::Vector3d RobotState::get_velocity() {
        return X.block<3, 1>(0, 3);
    }
    const Eigen::Vector3d RobotState::get_position() {
        return X.block<3, 1>(0, 4);
    }
    const Eigen::Vector3d RobotState::get_gyroscope_bias() {
        return Theta.head(3);
    }
    const Eigen::Vector3d RobotState::get_accelerometer_bias() {
        return Theta.tail(3);
    }
    int RobotState::dim_X() {
        return X.cols();
    }
    int RobotState::dim_theta() {
        return Theta.rows();
    }
    int RobotState::dim_P() {
        return P.cols();
    }

    void RobotState::set_X(const Eigen::MatrixXd& x) {
        X = x;
    }
    void RobotState::set_theta(const Eigen::VectorXd& t) {
        Theta = t;
    }
    void RobotState::set_P(const Eigen::MatrixXd& p) {
        P = p;
    }
    void RobotState::set_rotation(const Eigen::Matrix3d& R) {
        X.block<3, 3>(0, 0) = R;
    }
    void RobotState::set_velocity(const Eigen::Vector3d& v) {
        X.block<3, 1>(0, 3) = v;
    }
    void RobotState::set_position(const Eigen::Vector3d& p) {
        X.block<3, 1>(0, 4) = p;
    }
    void RobotState::set_gyroscope_bias(const Eigen::Vector3d& bg) {
        Theta.head(3) = bg;
    }
    void RobotState::set_accelerometer_bias(const Eigen::Vector3d& ba) {
        Theta.tail(3) = ba;
    }


    void RobotState::copy_diag_X(int n, Eigen::MatrixXd& big_X) {
        int dim_X = this->dim_X();
        for (int i = 0; i < n; ++i) {
            int start_index = big_X.rows();
            big_X.conservativeResize(start_index + dim_X, start_index + dim_X);
            big_X.block(start_index, 0, dim_X, start_index)     = Eigen::MatrixXd::Zero(dim_X, start_index);
            big_X.block(0, start_index, start_index, dim_X)     = Eigen::MatrixXd::Zero(start_index, dim_X);
            big_X.block(start_index, start_index, dim_X, dim_X) = X;
        }
        return;
    }

    std::ostream& operator<<(std::ostream& os, const RobotState& s) {
        os << "--------- Robot State -------------" << std::endl;
        os << "X:\n" << s.X << std::endl << std::endl;
        os << "Theta:\n" << s.Theta << std::endl << std::endl;
        os << "P:\n" << s.P << std::endl;
        os << "-----------------------------------";
        return os;
    }

}  // namespace utility::math::filter::inekf
