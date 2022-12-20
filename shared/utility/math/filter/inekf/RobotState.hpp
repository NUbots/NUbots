/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   RobotState.h
 *  @author Ross Hartley
 *  @brief  Header file for RobotState
 *  @date   September 25, 2018
 **/

#ifndef UTILITY_MATH_FILTER_INEKF_ROBOTSTATE_HPP
#define UTILITY_MATH_FILTER_INEKF_ROBOTSTATE_HPP

#include <Eigen/Dense>
#include <iostream>

#if INEKF_USE_MUTEX
    #include <mutex>
#endif

namespace utility::math::filter::inekf {

    class RobotState {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        RobotState();
        RobotState(const Eigen::MatrixXd& X);
        RobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta);
        RobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta, const Eigen::MatrixXd& P);

        const Eigen::MatrixXd get_X();
        const Eigen::VectorXd get_theta();
        const Eigen::MatrixXd get_P();
        const Eigen::Matrix3d get_rotation();
        const Eigen::Vector3d get_velocity();
        const Eigen::Vector3d get_position();
        const Eigen::Vector3d get_gyroscope_bias();
        const Eigen::Vector3d get_accelerometer_bias();
        int dim_X();
        int dim_theta();
        int dim_P();

        void set_X(const Eigen::MatrixXd& X);
        void set_P(const Eigen::MatrixXd& P);
        void set_theta(const Eigen::VectorXd& theta);
        void set_rotation(const Eigen::Matrix3d& R);
        void set_velocity(const Eigen::Vector3d& v);
        void set_position(const Eigen::Vector3d& p);
        void set_gyroscope_bias(const Eigen::Vector3d& bg);
        void set_accelerometer_bias(const Eigen::Vector3d& ba);

        void copy_diag_X(int n, Eigen::MatrixXd& big_X);

        friend std::ostream& operator<<(std::ostream& os, const RobotState& s);

    private:
        Eigen::MatrixXd X;
        Eigen::VectorXd Theta;
        Eigen::MatrixXd P;
    };

}  // namespace utility::math::filter::inekf
#endif
