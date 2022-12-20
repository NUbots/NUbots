/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   NoiseParams.h
 *  @author Ross Hartley
 *  @brief  Header file for Invariant EKF noise parameter class
 *  @date   September 25, 2018
 **/
#ifndef UTILITY_MATH_FILTER_INEKF_NOISEPARAMS_HPP
#define UTILITY_MATH_FILTER_INEKF_NOISEPARAMS_HPP

#include <Eigen/Dense>
#include <iostream>

namespace utility::math::filter::inekf {

    class NoiseParams {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        NoiseParams();
        NoiseParams(double gyro_noise, double acc_noise, double gyro_bias, double acc_bias, double contact);

        void set_gyroscope_noise(double std);
        void set_gyroscope_noise(const Eigen::Vector3d& std);
        void set_gyroscope_noise(const Eigen::Matrix3d& cov);

        void set_accelerometer_noise(double std);
        void set_accelerometer_noise(const Eigen::Vector3d& std);
        void set_accelerometer_noise(const Eigen::Matrix3d& cov);

        void set_gyroscope_bias_noise(double std);
        void set_gyroscope_bias_noise(const Eigen::Vector3d& std);
        void set_gyroscope_bias_noise(const Eigen::Matrix3d& cov);

        void set_accelerometer_bias_noise(double std);
        void set_accelerometer_bias_noise(const Eigen::Vector3d& std);
        void set_accelerometer_bias_noise(const Eigen::Matrix3d& cov);

        void set_landmark_noise(double std);
        void set_landmark_noise(const Eigen::Vector3d& std);
        void set_landmark_noise(const Eigen::Matrix3d& cov);

        void set_contact_noise(double std);
        void set_contact_noise(const Eigen::Vector3d& std);
        void set_contact_noise(const Eigen::Matrix3d& cov);

        Eigen::Matrix3d get_gyroscope_cov();
        Eigen::Matrix3d get_accelerometer_cov();
        Eigen::Matrix3d get_gyroscope_bias_cov();
        Eigen::Matrix3d get_accelerometer_bias_cov();
        Eigen::Matrix3d get_landmark_cov();
        Eigen::Matrix3d get_contact_cov();

        friend std::ostream& operator<<(std::ostream& os, const NoiseParams& p);

    private:
        Eigen::Matrix3d Qg_;
        Eigen::Matrix3d Qa_;
        Eigen::Matrix3d Qbg_;
        Eigen::Matrix3d Qba_;
        Eigen::Matrix3d Ql_;
        Eigen::Matrix3d Qc_;
    };

}  // namespace utility::math::filter::inekf
#endif
