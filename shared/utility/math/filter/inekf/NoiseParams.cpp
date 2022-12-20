/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   NoiseParams.cpp
 *  @author Ross Hartley
 *  @brief  Source file for Invariant EKF noise parameter class
 *  @date   September 25, 2018
 **/

#include "NoiseParams.hpp"

namespace utility::math::filter::inekf {

    // ------------ NoiseParams -------------
    // Default Constructor
    NoiseParams::NoiseParams() {
        set_gyroscope_noise(0.01);
        set_accelerometer_noise(0.1);
        set_gyroscope_bias_noise(0.00001);
        set_accelerometer_bias_noise(0.0001);
        set_landmark_noise(0.1);
        set_contact_noise(0.1);
    }

    NoiseParams::NoiseParams(double gyro_noise, double acc_noise, double gyro_bias, double acc_bias, double contact) {
        set_gyroscope_noise(gyro_noise);
        set_accelerometer_noise(acc_noise);
        set_gyroscope_bias_noise(gyro_bias);
        set_accelerometer_bias_noise(acc_bias);
        set_landmark_noise(0.0);
        set_contact_noise(contact);
    }

    void NoiseParams::set_gyroscope_noise(double std) {
        Qg_ = std * std * Eigen::Matrix3d::Identity();
    }
    void NoiseParams::set_gyroscope_noise(const Eigen::Vector3d& std) {
        Qg_ << std(0) * std(0), 0, 0, 0, std(1) * std(1), 0, 0, 0, std(2) * std(2);
    }
    void NoiseParams::set_gyroscope_noise(const Eigen::Matrix3d& cov) {
        Qg_ = cov;
    }

    void NoiseParams::set_accelerometer_noise(double std) {
        Qa_ = std * std * Eigen::Matrix3d::Identity();
    }
    void NoiseParams::set_accelerometer_noise(const Eigen::Vector3d& std) {
        Qa_ << std(0) * std(0), 0, 0, 0, std(1) * std(1), 0, 0, 0, std(2) * std(2);
    }
    void NoiseParams::set_accelerometer_noise(const Eigen::Matrix3d& cov) {
        Qa_ = cov;
    }

    void NoiseParams::set_gyroscope_bias_noise(double std) {
        Qbg_ = std * std * Eigen::Matrix3d::Identity();
    }
    void NoiseParams::set_gyroscope_bias_noise(const Eigen::Vector3d& std) {
        Qbg_ << std(0) * std(0), 0, 0, 0, std(1) * std(1), 0, 0, 0, std(2) * std(2);
    }
    void NoiseParams::set_gyroscope_bias_noise(const Eigen::Matrix3d& cov) {
        Qbg_ = cov;
    }

    void NoiseParams::set_accelerometer_bias_noise(double std) {
        Qba_ = std * std * Eigen::Matrix3d::Identity();
    }
    void NoiseParams::set_accelerometer_bias_noise(const Eigen::Vector3d& std) {
        Qba_ << std(0) * std(0), 0, 0, 0, std(1) * std(1), 0, 0, 0, std(2) * std(2);
    }
    void NoiseParams::set_accelerometer_bias_noise(const Eigen::Matrix3d& cov) {
        Qba_ = cov;
    }

    void NoiseParams::set_landmark_noise(double std) {
        Ql_ = std * std * Eigen::Matrix3d::Identity();
    }
    void NoiseParams::set_landmark_noise(const Eigen::Vector3d& std) {
        Ql_ << std(0) * std(0), 0, 0, 0, std(1) * std(1), 0, 0, 0, std(2) * std(2);
    }
    void NoiseParams::set_landmark_noise(const Eigen::Matrix3d& cov) {
        Ql_ = cov;
    }

    void NoiseParams::set_contact_noise(double std) {
        Qc_ = std * std * Eigen::Matrix3d::Identity();
    }
    void NoiseParams::set_contact_noise(const Eigen::Vector3d& std) {
        Qc_ << std(0) * std(0), 0, 0, 0, std(1) * std(1), 0, 0, 0, std(2) * std(2);
    }
    void NoiseParams::set_contact_noise(const Eigen::Matrix3d& cov) {
        Qc_ = cov;
    }

    Eigen::Matrix3d NoiseParams::get_gyroscope_cov() {
        return Qg_;
    }
    Eigen::Matrix3d NoiseParams::get_accelerometer_cov() {
        return Qa_;
    }
    Eigen::Matrix3d NoiseParams::get_gyroscope_bias_cov() {
        return Qbg_;
    }
    Eigen::Matrix3d NoiseParams::get_accelerometer_bias_cov() {
        return Qba_;
    }
    Eigen::Matrix3d NoiseParams::get_landmark_cov() {
        return Ql_;
    }
    Eigen::Matrix3d NoiseParams::get_contact_cov() {
        return Qc_;
    }

    std::ostream& operator<<(std::ostream& os, const NoiseParams& p) {
        os << "--------- Noise Params -------------" << std::endl;
        os << "Gyroscope Covariance:\n" << p.Qg_ << std::endl;
        os << "Accelerometer Covariance:\n" << p.Qa_ << std::endl;
        os << "Gyroscope Bias Covariance:\n" << p.Qbg_ << std::endl;
        os << "Accelerometer Bias Covariance:\n" << p.Qba_ << std::endl;
        os << "Landmark Covariance:\n" << p.Ql_ << std::endl;
        os << "Contact Covariance:\n" << p.Qc_ << std::endl;
        os << "-----------------------------------" << std::endl;
        return os;
    }

}  // namespace utility::math::filter::inekf
