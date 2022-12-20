/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   InEKF.h
 *  @author Ross Hartley
 *  @brief  Header file for Invariant EKF
 *  @date   September 25, 2018
 **/

#ifndef UTILITY_MATH_FILTER_INEKF_INEKF_HPP
#define UTILITY_MATH_FILTER_INEKF_INEKF_HPP

#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
#include <map>
#include <vector>

#include "LieGroup.hpp"
#include "NoiseParams.hpp"
#include "RobotState.hpp"

namespace utility::math::filter::inekf {

    struct KinematicPose {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        int id                                 = 0;
        Eigen::Matrix4d pose                   = Eigen::Matrix4d::Identity();
        Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity();
    };

    struct Landmark {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        int id                   = 0;
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
    };

    using eigen_pair_int_vec3d = Eigen::aligned_allocator<std::pair<const int, Eigen::Vector3d>>;

    typedef std::map<int, Eigen::Vector3d, std::less<int>, eigen_pair_int_vec3d> map_int_vec3d;
    typedef std::map<int, Eigen::Vector3d, std::less<int>, eigen_pair_int_vec3d>::iterator map_int_vec3d_it;

    typedef std::vector<Landmark, Eigen::aligned_allocator<Landmark>> landmarks;
    typedef std::vector<Landmark, Eigen::aligned_allocator<Landmark>>::const_iterator landmarks_it;

    typedef std::vector<KinematicPose, Eigen::aligned_allocator<KinematicPose>> kinematics;
    typedef std::vector<KinematicPose, Eigen::aligned_allocator<KinematicPose>>::const_iterator kinematics_it;

    struct Observation {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::VectorXd Y;
        Eigen::VectorXd b;
        Eigen::MatrixXd H;
        Eigen::MatrixXd N;
        Eigen::MatrixXd PI;

        bool empty() {
            return Y.rows() == 0;
        }

        friend std::ostream& operator<<(std::ostream& os, const Observation& o) {
            os << "---------- Observation ------------" << std::endl;
            os << "Y:\n" << o.Y << std::endl << std::endl;
            os << "b:\n" << o.b << std::endl << std::endl;
            os << "H:\n" << o.H << std::endl << std::endl;
            os << "N:\n" << o.N << std::endl << std::endl;
            os << "PI:\n" << o.PI << std::endl;
            os << "-----------------------------------";
            return os;
        }
    };


    class InEKF {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        InEKF();
        InEKF(RobotState state, NoiseParams params);

        RobotState get_state();
        map_int_vec3d get_prior_landmarks();
        std::map<int, int> get_estimated_landmarks();
        std::map<int, bool> get_contacts();
        std::map<int, int> get_estimated_contact_positions();
        void set_state(RobotState state);
        void set_noise_params(NoiseParams params);
        void set_prior_landmarks(const map_int_vec3d& prior_landmarks);
        void set_contacts(std::vector<std::pair<int, bool>> contacts);

        void propagate(const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc, double dt);
        void correct(const Observation& obs);
        void correct_landmarks(const landmarks& measured_landmarks);
        void correct_kinematics(const kinematics& measured_kinematics);

    private:
        RobotState state{};
        NoiseParams noise_params{};
        const Eigen::Vector3d gravity{0.0, 0.0, -9.81};
        map_int_vec3d prior_landmarks{};
        std::map<int, int> estimated_landmarks{};
        std::map<int, bool> contacts{};
        std::map<int, int> estimated_contact_positions{};
    };

}  // namespace utility::math::filter::inekf
#endif
