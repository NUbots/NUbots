/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   LieGroup.h
 *  @author Ross Hartley
 *  @brief  Header file for various Lie Group functions
 *  @date   September 25, 2018
 **/

#ifndef UTILITY_MATH_FILTER_INEKF_LIEGROUP_HPP
#define UTILITY_MATH_FILTER_INEKF_LIEGROUP_HPP

#include <Eigen/Dense>
#include <iostream>

namespace utility::math::filter::inekf {

    extern const double TOLERANCE;

    Eigen::Matrix3d skew(const Eigen::Vector3d& v);
    Eigen::Matrix3d exp_so3(const Eigen::Vector3d& w);
    Eigen::MatrixXd exp_sek3(const Eigen::VectorXd& v);
    Eigen::MatrixXd adjoint_sek3(const Eigen::MatrixXd& X);

}  // namespace utility::math::filter::inekf
#endif
