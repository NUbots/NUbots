/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2020 NUbots <nubots@nubots.net>
 */

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch.hpp>
#include <iostream>

#include "utility/math/filter/eigen/MahonyFilter.h"

using utility::math::filter::MahonyUpdate;

static const double ERROR_THRESHOLD = 1.0;

// Mahony config
static const double Kp = 10.8279;
static const double Ki = 17.8421;
static const double ts = 0.0111;
static const Eigen::Vector4d initial_quat = Eigen::Vector4d(1, 0, 0, 0);

TEST_CASE("Test the Mahony Filter", "[utility][math][filter][mahony]") {
    Eigen::Vector3d bias(0,0,0);

    Eigen::Quaterniond Rwt(initial_quat[0],initial_quat[1],initial_quat[2],initial_quat[3]);

    Eigen::Vector3d gyro;
    Eigen::Vector3d acc;
    Eigen::Matrix3d Rtw_ground;
    double diff = 0;

    gyro <<  0.0 ,  0.0 ,  0.0 ;
    acc <<  0.0 ,  0.0 ,  -9.80665 ;
    MahonyUpdate(acc, gyro, ts, Ki, Kp, Rwt, bias);
    Rtw_ground <<  1.0 ,  0.0 ,  0.0 ,  0.0 ,  1.0 ,  0.0 ,  0.0 ,  0.0 ,  1.0 ;
    Rtw_ground.col(0).normalize();
    Rtw_ground.col(1).normalize();
    Rtw_ground.col(2).normalize();
    diff = ((Rtw_ground - Rwt.toRotationMatrix().transpose())).cwiseAbs().maxCoeff();
    INFO("Simscape: " << Rtw_ground.matrix());
    INFO("Mahony: " << Rwt.toRotationMatrix().transpose().matrix());
    INFO("Threshold is " << ERROR_THRESHOLD << ", got " << diff);
    REQUIRE(diff < ERROR_THRESHOLD);

}
