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

static const double ERROR_THRESHOLD = 0.1;

// Mahony config
// Optimal parameters determined in MATLAB optimisation routine.
// To see report on this, go to public/projects folder on the NAS.
static const double Kp = 10.8279;
static const double Ki = 17.8421;
static const double ts = 0.0085;
static const Eigen::Vector4d initial_quat = Eigen::Vector4d(1, 0, 0, 0);

std::vector<Eigen::Vector3d> gyro;
std::vector<Eigen::Vector3d> acc;
std::vector<Eigen::Matrix3d> Rtw_ground;

void init() {
    Eigen::Matrix3d Rtw;
    gyro.push_back(Eigen::Vector3d( 0.0 ,  0.0 ,  0.0 ));
    acc.push_back(Eigen::Vector3d( 0.0 ,  0.0 ,  -9.80665 ));
    Rtw <<  1.0 ,  0.0 ,  0.0 ,  0.0 ,  1.0 ,  0.0 ,  0.0 ,  0.0 ,  1.0 ;
    Rtw_ground.push_back(Rtw);
}

TEST_CASE("Test the Mahony Filter", "[utility][math][filter][mahony]") {
    // Set the bias. Mahony will update this.
    Eigen::Vector3d bias(0, 0, 0);

    // Set the initial quaternion
    Eigen::Quaterniond Rwt(initial_quat[0],initial_quat[1],initial_quat[2],initial_quat[3]);

    // Init variables
    double diff = 0;

    // Populate data from Simscape
    init();

    for (int i = 0 ; i < gyro.size() ; i++) {
        MahonyUpdate(acc.at(i), gyro.at(i), ts, Ki, Kp, Rwt, bias);
        Rtw_ground.at(i).col(0).normalize();
        Rtw_ground.at(i).col(1).normalize();
        Rtw_ground.at(i).col(2).normalize();
        diff = abs(Eigen::Quaterniond(Rtw_ground.at(i)).normalized().dot(Rwt.inverse()));
        INFO("Iteration: " << i);
        INFO("Simscape: " << Rtw_ground.at(i).matrix());
        INFO("Mahony: " << Rwt.toRotationMatrix().transpose().matrix());
        INFO("Threshold is " << ERROR_THRESHOLD << ", got " << diff);
        REQUIRE(diff > 1 - ERROR_THRESHOLD);
    }
}
