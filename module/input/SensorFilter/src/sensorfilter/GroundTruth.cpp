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
 * Copyright 2023 NUbots <nubots@nubots.net>
 */

#include "SensorFilter.hpp"

#include "utility/math/euler.hpp"

namespace module::input {

    using utility::math::euler::MatrixToEulerIntrinsic;

    void SensorFilter::update_odometry_ground_truth(std::unique_ptr<Sensors>& sensors, const RawSensors& raw_sensors) {
        // **************** Construct Odometry Output ****************
        Eigen::Isometry3d Hwt = Eigen::Isometry3d(raw_sensors.odometry_ground_truth.Htw).inverse();
        sensors->Htw          = Hwt.inverse().matrix();

        Eigen::Vector3d Hwt_rpy = MatrixToEulerIntrinsic(Hwt.rotation());
        Eigen::Isometry3d Hwr   = Eigen::Isometry3d::Identity();
        Hwr.linear()            = Eigen::AngleAxisd(Hwt_rpy.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Hwr.translation()       = Eigen::Vector3d(Hwt.translation().x(), Hwt.translation().y(), 0.0);
        sensors->Hrw            = Hwr.inverse().matrix();
    }
}  // namespace module::input
