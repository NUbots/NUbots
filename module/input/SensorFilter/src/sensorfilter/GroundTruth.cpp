/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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
