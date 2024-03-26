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

#include "utility/math/filter/MahonyFilter.hpp"

#include "SensorFilter.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/BodySide.hpp"

#include "utility/input/FrameID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/euler.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::input {

    using message::actuation::BodySide;

    using extension::Configuration;

    using utility::input::FrameID;
    using utility::input::ServoID;
    using utility::math::euler::EulerIntrinsicToMatrix;
    using utility::math::euler::MatrixToEulerIntrinsic;
    using utility::math::filter::MahonyUpdate;
    using utility::support::Expression;

    void SensorFilter::configure_mahony(const Configuration& config) {
        // Mahony Filter Config
        cfg.Ki           = config["mahony"]["Ki"].as<Expression>();
        cfg.Kp           = config["mahony"]["Kp"].as<Expression>();
        cfg.initial_bias = Eigen::Vector3d(config["mahony"]["initial_bias"].as<Expression>());
        bias_mahony      = cfg.initial_bias;
        Hwt_mahony       = cfg.initial_Hwt;
    }

    void SensorFilter::reset_mahony() {
        bias_mahony = cfg.initial_bias;
        Hwt_mahony  = cfg.initial_Hwt;
    }

    void SensorFilter::update_odometry_mahony(std::unique_ptr<Sensors>& sensors,
                                              const std::shared_ptr<const Sensors>& previous_sensors,
                                              const RawSensors& raw_sensors,
                                              const std::shared_ptr<const WalkState>& walk_state) {

        // Perform anchor update (translation and yaw measurement update)
        if (walk_state != nullptr) {
            anchor_update(sensors, *walk_state);
        }

        //  Perform Mahony update (roll and pitch measurement update)
        const double dt = std::max(
            std::chrono::duration_cast<std::chrono::duration<double>>(
                raw_sensors.timestamp - (previous_sensors ? previous_sensors->timestamp : raw_sensors.timestamp))
                .count(),
            0.0);
        utility::math::filter::MahonyUpdate(sensors->accelerometer,
                                            sensors->gyroscope,
                                            Hwt_mahony,
                                            dt,
                                            cfg.Ki,
                                            cfg.Kp,
                                            bias_mahony);

        // Convert the rotation matrices from anchor and mahony method into euler angles
        Eigen::Vector3d rpy_mahony = MatrixToEulerIntrinsic(Hwt_mahony.linear());
        Eigen::Vector3d rpy_anchor = MatrixToEulerIntrinsic(Hwt.linear());

        // Remove yaw from mahony filter (prevents it breaking after numerous rotations)
        Hwt_mahony.linear() = EulerIntrinsicToMatrix(Eigen::Vector3d(rpy_mahony.x(), rpy_mahony.y(), 0.0));


        // Fuse roll and pitch of mahony filter with yaw of anchor method
        sensors->Htw.linear() =
            EulerIntrinsicToMatrix(Eigen::Vector3d(rpy_mahony.x(), rpy_mahony.y(), rpy_anchor.z())).inverse();

        // Update translation of odometry with translation from anchor method
        sensors->Htw.translation() = Hwt.inverse().translation();

        // Construct robot {r} to world {w} space transform (just x-y translation and yaw rotation)
        Eigen::Isometry3d Hwr = Eigen::Isometry3d::Identity();
        Hwr.linear()          = Eigen::AngleAxisd(rpy_anchor.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Hwr.translation()     = Eigen::Vector3d(Hwt.translation().x(), Hwt.translation().y(), 0.0);
        sensors->Hrw          = Hwr.inverse();
        update_loop.enable();
    }
}  // namespace module::input
