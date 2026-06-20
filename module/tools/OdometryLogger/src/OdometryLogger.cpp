/*
 * MIT License
 *
 * Copyright (c) 2025 NUbots
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
#include "OdometryLogger.hpp"

#include "extension/Configuration.hpp"

#include "message/behaviour/state/WalkState.hpp"
#include "message/input/MotionCapture.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/OdometryRecord.hpp"

#include "utility/math/euler.hpp"

namespace module::tools {

    using extension::Configuration;
    using message::behaviour::state::WalkState;
    using message::input::MotionCapture;
    using message::input::Sensors;
    using message::localisation::OdometryRecord;

    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::math::euler::rpy_intrinsic_to_mat;

    OdometryLogger::OdometryLogger(std::unique_ptr<NUClear::Environment> environment)
        : NUClear::Reactor(std::move(environment)) {

        on<Configuration>("OdometryLogger.yaml").then([this](const Configuration& config) {
            this->log_level         = config["log_level"].as<NUClear::LogLevel>();
            cfg.robot_rigid_body_id = config["robot_rigid_body_id"].as<uint32_t>();
        });

        on<Trigger<Sensors>, With<WalkState>, With<MotionCapture>>().then(
            [this](const Sensors& sensors,
                   const WalkState& walk_state,
                   const MotionCapture& motion_capture) {

                for (const auto& rigid_body : motion_capture.rigid_bodies) {
                    if (rigid_body.id != cfg.robot_rigid_body_id || !rigid_body.tracking_valid) {
                        continue;
                    }

                    // Build Hft from mocap rigid body (same coordinate correction as Mocap module)
                    Eigen::Isometry3d Hft = Eigen::Isometry3d::Identity();
                    Hft.translation() =
                        Eigen::Vector3d(-rigid_body.position.y(), rigid_body.position.x(), rigid_body.position.z());
                    auto rpy     = mat_to_rpy_intrinsic(Eigen::Quaterniond(rigid_body.rotation(0),
                                                                        rigid_body.rotation(1),
                                                                        rigid_body.rotation(2),
                                                                        rigid_body.rotation(3))
                                                        .toRotationMatrix());
                    Hft.linear() = rpy_intrinsic_to_mat(Eigen::Vector3d(rpy.y(), -(rpy.z() - M_PI), rpy.x()));

                    auto record = std::make_unique<OdometryRecord>();

                    record->timestamp       = sensors.timestamp;
                    record->velocity_target = walk_state.velocity_target;
                    record->accelerometer   = sensors.accelerometer;
                    record->gyroscope       = sensors.gyroscope;

                    for (const auto& servo : sensors.servo) {
                        record->present_position.push_back(servo.present_position);
                        record->present_velocity.push_back(servo.present_velocity);
                        record->goal_position.push_back(servo.goal_position);
                    }

                    record->Htw            = sensors.Htw;
                    record->Htw_kinematic  = sensors.Htw_kinematic;

                    if (!ground_truth_initialised) {
                        ground_truth_Hfw.translation().head<2>() = Hft.translation().head<2>();
                        ground_truth_Hfw.translation()[2]        = 0;
                        double yaw                               = mat_to_rpy_intrinsic(Hft.rotation()).z();
                        ground_truth_Hfw.linear()                = rpy_intrinsic_to_mat(Eigen::Vector3d(0, 0, yaw));
                        ground_truth_initialised                 = true;
                    }

                    Eigen::Isometry3d Htw_gt = Hft.inverse() * ground_truth_Hfw;
                    record->Htw_ground_truth = Htw_gt.matrix();
                    record->Hft              = Hft.matrix();

                    emit(std::move(record));
                    break;
                }
            });
    }

}  // namespace module::tools
