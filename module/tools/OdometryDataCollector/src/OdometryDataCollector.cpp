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
#include "OdometryDataCollector.hpp"

#include "extension/Configuration.hpp"

#include "message/behaviour/state/WalkState.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"
#include "message/localisation/OdometryRecord.hpp"

#include "utility/math/euler.hpp"

namespace module::tools {

    using extension::Configuration;
    using message::behaviour::state::WalkState;
    using message::input::Sensors;
    using message::localisation::RobotPoseGroundTruth;
    using message::localisation::OdometryRecord;
    
    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::math::euler::rpy_intrinsic_to_mat;

    OdometryDataCollector::OdometryDataCollector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("OdometryDataCollector.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Trigger<Sensors>, With<WalkState>, With<RobotPoseGroundTruth>>().then(
            [this](const Sensors& sensors,
                   const WalkState& walk_state,
                   const RobotPoseGroundTruth& robot_pose_ground_truth) {

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

                record->Htw = sensors.Htw;

                Eigen::Isometry3d Hft = Eigen::Isometry3d(robot_pose_ground_truth.Hft);
                if (!ground_truth_initialised) {
                    ground_truth_Hfw.translation().head<2>() = Hft.translation().head<2>();
                    ground_truth_Hfw.translation()[2]        = 0;
                    double yaw                               = mat_to_rpy_intrinsic(Hft.rotation()).z();
                    ground_truth_Hfw.linear()                = rpy_intrinsic_to_mat(Eigen::Vector3d(0, 0, yaw));
                    ground_truth_initialised                 = true;
                }

                Eigen::Isometry3d Htw_gt   = Hft.inverse() * ground_truth_Hfw;
                record->Htw_ground_truth   = Htw_gt.matrix();
                record->Hft                = robot_pose_ground_truth.Hft;

                emit(std::move(record));
            });
    }
}  // namespace module::tools
