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
#include "WalkToReadyPosition.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/skill/Walk.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::input::Sensors;
    using message::localisation::Field;
    using message::planning::WalkTo;
    using WalkToReadyPositionTask = message::strategy::WalkToReadyPosition;
    using message::skill::Walk;

    WalkToReadyPosition::WalkToReadyPosition(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("WalkToReadyPosition.yaml").then([this](const Configuration& config) {
            // Use configuration here from file WalkToReadyPosition.yaml
            this->log_level       = config["log_level"].as<NUClear::LogLevel>();
            cfg.stop_threshold    = config["stop_threshold"].as<double>();
            cfg.stopped_threshold = config["stopped_threshold"].as<double>();
        });

        on<Start<WalkToReadyPositionTask>>().then([this] { current_threshold = cfg.stop_threshold; });

        on<Provide<WalkToReadyPositionTask>, With<Field>, With<Sensors>>().then(
            [this](const WalkToReadyPositionTask& walk_to_field_position, const Field& field, const Sensors& sensors) {
                // Transform from desired field position into robot space
                Eigen::Isometry3d Hrd = sensors.Hrw * field.Hfw.inverse() * walk_to_field_position.Hfd;

                double translational_error = Hrd.translation().norm();
                log<NUClear::INFO>("translational_error:", translational_error);


                // Get position of robot on field
                Eigen::Isometry3d Hfr          = field.Hfw * sensors.Hrw.inverse();
                Eigen::Vector3d robot_unit_x   = Hfr.linear().col(0);
                Eigen::Vector3d desired_unit_x = walk_to_field_position.Hfd.linear().col(0);
                double desired_heading         = std::atan2(desired_unit_x.y(), desired_unit_x.x());
                double measured_heading        = std::atan2(robot_unit_x.y(), robot_unit_x.x());

                double angle_error = std::abs(desired_heading - measured_heading);
                // Normalize the angle error to be within the range [-pi, pi]
                angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error));


                if (translational_error < current_threshold && std::abs(angle_error) < current_threshold) {
                    emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()));
                    current_threshold = cfg.stopped_threshold;
                }
                else {
                    emit<Task>(std::make_unique<WalkTo>(Hrd));
                }
            });
    }

}  // namespace module::strategy
