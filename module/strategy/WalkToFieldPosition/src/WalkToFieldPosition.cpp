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
#include "WalkToFieldPosition.hpp"

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
    using message::skill::Walk;
    using WalkToFieldPositionTask = message::strategy::WalkToFieldPosition;

    WalkToFieldPosition::WalkToFieldPosition(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("WalkToFieldPosition.yaml").then([this](const Configuration& config) {
            // Use configuration here from file WalkToFieldPosition.yaml
            this->log_level                    = config["log_level"].as<NUClear::LogLevel>();
            cfg.stop_threshold                 = config["stop_threshold"].as<double>();
            cfg.stopped_threshold              = config["stopped_threshold"].as<double>();
            cfg.min_stop_time                  = config["min_stop_time"].as<double>();
            cfg.min_resume_time                = config["min_resume_time"].as<double>();
        });

        on<Start<WalkToFieldPositionTask>>().then([this] {
            current_threshold = cfg.stop_threshold;
            in_stopping_state = false;
            in_resuming_state = false;
        });

        on<Provide<WalkToFieldPositionTask>, With<Field>, With<Sensors>>().then(
            [this](const WalkToFieldPositionTask& walk_to_field_position, const Field& field, const Sensors& sensors) {
                // Transform from desired field position into robot space
                Eigen::Isometry3d Hrd      = sensors.Hrw * field.Hfw.inverse() * walk_to_field_position.Hfd;
                double translational_error = Hrd.translation().norm();

                // Compute the error between the desired heading and the measured heading
                Eigen::Isometry3d Hfr          = field.Hfw * sensors.Hrw.inverse();
                Eigen::Vector3d desired_unit_x = walk_to_field_position.Hfd.linear().col(0);
                double desired_heading         = std::atan2(desired_unit_x.y(), desired_unit_x.x());
                double measured_heading        = std::atan2(Hfr.linear().col(0).y(), Hfr.linear().col(0).x());

                double angle_error = std::abs(desired_heading - measured_heading);
                // Normalize the angle error to be within the range [-pi, pi]
                angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error));

                auto now = std::chrono::steady_clock::now();
                bool within_stop_threshold = translational_error < current_threshold && std::abs(angle_error) < current_threshold;

                // State machine for stopping/resuming with time-based hysteresis
                if (within_stop_threshold && walk_to_field_position.stop_at_target) {
                    if (!in_stopping_state) {
                        // Just entered stopping threshold
                        entered_stop_threshold = now;
                        in_stopping_state = true;
                        in_resuming_state = false;
                        log<DEBUG>("Entered stopping threshold");
                    }

                    // Check if we've been within threshold long enough to stop
                    auto time_in_threshold = std::chrono::duration<double>(now - entered_stop_threshold).count();

                    if (time_in_threshold >= cfg.min_stop_time) {
                        emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()));
                        // Increase the threshold to the stopped threshold to prevent oscillations
                        current_threshold = cfg.stopped_threshold;
                        log<DEBUG>("Stopped at field position after", time_in_threshold, "seconds");
                    }
                    else {
                        log<DEBUG>("Walking to field position (within threshold for", time_in_threshold, "seconds)");
                        emit<Task>(std::make_unique<WalkTo>(Hrd));
                    }
                }
                else {
                    if (in_stopping_state) {
                        // Just left stopping threshold
                        left_stop_threshold = now;
                        in_stopping_state = false;
                        in_resuming_state = true;
                        log<DEBUG>("Left stopping threshold");
                    }

                    if (in_resuming_state) {
                        // Check if we've been outside threshold long enough to resume
                        auto time_outside_threshold = std::chrono::duration<double>(now - left_stop_threshold).count();

                        if (time_outside_threshold >= cfg.min_resume_time) {
                            // Reset to original threshold and resume walking
                            current_threshold = cfg.stop_threshold;
                            in_resuming_state = false;
                            log<DEBUG>("Resuming walking after", time_outside_threshold, "seconds");
                        }
                    }

                    log<DEBUG>("Walking to field position");
                    emit<Task>(std::make_unique<WalkTo>(Hrd));
                }
            });
    }

}  // namespace module::strategy
