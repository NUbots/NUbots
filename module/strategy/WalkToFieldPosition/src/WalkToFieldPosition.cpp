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
#include "message/strategy/StandStill.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::input::Sensors;
    using message::localisation::Field;
    using message::planning::TurnOnSpot;
    using message::planning::WalkTo;
    using message::strategy::StandStill;
    using WalkToFieldPositionTask = message::strategy::WalkToFieldPosition;

    WalkToFieldPosition::WalkToFieldPosition(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("WalkToFieldPosition.yaml").then([this](const Configuration& config) {
            // Use configuration here from file WalkToFieldPosition.yaml
            this->log_level               = config["log_level"].as<NUClear::LogLevel>();
            cfg.stop_tolerance_position   = config["stop_tolerance"]["position"].as<double>();
            cfg.stop_tolerance_heading    = config["stop_tolerance"]["heading"].as<double>();
            cfg.resume_tolerance_position = config["resume_tolerance"]["position"].as<double>();
            cfg.resume_tolerance_heading  = config["resume_tolerance"]["heading"].as<double>();
        });

        on<Provide<WalkToFieldPositionTask>, With<Field>, With<Sensors>, Every<30, Per<std::chrono::seconds>>>().then(
            [this](const WalkToFieldPositionTask& walk_to_field_position, const Field& field, const Sensors& sensors) {
                // Get the transformation from robot {r} space to field {f} space
                const Eigen::Isometry3d Hfr = field.Hfw * sensors.Hrw.inverse();

                // Get the vector from field to desired point
                const Eigen::Vector3d rPFf(walk_to_field_position.rPFf.x(), walk_to_field_position.rPFf.y(), 0.0);

                // Create a unit vector in the direction of the desired heading in field space
                const Eigen::Vector3d uHFf(std::cos(walk_to_field_position.heading),
                                           std::sin(walk_to_field_position.heading),
                                           0.0);

                // Transform the field position from field {f} space to robot {r} space
                const Eigen::Vector3d rPRr(Hfr.inverse() * rPFf);

                // Compute the current position error and heading error in field {f} space
                const double position_error = (Hfr.translation().head(2) - rPFf.head(2)).norm();
                Eigen::Vector2d uXRf        = Hfr.rotation().col(0).head<2>();
                const double heading_error  = std::acos(std::max(-1.0, std::min(1.0, uXRf.dot(uHFf.head<2>()))));

                // If we have stopped but our position and heading error is now above resume tolerance, resume walking
                if (stopped && position_error < cfg.resume_tolerance_position
                    && heading_error < cfg.resume_tolerance_heading) {
                    stopped = true;
                    return;
                }

                // If the error in the desired field position and heading is low enough, don't do anything
                if (!stopped && position_error < cfg.stop_tolerance_position
                    && heading_error < cfg.stop_tolerance_heading) {
                    stopped = true;
                    return;
                }

                // If we are at position, but not at heading, then rotate in place in the direction of the desired
                // heading
                log<NUClear::INFO>("cfg.stop_tolerance_position: {}", cfg.stop_tolerance_position);
                if (position_error < cfg.stop_tolerance_position) {
                    // Rotate the desired heading in field {f} space to robot space
                    const Eigen::Vector3d uHRr(Hfr.inverse().linear() * uHFf);
                    const double desired_heading = std::atan2(uHRr.y(), uHRr.x());
                    bool clockwise               = desired_heading < 0;
                    // Emit a TurnOnSpot task
                    if (heading_error > cfg.stop_tolerance_heading) {
                        emit<Task>(std::make_unique<TurnOnSpot>(clockwise));
                        log<NUClear::INFO>("Rotating in place to desired heading");
                    }
                }
                // Otherwise, walk directly to the field position
                else {
                    log<NUClear::INFO>("Walking directly to field position");
                    const double desired_heading = std::atan2(rPRr.y(), rPRr.x());
                    emit<Task>(std::make_unique<WalkTo>(rPRr, desired_heading));
                    emit(std::make_unique<WalkTo>(rPRr, desired_heading));
                }

                if (log_level <= NUClear::INFO) {
                    log<NUClear::INFO>("Position error: ", position_error, " Heading error: ", heading_error);
                }
            });
    }

}  // namespace module::strategy
